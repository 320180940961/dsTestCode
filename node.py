#!/usr/bin/env python3
"""
该模块定义了用于路径记录和导航的核心类。
导航逻辑已升级为“先转向，再直行”的闭环控制模式。
"""
import copy
import json
import math
import os
from datetime import datetime
from threading import Event, Thread

import numpy as np
import rospy
from ros_ht_msg.msg import ht_control, Lift_state, Lift_control
from ros_gt_desk.msg import MotorStatus, SliderStatus, MotorControl, SliderControl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion

# --- 全局常量 ---
NAV_STATUS_IDLE = "闲置"
NAV_STATUS_RUNNING = "导航中"
NAV_STATUS_DONE = "已完成"


class Naver(Thread):
    """
    导航器线程类。
    管理路径记录、导航任务执行、ROS消息订阅和状态更新。
    """

    def __init__(self, recorder, mode="record", dir_mode="forward", mvto_point=None):
        """
        初始化 Naver 实例。

        :param recorder: PathRecorder 的实例，用于文件操作。
        :param mode: "record" 或 "navigation"。
        :param dir_mode: "forward" 或 "reverse"。
        :param mvto_point: 导航到指定点的索引。
        """
        super().__init__(daemon=True)

        # 核心属性
        self.recorder = recorder
        self.mode = mode
        self.dir_mode = dir_mode
        self.mvto_point = mvto_point
        self.stop_event = Event()
        self.log = []

        # 为电机指令添加时间戳变量，并初始化为0
        self.last_motor_command_time = 0.0

        self.file_logger = None

        # 状态属性
        self.status = NAV_STATUS_IDLE
        self.current_position = None
        self.current_target = None
        self.current_target_index = None

        # 【新增】: 暂停/恢复功能所需属性
        self.pause_event = Event()  # False为运行, True为暂停
        self.is_paused = False

        # 路径点管理
        self.points = copy.deepcopy(self.recorder.load())
        self.modified = False

        # ROS 消息存储
        self.latest_navsat_fix = None
        self.latest_odometry = None
        self.latest_slider_status = None
        self.latest_motor_status = None
        self.latest_lift_state = None

        # ROS 订阅者与发布者
        self.sub1 = rospy.Subscriber("/gps/filtered", NavSatFix, self.navsat_fix_callback)
        self.sub2 = rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)
        self.sub3 = rospy.Subscriber("/gt_desk/slider_status", SliderStatus, self.slider_callback)
        self.sub4 = rospy.Subscriber("/gt_desk/mower_status", MotorStatus, self.motor_callback)
        self.sub5 = rospy.Subscriber("/Lift_Motion", Lift_state, self.lift_callback)
        self.commander = BasePlateCommand(log_container=self.log)

        self.slider_pub = rospy.Publisher("/gt_desk/slider_control", SliderControl, queue_size=10)
        self.motor_pub = rospy.Publisher("/gt_desk/mower_control", MotorControl, queue_size=10)
        self.lift_pub = rospy.Publisher("/Lift_Control", Lift_control, queue_size=10)

        # --- 可调校的闭环控制参数 ---
        self.arrival_threshold = 200      # 到达阈值（mm）, 可适当放宽
        self.turn_arrival_threshold = 5.0 # 转向完成阈值（度），小于此值认为车头已对准
        self.kp_turn = 1.5                # 转向比例增益
        # self.kp_turn = 0.5                # 转向比例增益
        self.forward_speed = 100           # 前进速度 mm/s
        self.reverse_speed = 100           # 倒车速度 mm/s
        # self.max_turn_cmd = 174           # 最大转向控制值

        # 直行时，每隔多少秒检查一次航向
        self.recalibration_interval = 3.0

        # 定义航向角偏移量 (RTK天线方向 与 车辆前进方向 的夹角)
        self.YAW_OFFSET_DEG = 90.0
        # self.YAW_OFFSET_DEG = 0.0

        # 【新增】: 用于检测机器人是否卡住的参数
        self.stuck_timeout = 3.0  # 几秒后开始检测 (秒)
        self.stuck_angle_threshold = 1.0  # 转向时，角度变化小于此值(度)则认为卡住
        self.stuck_distance_threshold = 0.1 # 直行时，移动距离小于此值(米)则认为卡住

        self.log.append(f"Naver in '{mode}' mode initialized.")

    def run(self):
        """线程主函数，根据模式执行不同任务。"""
        if self.mode == "navigation":
            self.run_navigation_task()
        elif self.mode == "record":
            rospy.spin()

    def send_sync_commands(self, target_point):
        """根据目标点中记录的状态，发送滑台、电机和升降台的控制指令"""
        # 1. 控制滑台
        if 'slider_y' in target_point:
            # 检查X轴差值，决定是执行移动还是强制归零
            should_zero = False
            if self.latest_slider_status:
                x1_pos = self.latest_slider_status.pos_x1
                x2_pos = self.latest_slider_status.pos_x2
                if abs(x1_pos - x2_pos) > 3:
                    self.log.append(f"[WARN] 滑台 X1({x1_pos}) 和 X2({x2_pos}) 差值大于3mm，将执行归零！")
                    should_zero = True
            else:
                self.log.append("[WARN] 无法获取滑台状态，跳过X轴差值检查。")

            slider_msg = SliderControl()
            # 假设模式2是移动到绝对位置: [mode:2, [pos,vel,acc]]
            if should_zero:
                # 执行归零操作
                slider_msg.mode = 2
                slider_msg.x = [0, 0, 0] 
                slider_msg.y = [0, 0, 0] 
            else:
                slider_msg.mode = 2
                slider_msg.y = [target_point['slider_y'], 0, 0] 
                slider_msg.x = [target_point['slider_x1'], 0, 0] 


            self.slider_pub.publish(slider_msg)

        # 2. 控制电机
        # 获取当前时间
        current_time = rospy.get_time()
        # 检查当前时间与上次发送时间的差值是否大于等于1秒
        if (current_time - self.last_motor_command_time) >= 1.0:
            if target_point['comment']=='start_motor':
                # mode 0：关 ；1：开
                # motor_msg = MotorControl(mode=1, voltage=0)
                motor_msg = MotorControl(mode=1)
                self.motor_pub.publish(motor_msg)
                # 发送后，立即更新时间戳为当前时间
                self.last_motor_command_time = current_time
            else:
                motor_msg = MotorControl(mode=0, voltage=0)
                self.motor_pub.publish(motor_msg)
                # 发送后，立即更新时间戳为当前时间
                self.last_motor_command_time = current_time
            

        # 3. 控制升降台
        if 'lift_height' in target_point:
            # 假设模式1是移动到指定高度
            lift_msg = Lift_control(mode=1, data=target_point['lift_height']//10)
            self.lift_pub.publish(lift_msg)

    def run_navigation_task(self):
        """执行一次完整的导航任务。"""
        self.status = NAV_STATUS_RUNNING
        self.log.append("正在获取初始定位...")
        wait_start_time = rospy.get_time()
        timeout = 5.0
        start_pos = self.get_current_position()

        while not start_pos and not self.stop_event.is_set() and (rospy.get_time() - wait_start_time < timeout):
            elapsed_time = rospy.get_time() - wait_start_time
            self.log.append(f"等待初始定位... ({elapsed_time:.1f}s)")
            rospy.sleep(0.5)
            start_pos = self.get_current_position()

        if not start_pos:
            self.log.append("错误: 获取初始位置失败，导航中止。")
            self.status = NAV_STATUS_IDLE
            return

        self.log.append("[ACTION] 导航前执行滑台归零...")
        try:
            # 创建一个归零指令消息 (mode=1)
            zeroing_msg = SliderControl(mode=2, x=[0, 0, 0], y=[0, 0, 0])
            self.slider_pub.publish(zeroing_msg)
            self.log.append("滑台归零指令已发送，请等待硬件完成操作。")
            rospy.sleep(5) # 预留5秒给硬件执行归零，可根据实际情况调整
        except Exception as e:
            self.log.append(f"[ERROR] 发送滑台归零指令失败: {e}")

        try:
            self._prepare_navigation(start_pos)
            if not self.nav_points:
                self.status = NAV_STATUS_DONE
                return
        except Exception as e:
            self.log.append(f"错误: 导航准备失败 - {e}")
            self.status = NAV_STATUS_IDLE
            return

        current_pos = start_pos
        for target in self.nav_points:
            if self.stop_event.is_set():
                self.log.append("导航被用户中断")
                break

            self.current_target = target
            try:
                self.current_target_index = self.points.index(target)
                idx_info = f"{self.current_target_index+1}/{len(self.points)}"
            except ValueError:
                self.current_target_index = None
                idx_info = "未知索引"

            arrived = self.go_to_point_turn_then_drive(current_pos, target, idx_info)

            if not arrived:
                self.log.append(f"前往点 {idx_info} 的任务被中断或失败。")
                break

            current_pos = self.get_current_position() or target
            self.log.append(f"确认到达 {idx_info} 号点。")

            # 【调用新增功能】: 到达后，校准车头朝向
            self.align_heading_to_waypoint_yaw(target, idx_info)
            if self.stop_event.is_set():
                self.log.append("姿态校准时被用户中断")
                break

            # # 【新增逻辑】: 到达后，再执行设备指令
            # self.log.append(f"开始执行目标点 {idx_info} 的设备指令...")
            # self.send_sync_commands(target)
            # # 增加一个延时，以等待滑台、升降台等物理设备完成动作
            # # 这个时间可以根据您的设备实际响应速度进行调整
            # self.log.append("等待设备动作完成 (5秒)...")
            # rospy.sleep(5.0)
            # self.log.append("设备动作完成。")

        self.status = NAV_STATUS_DONE
        if not self.stop_event.is_set():
            self.log.append(f"{self.dir_mode} 导航任务完成")

        self.current_target = None
        self.current_target_index = None
        self.stop_robot()
        self.cleanup_subscribers()

    # 【新增】: 新增用于检查和处理暂停状态的辅助函数 _wait_if_paused
    def _wait_if_paused(self):
        """检查并处理暂停事件。"""
        if self.pause_event.is_set():
            self.commander.send_stop_command()
            self.log.append("导航已暂停。")
            while self.pause_event.is_set():
                rospy.sleep(0.2)
                if self.stop_event.is_set():
                    return False  # 如果在暂停时收到停止信号，则退出
            self.log.append("导航已恢复。")
        return True

    # 【新增功能】: 到达目标点后，将车头朝向对准该点记录的朝向
    def align_heading_to_waypoint_yaw(self, target_pos, target_info):
        """
        一个独立的函数，负责在到达目标点后，将车头旋转到与该点记录的朝向一致。
        允许正向或180度反向对准，取最短路径。
        """
        # 检查目标点是否有'yaw'数据
        if 'yaw' not in target_pos:
            self.log.append(f"目标点 {target_info} 无朝向数据，跳过姿态校准。")
            return True

        self.log.append(f"开始校准到达点 {target_info} 的姿态...")
        control_rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            # 检查是否需要暂停或完全停止
            if not self._wait_if_paused(): return False
            if self.stop_event.is_set(): return False
            
            current_pos = self.get_current_position()

            # --- 核心计算逻辑 ---
            current_yaw = current_pos.get('yaw', 0.0)
            target_yaw = target_pos.get('yaw', 0.0)

            # 计算正向和反向的角度差
            error_forward = target_yaw - current_yaw
            error_reverse = (target_yaw - 180.0) - current_yaw # 也可写成 (target_yaw + 180.0)

            # 将角度差标准化到[-180, 180]范围
            error_forward = (error_forward + 180) % 360 - 180
            error_reverse = (error_reverse + 180) % 360 - 180

            # 选择绝对值更小的那个作为最终的角度差
            yaw_error_deg = error_forward if abs(error_forward) < abs(error_reverse) else error_reverse
            
            self.log.append(f"姿态校准中... 目标点: {target_info}, 角度差: {yaw_error_deg:.1f}°")
            
            # 判断转向是否完成
            if abs(yaw_error_deg) < self.turn_arrival_threshold:
                self.log.append(f"到达点 {target_info} 姿态校准完成。")
                self.commander.send_stop_command()
                return True # 成功完成

            yaw_error_rad = math.radians(yaw_error_deg)

            # 2. Minimum rotate speed 100/1000 rad/s
            if yaw_error_rad >0:
                # turn_cmd = np.clip(angular_velocity_cmd, 100, self.max_turn_cmd)
                turn_cmd = np.clip(yaw_error_rad, math.pi*0.5, math.pi*0.95)
            else:
                turn_cmd = np.clip(yaw_error_rad, -math.pi*0.95, -math.pi*0.5)
            
            
            angular_velocity_cmd = turn_cmd * 200 # 乘以系数并取反以匹配坐标系

            self.commander.send_move_command(0, angular_velocity_cmd)
            control_rate.sleep()

        return False

    def align_heading_to_target(self, start_pos, target_pos, target_info):
        """
        一个独立的函数，负责原地旋转直到车头对准目标。
        """
        self.log.append("正在执行原地转向...")
        control_rate = rospy.Rate(10)
        turn_start_time = rospy.get_time()
        initial_turn_pos = self.get_current_position() or start_pos
        stuck_warning_issued = False

        while not rospy.is_shutdown():
            # 检查是否需要暂停或完全停止
            if not self._wait_if_paused(): return False
            if self.stop_event.is_set(): return False
            
            self.send_sync_commands(target_pos)

            # 获取最新位置并计算角度差
            current_pos = self.get_current_position() or start_pos
            yaw_error_rad, _ = self.compute_heading_error(current_pos, target_pos)
            yaw_error_deg = math.degrees(yaw_error_rad)

            self.log.append(f"转向中... 目标点: {target_info}, 角度差: {yaw_error_deg:.1f}°")

            # 卡住检测逻辑
            if not stuck_warning_issued and (rospy.get_time() - turn_start_time > self.stuck_timeout):
                yaw_change = abs(current_pos['yaw'] - initial_turn_pos['yaw'])
                if yaw_change > 180: yaw_change = 360 - yaw_change
                
                if yaw_change < self.stuck_angle_threshold:
                    self.log.append("[WARN] 机器人可能无法正常转向！")
                    stuck_warning_issued = True
            
            # 判断转向是否完成
            if abs(yaw_error_deg) < self.turn_arrival_threshold:
                self.log.append("转向对准完成。")
                self.commander.send_stop_command()
                return # 成功完成转向

            # angular_velocity_cmd = self.kp_turn * yaw_error_rad
            # turn_cmd = np.clip(angular_velocity_cmd * 200, -self.max_turn_cmd, self.max_turn_cmd)
            #  turn_cmd = np.clip(angular_velocity_cmd * 10, -self.max_turn_cmd, self.max_turn_cmd) 
            # 1 .to +- pi
            # turn_cmd = np.clip(angular_velocity_cmd , -math.pi*0.95, math.pi*0.95) * 1000

            # 2. Minimum rotate speed 100/1000 rad/s
            if yaw_error_rad >0:
                # turn_cmd = np.clip(angular_velocity_cmd, 100, self.max_turn_cmd)
                turn_cmd = np.clip(yaw_error_rad, math.pi*0.5, math.pi*0.95)
            else:
                turn_cmd = np.clip(yaw_error_rad, -math.pi*0.95, -math.pi*0.5)
                
                # turn_cmd = np.clip(angular_velocity_cmd, -self.max_turn_cmd, -100 )


            self.log.append(f"yaw_error_deg：{yaw_error_deg}")
            
            # Trasn GEO coord to ROS coord
            self.commander.send_move_command(0, -turn_cmd * 200)
            control_rate.sleep()

    def go_to_point_turn_then_drive(self, start_pos, target_pos, target_info):
        """采用“先原地转向，再直线行驶”的策略导航到目标点。"""
        self.log.append(f"开始导航至目标点 {target_info}")
        # self.log.append("*"*10)
        # self.log.append(f"Lat {target_pos['lat']} Lng{target_pos['lng']}")
        # self.log.append("*"*10)
        # control_rate = rospy.Rate(10)
        control_rate = rospy.Rate(2)

        # --- 阶段一: 原地旋转，对准目标 ---
        self.align_heading_to_target(start_pos, target_pos, target_info)
        if self.stop_event.is_set():
            return False

        # 确保车辆在进入下一阶段前已稳定
        self.commander.send_stop_command()
        rospy.sleep(0.5)

        # --- 阶段二: 直线行驶，并周期性校准航向 ---
        self.log.append("正在直线行驶...")

        # 【修改点】: 增加直行卡住检测逻辑
        drive_start_time = rospy.get_time()
        initial_drive_pos = self.get_current_position()
        stuck_warning_issued = False
        last_recalibration_time = rospy.get_time() # 初始化中途校准计时器

        current_pos = self.get_current_position()
        _, reverse_motion = self.compute_heading_error(current_pos, target_pos)
        linear_velocity = -self.reverse_speed if reverse_motion else self.forward_speed

        while not rospy.is_shutdown():
            # 【新增】: 在核心导航循环中调用暂停检查
            if not self._wait_if_paused(): 
                return False
            
            if self.stop_event.is_set():
                return False

            self.send_sync_commands(target_pos)
            current_pos = self.get_current_position()
            distance = self._calc_distance(current_pos['lat'], current_pos['lng'], target_pos['lat'], target_pos['lng'])

            self.log.append(f"行驶中... 目标点: {target_info}, 距离目标: {distance:.2f}mm 方向：{reverse_motion}")

            # 检查是否卡住
            if not stuck_warning_issued and (rospy.get_time() - drive_start_time > self.stuck_timeout):
                distance_moved = self._calc_distance(initial_drive_pos['lat'], initial_drive_pos['lng'], current_pos['lat'], current_pos['lng'])
                if distance_moved < self.stuck_distance_threshold:
                    self.log.append("[WARN] 机器人可能无法直线行驶！")
                    stuck_warning_issued = True # 只警告一次
            
            # 中途航向校准逻辑
            if rospy.get_time() - last_recalibration_time > self.recalibration_interval:
                self.log.append("...执行中途航向检查...")
                yaw_error_rad_check, _ = self.compute_heading_error(current_pos, target_pos)
                yaw_error_deg_check = math.degrees(yaw_error_rad_check)
                
                if abs(yaw_error_deg_check) > self.turn_arrival_threshold:
                    self.log.append(f"航向偏差 ({yaw_error_deg_check:.1f}°) 过大，进行中途校准。")
                    self.commander.send_stop_command()
                    rospy.sleep(0.5)
                    current_pos = self.get_current_position()
                    # 再次调用转向函数进行校准
                    self.align_heading_to_target(current_pos, target_pos, target_info)
                    self.log.append("中途校准完成，继续直线行驶。")
                
                last_recalibration_time = rospy.get_time() # 重置计时器

            if distance < self.arrival_threshold:
                # self.align_heading_to_target(start_pos, target_pos, target_info)
                self.log.append(f"距离目标点({distance:.2f} mm)小于阈值，认为到达。")
                self.commander.send_stop_command()
                return True

            # 直行中的微小方向修正
            # yaw_error_rad, _ = self.compute_heading_error(current_pos, target_pos)
            # angular_velocity_cmd = self.kp_turn * yaw_error_rad
            # turn_cmd = np.clip(angular_velocity_cmd * 100, -self.max_turn_cmd, self.max_turn_cmd)
            
            # self.commander.send_move_command(linear_velocity, turn_cmd)
            # control_rate.sleep()
            

            # self.commander.send_move_command(linear_velocity, turn_cmd)
            self.commander.send_move_command(linear_velocity, 0)
            control_rate.sleep()
        return False

    def compute_heading_error(self, start, target):
        """计算当前航向与目标方向的角度差。"""
        lat1, lon1, raw_yaw_deg = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])

        # 应用航向角偏移量，得到真实的车辆前进方向
        # travel_yaw_deg = raw_yaw_deg + self.YAW_OFFSET_DEG
        # ROS角度(弧度) = -地理方位角(弧度) + π/2
        self.log.append(f"Cur ROS: {raw_yaw_deg} ")
        travel_yaw_deg = -raw_yaw_deg + 90
        # travel_yaw_deg=raw_yaw_deg+90
        self.log.append(f"Cur GEO: {travel_yaw_deg} ")

        lat1_rad, lon1_rad, lat2_rad, lon2_rad, current_travel_yaw_rad = map(
            math.radians, [lat1, lon1, lat2, lon2, travel_yaw_deg]
        )

        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        
        target_angle_rad = math.atan2(y, x)
        # target_angle_rad = math.atan2(x, y)

        # 使用修正后的航向进行误差计算
        yaw_error = target_angle_rad - current_travel_yaw_rad
        # yaw_error = -target_angle_rad + current_travel_yaw_rad
        # 标准化到 [-pi, pi] 范围
        self.log.append(f"Tar: {target_angle_rad * 180 /math.pi} Cur: {current_travel_yaw_rad * 180 /math.pi}")
        # self.log.append(f"Yaw Old {yaw_error}")
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        # self.log.append(f"Yaw Fix {yaw_error}")

        reverse_motion = abs(yaw_error) > (math.pi / 2)
        # reverse_motion = abs(yaw_error) > (math.pi)
        if reverse_motion:
            yaw_error = yaw_error - math.copysign(math.pi, yaw_error)

        return yaw_error, reverse_motion
        # return yaw_error, reverse_motion

    def stop_robot(self):
        if self.commander:
            self.commander.send_stop_command()

    # 【新增】: 在 Naver 类中新增控制暂停/恢复的公共方法
    def pause_navigation(self):
        """暂停导航。"""
        if self.status == NAV_STATUS_RUNNING and not self.is_paused:
            self.is_paused = True
            self.pause_event.set() # 设置事件，通知循环暂停

    def resume_navigation(self):
        """恢复导航。"""
        if self.status == NAV_STATUS_RUNNING and self.is_paused:
            self.is_paused = False
            self.pause_event.clear() # 清除事件，让循环继续

    def navsat_fix_callback(self, msg):
        self.latest_navsat_fix = msg

    def odometry_callback(self, msg):
        self.latest_odometry = msg

    # 新增： ADD 滑台、电机、升降台
    def slider_callback(self, msg):
        """存储最新的滑台状态"""
        self.latest_slider_status = msg

    def motor_callback(self, msg):
        """存储最新的割草电机状态"""
        self.latest_motor_status = msg

    def lift_callback(self, msg):
        """存储最新的升降台状态"""
        self.latest_lift_state = msg

    def _get_combined_position(self):
        if self.latest_navsat_fix is None or self.latest_odometry is None:
            return None

        odom_pose = self.latest_odometry.pose.pose
        pos = odom_pose.position
        q = odom_pose.orientation

        try:
            _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_deg = math.degrees(yaw_rad)
        except Exception:
            yaw_deg = 0.0

        return {
            "lat": self.latest_navsat_fix.latitude,
            "lng": self.latest_navsat_fix.longitude,
            "yaw": yaw_deg,
            "x": pos.x,
            "y": pos.y,
            "angle": yaw_deg,
            "time": datetime.now().strftime("%H:%M:%S")
        }

    def get_current_position(self):
        self.current_position = self._get_combined_position()
        return self.current_position

    def add_point(self, comment=""):
        point = self.get_current_position()
        if not point:
            self.log.append("无法获取当前位置，添加点失败")
            return

        # 【修改点】: 在point字典中增加新的键值对
        if self.latest_slider_status:
            point['slider_x1'] = self.latest_slider_status.pos_x1
            point['slider_x2'] = self.latest_slider_status.pos_x2
            point['slider_y'] = self.latest_slider_status.pos_y

        if self.latest_motor_status:
            point['motor_power'] = self.latest_motor_status.power
            point['motor_motor'] = self.latest_motor_status.motor
            point['motor_voltage'] = self.latest_motor_status.voltage


        if self.latest_lift_state:
            point['lift_height'] = self.latest_lift_state.hight
            point['lift_state'] = self.latest_lift_state.lift_state

        point["comment"] = comment
        self.points.append(point)
        self.modified = True
        self.log.append(f"添加点: ({point['lat']:.6f}, {point['lng']:.6f})")

    def stop_navigation(self):
        self.stop_event.set()
        self.log.append("导航停止信号已发送")

    def cleanup_subscribers(self):
        if hasattr(self, 'sub1') and self.sub1:
            self.sub1.unregister()
        if hasattr(self, 'sub2') and self.sub2:
            self.sub2.unregister()
        if hasattr(self, 'sub3') and self.sub3:
            self.sub3.unregister()
        if hasattr(self, 'sub4') and self.sub4:
            self.sub4.unregister()
        if hasattr(self, 'sub5') and self.sub5:
            self.sub5.unregister()
        if self.commander:
            self.commander.cleanup()

        # 【新增功能开始】: 在任务结束时，将内存中的全部日志保存到文件
        if self.recorder and self.recorder.listname and self.log:
            try:
                # 1. 定义日志文件存放目录和文件名
                base_dir = os.path.dirname(os.path.abspath(__file__))
                log_dir = os.path.join(base_dir, "logs")
                os.makedirs(log_dir, exist_ok=True) # 确保目录存在

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                summary_filename = f"{self.recorder.listname}_{timestamp}_summary.log"
                summary_filepath = os.path.join(log_dir, summary_filename)

                # 2. 将self.log列表内容写入文件
                with open(summary_filepath, 'w', encoding='utf-8') as f:
                    # 使用 "\n".join() 将列表中的所有字符串用换行符连接成一个大字符串
                    f.write("\n".join(self.log))
                
                # 3. 在屏幕上提示用户保存成功
                self.log.append(f"会话日志摘要已保存到: {summary_filename}")

            except Exception as e:
                # 在屏幕上提示错误
                self.log.append(f"[ERROR] 保存日志摘要失败: {e}")
        # 【新增功能结束】

        # 关闭实时文件记录器 (这是我们之前添加的功能，保持不变)
        if self.file_logger:
            self.log.append("文件记录器已关闭。")
            self.file_logger.close()

    def _prepare_navigation(self, current_pos, mvto_point=None):
        if not self.points:
            raise ValueError("路径点为空")

        if mvto_point is not None:
            self._prepare_mvto_navigation(current_pos, mvto_point)
        elif self.dir_mode == "reverse":
            self._prepare_reverse_navigation(current_pos)
        else:
            self._prepare_forward_navigation(current_pos)

        self.log.append(f"导航准备完成 ({self.dir_mode}方向, {len(self.nav_points)}点)")

    def _prepare_forward_navigation(self, current_pos):
        start_index = self._find_nearest_point(current_pos)
        self.log.append(f"正向导航: 从最近点({start_index})开始前往终点")
        self.nav_points = self.points[start_index:]

    def _prepare_reverse_navigation(self, current_pos):
        start_index = self._find_nearest_point(current_pos)
        self.log.append(f"逆向导航: 从最近点({start_index})开始返回起点")
        self.nav_points = self.points[start_index::-1]

    def _prepare_mvto_navigation(self, current_pos, target_index):
        if not (0 <= target_index < len(self.points)):
            raise ValueError(f"目标点索引{target_index}超出范围")

        target_point = self.points[target_index]
        self.log.append(f"指定目标点: {target_index} - ({target_point['lat']:.5f}, {target_point['lng']:.5f})")
        self.log.append(f"从当前点 ({current_pos['lat']:.5f}, {current_pos['lng']:.5f}) 开始规划路径...")
        self.nav_points = [current_pos, target_point]

    def _find_nearest_point(self, current_pos):
        return min(
            range(len(self.points)),
            key=lambda i: self._calc_distance(
                self.points[i]['lat'], self.points[i]['lng'],
                current_pos['lat'], current_pos['lng']
            )
        )

    def _calc_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])

        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c * 1000

    def save_points(self):
        if not self.modified:
            self.log.append("无修改，跳过保存")
            return
        self.recorder.save(self.points)
        self.modified = False
        self.log.append(f"保存{len(self.points)}个点到文件")

    def delete_point(self, index):
        if not (0 <= index < len(self.points)):
            self.log.append(f"无效索引: {index}")
            return 0
        self.points.pop(index)
        self.modified = True
        self.log.append(f"删除点 {index}")
        return len(self.points)

    def insert_point(self, index, comment=""):
        if not (0 <= index <= len(self.points)):
            self.log.append(f"无效索引: {index}")
            return None
        point = self.get_current_position()
        if not point:
            self.log.append("无法获取当前位置，插入点失败")
            return
        point["comment"] = comment
        self.points.insert(index, point)
        self.modified = True
        self.log.append(f"在位置{index}插入点")


class BasePlateCommand:
    """负责向机器人底盘发送具体运动指令的工具类。"""

    def __init__(self, log_container=None):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.log_container = log_container if log_container is not None else []

    def send_move_command(self, linear_velocity, turn_cmd):
        """发送单条移动指令。"""
        # Mode 1 轮速转速模式 2 线速度模式
        control_msg = ht_control(
            mode=2,
            x=int(linear_velocity),
            y=int(turn_cmd),
            stop=0
        )
        # print(f"[BaseComand] linear_velocity{linear_velocity} turn_cmd{turn_cmd}")
        self.log_container.append(f"[BaseComand] linear_velocity {linear_velocity} turn_cmd {turn_cmd}")
        self.pub.publish(control_msg)
    
    def send_stop_command(self):
        """发送单条停止指令。"""
        control_msg = ht_control(mode=1, x=0, y=0, stop=1)
        self.pub.publish(control_msg)
        self.log_container.append("车辆停止指令已发送")

    def cleanup(self):
        """注销 publisher，释放资源。"""
        if self.pub:
            self.pub.unregister()
            self.log_container.append("BasePlateCommand publisher 已注销。")


class PathRecorder:
    """负责将路径点数据以JSON格式加载和保存到文件。"""

    def __init__(self, listname):
        self.listname = listname.split('.')[0]
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.json_dir = os.path.join(base_dir, "paths_json")
        self.path_file = os.path.join(self.json_dir, f"{self.listname}.json")

        if not os.path.exists(self.json_dir):
            os.makedirs(self.json_dir)
            
    def load(self):
        """从文件加载路径点列表。"""
        if not os.path.exists(self.path_file):
            return []
            
        try:
            with open(self.path_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"加载路径失败: {str(e)}")
            return []
            
    def save(self, points):
        """将路径点列表保存到文件。"""
        try:
            with open(self.path_file, "w", encoding="utf-8") as f:
                json.dump(points, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            rospy.logerr(f"保存路径失败: {str(e)}")
            return False
