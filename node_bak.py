#!/usr/bin/env python3
"""
该模块定义了用于路径记录和导航的核心类。

- Naver: 一个线程类，负责处理记录和导航两种模式下的主要逻辑。
- BasePlateCommand: 一个工具类，负责向机器人底层发送具体的运动指令。
- PathRecorder: 一个工具类，负责路径点数据的加载和保存（JSON格式）。
"""
import copy
import json
import math
import os
from datetime import datetime
from threading import Event, Thread

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from ros_ht_msg.msg import ht_control
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
    根据不同的模式（record/navigation）执行不同的任务。
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

        # 状态属性
        self.status = NAV_STATUS_IDLE
        self.current_position = None
        self.current_target = None
        self.current_target_index = None

        # 路径点管理
        self.points = copy.deepcopy(self.recorder.load())
        self.modified = False

        # ROS 消息存储
        self.latest_navsat_fix = None
        self.latest_odometry = None

        # ROS 订阅者
        self.sub1 = rospy.Subscriber("/gps/filtered", NavSatFix, self.navsat_fix_callback)
        self.sub2 = rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)

        self.log.append(f"Naver in '{mode}' mode initialized.")

    def run(self):
        """线程主函数，根据模式执行不同任务。"""
        if self.mode == "navigation":
            self.run_navigation_task()
        elif self.mode == "record":
            # 在记录模式下，线程只负责保持ROS消息监听
            rospy.spin()

    def run_navigation_task(self):
        """执行一次完整的导航任务。"""
        self.status = NAV_STATUS_RUNNING

        # 1. 等待初始定位
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
            self.log.append("请检查 /gps/filtered 和 /odometry/filtered 话题是否正常发布。")
            self.status = NAV_STATUS_IDLE
            return

        # 2. 成功获取位置后，进行导航准备
        try:
            self._prepare_navigation(self.mvto_point)
            if not self.nav_points:
                self.status = NAV_STATUS_DONE
                return
        except Exception as e:
            self.log.append(f"错误: 导航准备失败 - {e}")
            self.status = NAV_STATUS_IDLE
            return

        # 3. 导航循环
        log_msg_map = {
            "forward": f"正向导航 (共{len(self.nav_points)}个点)",
            "reverse": f"逆向导航 (剩余{len(self.nav_points)}段)"
        }
        log_msg = log_msg_map.get(self.dir_mode, f"前往指定点 (共{len(self.nav_points)}段)")
        self.log.append(log_msg)

        current_pos = start_pos
        for target in self.nav_points:
            if self.stop_event.is_set():
                self.log.append("导航被用户中断")
                break

            self.current_target = target
            try:
                self.current_target_index = self.points.index(target)
                idx_info = f"{self.current_target_index}/{len(self.points)}"
            except ValueError:
                self.current_target_index = None
                idx_info = "未知索引"

            motion = self.compute_motion_params(current_pos, target)
            self.execute_motion(motion)

            next_pos = self.get_current_position()
            if next_pos:
                current_pos = next_pos
                self.log.append(f"到达 {idx_info} 号点: ({current_pos['lat']:.5f}, {current_pos['lng']:.5f})")
            else:
                current_pos = target
                self.log.append(f"到达 {idx_info} 号点 (位置更新失败)")

            rospy.sleep(0.1)

        # 4. 结束阶段
        self.status = NAV_STATUS_DONE
        if not self.stop_event.is_set():
            self.log.append(f"{self.dir_mode} 导航任务完成")

        self.current_target = None
        self.current_target_index = None
        self.stop_robot()
        self.cleanup_subscribers()

    def navsat_fix_callback(self, msg):
        self.latest_navsat_fix = msg

    def odometry_callback(self, msg):
        self.latest_odometry = msg

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

    def _prepare_navigation(self, mvto_point=None):
        if not self.points:
            raise ValueError("路径点为空")

        self.nav_points = self.points  # 默认值
        if mvto_point is not None:
            self._prepare_mvto_navigation(mvto_point)
        elif self.dir_mode == "reverse":
            self._prepare_reverse_navigation()
        else:
            self.log.append("使用默认的正向导航")

        self.log.append(f"导航准备完成 ({self.dir_mode}方向, {len(self.nav_points)}点)")

    def _prepare_reverse_navigation(self):
        current_pos = self.get_current_position()
        if current_pos:
            start_index = self._find_nearest_point(current_pos)
            self.log.append(f"定位到最近点 {start_index} 开始返回")
        else:
            start_index = len(self.points) - 1
            self.log.append("无法定位, 从终点开始返回")

        if not (0 <= start_index < len(self.points)):
            start_index = len(self.points) - 1

        self.nav_points = self.points[start_index::-1]

    def _prepare_mvto_navigation(self, target_index):
        if not (0 <= target_index < len(self.points)):
            raise ValueError(f"目标点索引{target_index}超出范围")

        target_point = self.points[target_index]
        self.log.append(f"指定目标点: {target_index} - ({target_point['lat']:.5f}, {target_point['lng']:.5f})")

        current_pos = self.get_current_position()
        if not current_pos:
            raise ValueError("无法获取当前位置以规划路径")

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
        R = 6371000  # 地球半径 (米)
        lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])

        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def compute_motion_params(self, start, target, V=100, Kp=1.0):
        if start is None:
            self.log.append("起点为空，无法计算运动参数")
            return 0, 0

        lat1, lon1, yaw_deg = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])

        lat1_rad, lon1_rad, lat2_rad, lon2_rad, current_yaw_rad = map(
            math.radians, [lat1, lon1, lat2, lon2, yaw_deg]
        )

        dlon = lon2_rad - lon1_rad

        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

        target_angle_rad = math.atan2(y, x)
        yaw_error = target_angle_rad - current_yaw_rad
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        omega = Kp * yaw_error
        distance = self._calc_distance(lat1, lon1, lat2, lon2)
        return omega, distance

    def execute_motion(self, motion):
        commander = BasePlateCommand([motion], self.stop_event, log_container=self.log)
        commander.execute()

    def stop_robot(self):
        commander = BasePlateCommand([], Event(), log_container=self.log)
        commander.stop_robot()

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

    def __init__(self, motion_sequence, stop_event, V=100, L=174, log_container=None):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.rate = rospy.Rate(10)
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.stop_event = stop_event
        self.log_container = log_container if log_container is not None else []

    def execute(self):
        """执行完整的运动序列（旋转+直行）。"""
        for omega, distance in self.motion_sequence:
            if self.stop_event.is_set():
                break

            if abs(omega) > 0.1:
                self.rotate(omega)

            if self.stop_event.is_set():
                break

            if distance > 0.2:
                self.move_straight(distance)

        self.stop_robot()

    def rotate(self, omega):
        """执行旋转动作。"""
        if abs(omega) < 0.1:
            self.log_container.append(f"旋转角度过小({omega:.2f} rad)，跳过")
            return

        control = ht_control(mode=1, x=0, y=self.L if omega > 0 else -self.L, stop=0)
        duration = min(5.0, max(0.5, abs(omega) * 2))
        start_time = rospy.Time.now()

        direction_str = '右' if omega > 0 else '左'
        self.log_container.append(f"开始旋转: 角度={omega:.2f} rad, 方向={direction_str}, 时间={duration:.1f}s")

        while (rospy.Time.now() - start_time) < rospy.Duration(duration):
            if self.stop_event.is_set():
                break
            self.pub.publish(control)
            self.rate.sleep()

    def move_straight(self, distance):
        """执行直线移动动作。"""
        if distance < 0.2:
            self.log_container.append(f"移动距离过短({distance:.2f}米)，跳过")
            return

        control = ht_control(mode=1, x=self.V, y=0, stop=0)
        speed_mps = self.V / 100.0  # 假设V是cm/s, 转换为m/s

        if speed_mps <= 0:
            return

        duration = distance / speed_mps
        start_time = rospy.Time.now()

        self.log_container.append(f"直线移动: 距离={distance:.2f}米, 速度={speed_mps:.2f}m/s, 时间={duration:.1f}s")

        while (rospy.Time.now() - start_time) < rospy.Duration(duration):
            if self.stop_event.is_set():
                break
            self.pub.publish(control)
            self.rate.sleep()

    def stop_robot(self):
        """发送车辆停止指令。"""
        control = ht_control(mode=1, x=0, y=0, stop=1)
        self.pub.publish(control)
        self.log_container.append("车辆停止指令已发送")


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