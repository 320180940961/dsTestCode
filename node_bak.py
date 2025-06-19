#!/usr/bin/env python3
"""
该模块定义了用于路径记录和导航的核心类。
导航逻辑已升级为基于实时位置反馈的闭环控制。
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

NAV_STATUS_IDLE = "闲置"
NAV_STATUS_RUNNING = "导航中"
NAV_STATUS_DONE = "已完成"


class Naver(Thread):
    def __init__(self, recorder, mode="record", dir_mode="forward", mvto_point=None):
        Thread.__init__(self, daemon=True)
        self.recorder = recorder
        self.mode = mode
        self.dir_mode = dir_mode
        self.mvto_point = mvto_point
        self.stop_event = Event()
        self.log = []
        self.status = NAV_STATUS_IDLE
        self.current_position = None
        self.current_target = None
        self.current_target_index = None
        self.points = copy.deepcopy(self.recorder.load())
        self.modified = False
        self.latest_navsat_fix = None
        self.latest_odometry = None

        self.sub1 = rospy.Subscriber("/gps/filtered", NavSatFix, self.navsat_fix_callback)
        self.sub2 = rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)

        # 【修改点】: 创建一个BasePlateCommand的实例，作为指令发送器
        self.commander = BasePlateCommand(log_container=self.log)
        
        # 【修改点】: 在此处定义闭环控制的参数，方便调整
        self.arrival_threshold = 0.1  # 到达阈值（米）
        self.kp_turn = 1.2            # 转向的比例增益
        self.forward_speed = 60       # 前进速度 (0-100)
        self.reverse_speed = 40       # 倒车速度 (0-100)
        self.max_turn_cmd = 174       # 最大转向控制值

        self.log.append(f"Naver in '{mode}' mode initialized.")

    def run(self):
        if self.mode == "navigation":
            self.run_navigation_task()
        elif self.mode == "record":
            rospy.spin()

    def run_navigation_task(self):
        self.status = NAV_STATUS_RUNNING
        self.log.append("正在获取初始定位...")
        wait_start_time = rospy.get_time()
        timeout = 5.0
        start_pos = self.get_current_position()
        while not start_pos and not self.stop_event.is_set() and (rospy.get_time() - wait_start_time < timeout):
            self.log.append(f"等待初始定位... ({rospy.get_time() - wait_start_time:.1f}s)")
            rospy.sleep(0.5)
            start_pos = self.get_current_position()
        if not start_pos:
            self.log.append("错误: 获取初始位置失败，导航中止。")
            self.status = NAV_STATUS_IDLE
            return
        
        try:
            self._prepare_navigation(self.mvto_point)
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
                idx_info = f"{self.current_target_index}/{len(self.points)}"
            except ValueError:
                self.current_target_index = None
                idx_info = "未知索引"

            arrived = self.go_to_point_closed_loop(current_pos, target, idx_info)
            
            if not arrived:
                self.log.append(f"前往点 {idx_info} 的任务被中断或失败。")
                break

            current_pos = self.get_current_position() or target
            self.log.append(f"确认到达 {idx_info} 号点。")

        self.status = NAV_STATUS_DONE
        if not self.stop_event.is_set():
            self.log.append(f"{self.dir_mode} 导航任务完成")
        
        self.current_target = None
        self.current_target_index = None
        self.stop_robot()
        self.cleanup_subscribers()

    def go_to_point_closed_loop(self, start_pos, target_pos, target_info):
        self.log.append(f"开始闭环控制前往目标点 {target_info}")
        current_pos = start_pos
        control_rate = rospy.Rate(10) # 10Hz的控制频率

        while not rospy.is_shutdown():
            if self.stop_event.is_set():
                return False

            realtime_pos = self.get_current_position()
            if realtime_pos:
                current_pos = realtime_pos
            
            distance = self._calc_distance(current_pos['lat'], current_pos['lng'], target_pos['lat'], target_pos['lng'])

            if distance < self.arrival_threshold:
                self.log.append(f"距离目标点({distance:.2f}m)小于阈值，认为到达。")
                self.stop_robot()
                return True

            yaw_error, reverse_motion = self.compute_heading_error(current_pos, target_pos)
            
            linear_velocity = -self.reverse_speed if reverse_motion else self.forward_speed
            angular_velocity_cmd = self.kp_turn * yaw_error
            
            turn_cmd = np.clip(angular_velocity_cmd * 100, -self.max_turn_cmd, self.max_turn_cmd)

            self.commander.send_move_command(linear_velocity, turn_cmd)
            control_rate.sleep()

    def compute_heading_error(self, start, target):
        lat1, lon1, yaw_deg = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])
        lat1_rad, lon1_rad, lat2_rad, lon2_rad, current_yaw_rad = map(math.radians, [lat1, lon1, lat2, lon2, yaw_deg])
        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        target_angle_rad = math.atan2(y, x)
        yaw_error = (target_angle_rad - current_yaw_rad + math.pi) % (2 * math.pi) - math.pi
        
        reverse_motion = abs(yaw_error) > (math.pi / 2)
        if reverse_motion:
            yaw_error = yaw_error - math.copysign(math.pi, yaw_error)
            
        return yaw_error, reverse_motion
    
    def stop_robot(self):
        self.commander.send_stop_command()

    # (其他方法保持不变, execute_motion被移除)
    def navsat_fix_callback(self, msg): self.latest_navsat_fix = msg
    def odometry_callback(self, msg): self.latest_odometry = msg
    def _get_combined_position(self):
        if self.latest_navsat_fix is None or self.latest_odometry is None: return None
        odom_pose = self.latest_odometry.pose.pose; x, y, q = odom_pose.position.x, odom_pose.position.y, odom_pose.orientation
        try: _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w]); yaw_deg = math.degrees(yaw_rad)
        except Exception: yaw_deg = 0.0
        return {"lat": self.latest_navsat_fix.latitude, "lng": self.latest_navsat_fix.longitude, "yaw": yaw_deg, "x": x, "y": y, "angle": yaw_deg, "time": datetime.now().strftime("%H:%M:%S")}
    def get_current_position(self): self.current_position = self._get_combined_position(); return self.current_position
    def add_point(self, comment=""):
        point = self.get_current_position();
        if not point: self.log.append("无法获取当前位置，添加点失败"); return
        point["comment"] = comment; self.points.append(point); self.modified = True; self.log.append(f"添加点: ({point['lat']:.6f}, {point['lng']:.6f})")
    def stop_navigation(self): self.stop_event.set(); self.log.append("导航停止信号已发送")
    def cleanup_subscribers(self):
        if hasattr(self, 'sub1') and self.sub1: self.sub1.unregister()
        if hasattr(self, 'sub2') and self.sub2: self.sub2.unregister()
    def _prepare_navigation(self, mvto_point=None):
        if not self.points: raise ValueError("路径点为空")
        self.nav_points = self.points
        if mvto_point is not None: self._prepare_mvto_navigation(mvto_point)
        elif self.dir_mode == "reverse": self._prepare_reverse_navigation()
        else: self.log.append("使用默认的正向导航")
        self.log.append(f"导航准备完成 ({self.dir_mode}方向, {len(self.nav_points)}点)")
    def _prepare_reverse_navigation(self):
        start_index = len(self.points) - 1
        self.log.append(f"逆向导航: 准备从终点(点{start_index})开始返回")
        self.nav_points = self.points[start_index::-1]
    def _prepare_mvto_navigation(self, target_index):
        if not (0 <= target_index < len(self.points)): raise ValueError(f"目标点索引{target_index}超出范围")
        target_point = self.points[target_index]; self.log.append(f"指定目标点: {target_index} - ({target_point['lat']:.5f}, {target_point['lng']:.5f})")
        current_pos = self.get_current_position()
        if not current_pos: raise ValueError("无法获取当前位置以规划路径")
        self.log.append(f"从当前点 ({current_pos['lat']:.5f}, {current_pos['lng']:.5f}) 开始规划路径...")
        self.nav_points = [current_pos, target_point]
    def _find_nearest_point(self, current_pos): return min(range(len(self.points)), key=lambda i: self._calc_distance(self.points[i]['lat'], self.points[i]['lng'], current_pos['lat'], current_pos['lng']))
    def _calc_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000; lat1_rad, lon1_rad, lat2_rad, lon2_rad = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon, dlat = lon2_rad - lon1_rad, lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    def save_points(self):
        if not self.modified: self.log.append("无修改，跳过保存"); return
        self.recorder.save(self.points); self.modified = False; self.log.append(f"保存{len(self.points)}个点到文件")
    def delete_point(self, index):
        if not (0 <= index < len(self.points)): self.log.append(f"无效索引: {index}"); return 0
        self.points.pop(index); self.modified = True; self.log.append(f"删除点 {index}"); return len(self.points)
    def insert_point(self, index, comment=""):
        if not (0 <= index <= len(self.points)): self.log.append(f"无效索引: {index}"); return None
        point = self.get_current_position();
        if not point: self.log.append("无法获取当前位置，插入点失败"); return
        point["comment"] = comment; self.points.insert(index, point); self.modified = True; self.log.append(f"在位置{index}插入点")


# 【核心修改】: BasePlateCommand 被简化为一个普通的指令发送工具类
class BasePlateCommand:
    def __init__(self, log_container=None):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.log_container = log_container if log_container is not None else []

    def send_move_command(self, linear_velocity, turn_cmd):
        """发送单条移动指令"""
        control_msg = ht_control(mode=1, x=linear_velocity, y=turn_cmd, stop=0)
        self.pub.publish(control_msg)
    
    def send_stop_command(self):
        """发送单条停止指令"""
        control_msg = ht_control(mode=1, x=0, y=0, stop=1)
        self.pub.publish(control_msg)
        self.log_container.append("车辆停止指令已发送")

class PathRecorder:
    def __init__(self, listname):
        self.listname = listname.split('.')[0]; base_dir = os.path.dirname(os.path.abspath(__file__))
        self.json_dir = os.path.join(base_dir, "paths_json"); self.path_file = os.path.join(self.json_dir, f"{self.listname}.json")
        if not os.path.exists(self.json_dir): os.makedirs(self.json_dir)
    def load(self):
        if not os.path.exists(self.path_file): return []
        try:
            with open(self.path_file, "r", encoding="utf-8") as f: return json.load(f)
        except Exception as e: rospy.logerr(f"加载路径失败: {str(e)}"); return []
    def save(self, points):
        try:
            with open(self.path_file, "w", encoding="utf-8") as f: json.dump(points, f, indent=2, ensure_ascii=False); return True
        except Exception as e: rospy.logerr(f"保存路径失败: {str(e)}"); return False