from threading import Thread, Event
from datetime import datetime
import os, json
import rospy
import copy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from ros_ht_msg.msg import ht_control
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# 导航状态常量
NAV_STATUS_IDLE = "闲置"
NAV_STATUS_RUNNING = "导航中"
NAV_STATUS_DONE = "已完成"


class Naver(Thread):
    def __init__(self, recorder, mode="record", dir_mode="forward", mvto_point=None):
        Thread.__init__(self)
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
        rospy.Subscriber("/gps/filtered", NavSatFix, self.navsat_fix_callback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)

        self.nav_thread = None
        
        self.log.append(f"[INIT] Naver in '{mode}' mode initialized.")
    
    def run(self):
        rospy.spin()

    def navsat_fix_callback(self, msg):
        self.latest_navsat_fix = msg

    def odometry_callback(self, msg):
        self.latest_odometry = msg

    def _get_combined_position(self):
        if self.latest_navsat_fix is None or self.latest_odometry is None:
            return None
        odom_pose = self.latest_odometry.pose.pose
        x = odom_pose.position.x
        y = odom_pose.position.y
        q = odom_pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            _, _, yaw_rad = euler_from_quaternion(quaternion)
            yaw_deg = math.degrees(yaw_rad)
        except Exception as e:
            self.log.append(f"[WARN] 从四元数计算Yaw失败: {e}")
            yaw_deg = 0.0
        lat = self.latest_navsat_fix.latitude
        lng = self.latest_navsat_fix.longitude
        return {
            "lat": lat, "lng": lng, "yaw": yaw_deg,
            "x": x, "y": y, "angle": yaw_deg,
            "time": datetime.now().strftime("%H:%M:%S")
        }

    def get_current_position(self):
        combined_position = self._get_combined_position()
        if combined_position:
            self.current_position = combined_position
            return combined_position
        else:
            return None 

    def _mock_gps(self):
        return {
            "lat": 31.123456, "lng": 121.654321, "yaw": 45.0,
            "x": 1.0, "y": 2.0, "angle": 45.0,
            "time": datetime.now().strftime("%H:%M:%S")
        }
    
    def add_point(self, comment=""):
        point = self.get_current_position()
        if not point:
            self.log.append("[WARN] 无法获取当前位置，添加点失败")
            return self.points
        point["comment"] = comment
        self.points.append(point)
        self.modified = True
        self.log.append(f"[INFO] 添加点: ({point['lat']:.6f}, {point['lng']:.6f})")
        return self.points

    def insert_point(self, index, comment=""):
        if index < 0 or index > len(self.points):
            self.log.append(f"[WARN] 无效索引: {index}")
            return None
        point = self.get_current_position()
        if not point:
            self.log.append("[WARN] 无法获取当前位置，插入点失败")
            return self.points
        point["comment"] = comment
        self.points.insert(index, point)
        self.modified = True
        self.log.append(f"[INFO] 在位置{index}插入点")
        return self.points

    def delete_point(self, index):
        if index < 0 or index >= len(self.points):
            self.log.append(f"[WARN] 无效索引: {index}")
            return 0
        self.points.pop(index)
        self.modified = True
        self.log.append(f"[INFO] 删除点 {index}")
        return len(self.points)
        
    def save_points(self):
        if not self.modified:
            self.log.append("[INFO] 无修改，跳过保存")
            return
        self.recorder.save(self.points)
        self.modified = False
        self.log.append(f"[INFO] 保存{len(self.points)}个点到文件")
    
    def _prepare_navigation(self, mvto_point=None):
        if not self.points:
            raise ValueError("路径点为空，无法导航")
        self.nav_points = self.points
        if mvto_point is not None:
             self._prepare_mvto_navigation(mvto_point)
        elif self.dir_mode == "reverse":
            self._prepare_reverse_navigation()
        else: # forward
            self.log.append("[INFO] 使用默认的正向导航")
        self.log.append(f"[INFO] 导航准备完成 ({self.dir_mode}方向, {len(self.nav_points)}点)")

    def _prepare_reverse_navigation(self):
        current_pos = self.get_current_position()
        if current_pos:
            start_index = self._find_nearest_point(current_pos)
            self.log.append(f"[INFO] 定位成功: 从{start_index}号点开始返回")
        else:
            start_index = len(self.points) - 1
            self.log.append("[WARN] 无法定位, 使用默认起点: 从终点开始返回")
        if start_index < 0 or start_index >= len(self.points):
            self.log.append(f"[ERROR] start_index {start_index} 超出范围")
            start_index = len(self.points) - 1
        self.nav_points = self.points[start_index::-1] if start_index >= 0 else self.points[::-1]

    def _prepare_mvto_navigation(self, target_index):
        if not (0 <= target_index < len(self.points)): 
            raise ValueError(f"目标点索引{target_index}超出范围")
        current_pos = self.get_current_position()
        start_index = self._find_nearest_point(current_pos) if current_pos else 0
        self.log.append(f"[INFO] 指定目标点: 从最近点 {start_index} 前往目标点 {target_index}")
        if start_index == target_index:
            self.log.append("[INFO] 已经在目标点, 无需移动")
            self.nav_points = []
            return
        if start_index < target_index:
            self.nav_points = self.points[start_index : target_index + 1]
        else:
            self.nav_points = self.points[start_index : target_index - 1 : -1]

    def _find_nearest_point(self, current_pos):
        min_dist = float('inf')
        nearest_index = 0
        for i, point in enumerate(self.points): 
            dist = self._calc_distance(
                point['lat'], point['lng'],
                current_pos['lat'], current_pos['lng']
            )
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index

    def _calc_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def start_navigation(self):
        if self.nav_thread and self.nav_thread.is_alive():
            self.stop_navigation()
        self.status = NAV_STATUS_RUNNING
        self.stop_event.clear()
        current_pos = self.get_current_position()
        if not current_pos:
            self.log.append("[WARN] 无法获取初始位置，将从路径的第一个点开始导航")
            if not self.points:
                self.log.append("[ERROR] 路径为空，无法启动导航")
                self.status = NAV_STATUS_IDLE
                return
            current_pos = self.points[0]
        self.nav_thread = Thread(target=self._navigate, args=(self.nav_points, current_pos))
        self.nav_thread.daemon = True
        self.nav_thread.start()
        self.log.append(f"[INFO] {self.dir_mode} 导航已启动")
    
    def stop_navigation(self):
        if self.nav_thread and self.nav_thread.is_alive():
            self.stop_event.set()
            self.nav_thread.join(timeout=2.0)
        self.status = NAV_STATUS_IDLE
        self.log.append("[INFO] 导航已停止")
    
    def _navigate(self, points, start_pos):
        if self.dir_mode == "forward" and self.mvto_point is None:
            self.log.append(f"[INFO] 正向导航 (共{len(points)}个点)")
        elif self.dir_mode == "reverse":
            self.log.append(f"[INFO] 逆向导航 (剩余{len(points)}段)")
        else:
            self.log.append(f"[INFO] 前往指定点 (共{len(points)}段)")
        current_pos = start_pos
        for target in points:
            if self.stop_event.is_set(): 
                self.log.append("[WARN] 导航被中断")
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
                self.current_position = next_pos
                self.log.append(f"[INFO] 到达 {idx_info} 号点")
            else:
                current_pos = target
                self.current_position = target
                self.log.append(f"[WARN] 到达 {idx_info} 号点 (位置更新失败)")
            rospy.sleep(0.1)
        self.status = NAV_STATUS_DONE
        self.log.append(f"[INFO] {self.dir_mode} 导航完成")
        self.current_target = None
        self.current_target_index = None
        self.stop_robot()

    def compute_motion_params(self, start, target, V=100, Kp=1.0):
        if start is None:
            self.log.append("[ERROR] 起点为空，无法计算运动参数")
            return 0, 0 # 返回一个无害的动作
        lat1, lon1, yaw_deg = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        current_yaw_rad = math.radians(yaw_deg)
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
        omega, distance = motion
        commander = BasePlateCommand([(omega, distance)], self.stop_event, log_container=self.log)
        commander.run() 
    
    def stop_robot(self):
        commander = BasePlateCommand([(0, 0)], Event(), log_container=self.log)
        commander.stop_robot()
        self.log.append("[INFO] 导航序列完成，车辆已停止")

class BasePlateCommand(Thread):
    def __init__(self, motion_sequence, stop_event, V=100, L=174, log_container=None):
        super().__init__()
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.rate = rospy.Rate(10)
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.stop_event = stop_event
        self.is_complete = False
        self.log_container = log_container if log_container is not None else []
        
    def run(self):
        for i, (omega, distance) in enumerate(self.motion_sequence):
            if self.stop_event.is_set(): break
            if abs(omega) > 0.1:
                self.rotate(omega)
                if self.stop_event.is_set(): break
            if distance > 0.2:
                self.move_straight(distance)
                if self.stop_event.is_set(): break
        self.stop_robot()
        self.is_complete = True
        
    def rotate(self, omega):
        if abs(omega) < 0.1:
            self.log_container.append(f"[MOTION] 旋转角度过小({omega:.2f} rad)，跳过")
            return
        control = ht_control(mode=1, x=0, y=self.L if omega > 0 else -self.L, stop=0)
        duration = min(5.0, max(0.5, abs(omega) * 2))
        start_time = rospy.Time.now()
        self.log_container.append(f"[MOTION] 开始旋转: 角度={omega:.2f} rad, 方向={'右' if omega > 0 else '左'}, 时间={duration:.1f}s")
        while (rospy.Time.now() - start_time) < rospy.Duration(duration) and not self.stop_event.is_set():
            self.pub.publish(control)
            self.rate.sleep()
            
    def move_straight(self, distance):
        if distance < 0.2:
            self.log_container.append(f"[MOTION] 移动距离过短({distance:.2f}米)，跳过")
            return
        control = ht_control(mode=1, x=self.V, y=0, stop=0)
        speed_mps = self.V * 0.01
        duration = distance / speed_mps if speed_mps > 0 else float('inf')
        start_time = rospy.Time.now()
        self.log_container.append(f"[MOTION] 直线移动: 距离={distance:.2f}米, 速度={speed_mps:.2f}m/s, 时间={duration:.1f}s")
        while (rospy.Time.now() - start_time) < rospy.Duration(duration) and not self.stop_event.is_set():
            self.pub.publish(control)
            self.rate.sleep()
            
    def stop_robot(self):
        control = ht_control(mode=1, x=0, y=0, stop=1)
        self.pub.publish(control)
        self.log_container.append("[MOTION] 车辆停止指令已发送")

class PathRecorder:
    def __init__(self, listname):
        self.listname = listname.split('.')[0]
        base_dir = os.path.dirname(os.path.abspath(__file__)) 
        self.json_dir = os.path.join(base_dir, "paths_json")
        self.path_file = os.path.join(self.json_dir, f"{self.listname}.json") 
        if not os.path.exists(self.json_dir): 
            os.makedirs(self.json_dir) 
        
    def load(self):
        if not os.path.exists(self.path_file):
            return []
        try:
            with open(self.path_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"加载路径失败: {str(e)}")
            return []
    
    def save(self, points):
        try:
            with open(self.path_file, "w", encoding="utf-8") as f:
                json.dump(points, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            rospy.logerr(f"保存路径失败: {str(e)}")
            return False