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

# (导航状态常量 NAV_STATUS_* 保持不变)
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
        self.log.append(f"Naver in '{mode}' mode initialized.")
    
    def run(self):
        if self.mode == "navigation": self.run_navigation_task()
        elif self.mode == "record": rospy.spin()

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
            self.log.append("错误: 获取初始位置失败，导航中止。"); self.status = NAV_STATUS_IDLE; return
        try:
            self._prepare_navigation(self.mvto_point)
            if not self.nav_points: self.status = NAV_STATUS_DONE; return
        except Exception as e:
            self.log.append(f"错误: 导航准备失败 - {e}"); self.status = NAV_STATUS_IDLE; return
        log_msg = f"正向导航 (共{len(self.nav_points)}个点)" if self.dir_mode == "forward" and self.mvto_point is None else f"逆向导航 (剩余{len(self.nav_points)}段)" if self.dir_mode == "reverse" else f"前往指定点 (共{len(self.nav_points)}段)"
        self.log.append(log_msg)
        current_pos = start_pos
        for target in self.nav_points:
            if self.stop_event.is_set(): self.log.append("导航被用户中断"); break
            self.current_target = target
            try: self.current_target_index = self.points.index(target); idx_info = f"{self.current_target_index}/{len(self.points)}"
            except ValueError: self.current_target_index = None; idx_info = "未知索引"
            motion = self.compute_motion_params(current_pos, target); self.execute_motion(motion)
            next_pos = self.get_current_position()
            if next_pos:
                current_pos = next_pos; self.log.append(f"到达 {idx_info} 号点: ({current_pos['lat']:.5f}, {current_pos['lng']:.5f})")
            else:
                current_pos = target; self.log.append(f"到达 {idx_info} 号点 (位置更新失败)")
            rospy.sleep(0.1)
        self.status = NAV_STATUS_DONE
        if not self.stop_event.is_set(): self.log.append(f"{self.dir_mode} 导航任务完成")
        self.current_target, self.current_target_index = None, None
        self.stop_robot(); self.cleanup_subscribers()

    def navsat_fix_callback(self, msg): self.latest_navsat_fix = msg
    def odometry_callback(self, msg): self.latest_odometry = msg

    def _get_combined_position(self):
        if self.latest_navsat_fix is None or self.latest_odometry is None: return None
        odom_pose = self.latest_odometry.pose.pose; x, y, q = odom_pose.position.x, odom_pose.position.y, odom_pose.orientation
        try: _, _, yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w]); yaw_deg = math.degrees(yaw_rad)
        except Exception: yaw_deg = 0.0
        return {"lat": self.latest_navsat_fix.latitude, "lng": self.latest_navsat_fix.longitude, "yaw": yaw_deg, "x": x, "y": y, "angle": yaw_deg, "time": datetime.now().strftime("%H:%M:%S")}
    
    def get_current_position(self): self.current_position = self._get_combined_position(); return self.current_position

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

    # 【修改点】: 逆向导航不再寻找最近点，而是始终从最后一个点开始
    def _prepare_reverse_navigation(self):
        """准备逆序导航数据，始终从最后一个点开始"""
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

    # 【修改点】: 增加智能倒车判断逻辑
    def compute_motion_params(self, start, target, V=100, Kp=1.0):
        if start is None: self.log.append("起点为空，无法计算运动参数"); return 0, 0, False
        lat1, lon1, yaw_deg = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])
        lat1_rad, lon1_rad, lat2_rad, lon2_rad, current_yaw_rad = map(math.radians, [lat1, lon1, lat2, lon2, yaw_deg])
        
        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        target_angle_rad = math.atan2(y, x)
        
        yaw_error = target_angle_rad - current_yaw_rad
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        
        # 判断是否需要倒车
        reverse_motion = False
        if abs(yaw_error) > (math.pi / 2):
            reverse_motion = True
            # 调整yaw_error为车尾朝向目标时的角度差
            yaw_error = yaw_error - math.copysign(math.pi, yaw_error)

        omega = Kp * yaw_error
        distance = self._calc_distance(lat1, lon1, lat2, lon2)
        return omega, distance, reverse_motion

    def execute_motion(self, motion):
        omega, distance, reverse_flag = motion # 接收倒车标志
        commander = BasePlateCommand([motion], self.stop_event, log_container=self.log)
        commander.execute(reverse_flag) # 将倒车标志传递下去

    def stop_robot(self):
        commander = BasePlateCommand([], Event(), log_container=self.log); commander.stop_robot()
    
    # (其他文件管理方法保持不变)
    def add_point(self, comment=""):
        point = self.get_current_position();
        if not point: self.log.append("无法获取当前位置，添加点失败"); return
        point["comment"] = comment; self.points.append(point); self.modified = True; self.log.append(f"添加点: ({point['lat']:.6f}, {point['lng']:.6f})")
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

class BasePlateCommand:
    V_REVERSE = 50
    SPEED_FACTOR = 1.0
    ROTATION_FACTOR = 2.0

    def __init__(self, motion_sequence, stop_event, V=100, L=174, log_container=None):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10); self.rate = rospy.Rate(10)
        self.motion_sequence = motion_sequence; self.V, self.L = V, L; self.stop_event = stop_event
        self.log_container = log_container if log_container is not None else []
    
    # 【修改点】: execute方法现在接收倒车标志
    def execute(self, reverse_flag=False):
        # motion_sequence 只包含一个 (omega, distance, reverse_flag) 元组
        # 但我们从 Naver 传递了正确的 reverse_flag
        # 为保持结构，我们依然用循环，尽管它只执行一次
        for omega, distance, _ in self.motion_sequence: # 忽略元组里的倒车标志，使用传入的
            if self.stop_event.is_set(): break
            
            # 只有需要转向时才转
            if abs(omega) > 0.1:
                self.rotate_in_place(omega)
            
            if self.stop_event.is_set(): break
            
            # 根据传入的标志决定前进还是倒车
            if distance > 0.2:
                self.move_straight(distance, reverse=reverse_flag)

    # 【修改点】: 转向方法改回原地旋转
    def rotate_in_place(self, omega):
        if abs(omega) < 0.1: self.log_container.append(f"旋转角度过小({omega:.2f} rad)，跳过"); return
        
        # 速度x为0，实现原地旋转
        control = ht_control(mode=1, x=0, y=self.L if omega > 0 else -self.L, stop=0)
        duration = min(5.0, max(0.5, abs(omega) * self.ROTATION_FACTOR))
        start_time = rospy.Time.now()
        
        direction_str = '右' if omega > 0 else '左'
        self.log_container.append(f"开始原地旋转: 角度={omega:.2f} rad, 方向={direction_str}, 时间={duration:.1f}s")
        
        while (rospy.Time.now() - start_time) < rospy.Duration(duration):
            if self.stop_event.is_set(): break
            self.pub.publish(control); self.rate.sleep()

    def move_straight(self, distance, reverse=False):
        if distance < 0.2: self.log_container.append(f"移动距离过短({distance:.2f}米)，跳过"); return
        linear_velocity = -self.V_REVERSE if reverse else self.V
        log_action = "直线倒车" if reverse else "直线移动"
        control = ht_control(mode=1, x=linear_velocity, y=0, stop=0)
        speed_for_calc = self.V_REVERSE if reverse else self.V
        speed_mps = speed_for_calc / 100.0
        if speed_mps <= 0: return
        effective_speed = speed_mps * self.SPEED_FACTOR
        duration = distance / effective_speed if effective_speed > 0 else float('inf')
        start_time = rospy.Time.now()
        self.log_container.append(f"{log_action}: 距离={distance:.2f}米, 速度={linear_velocity/100.0:.2f}m/s, 时间={duration:.1f}s")
        while (rospy.Time.now() - start_time) < rospy.Duration(duration):
            if self.stop_event.is_set(): break
            self.pub.publish(control); self.rate.sleep()

    def stop_robot(self):
        self.pub.publish(ht_control(mode=1, x=0, y=0, stop=1)); self.log_container.append("车辆停止指令已发送")

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