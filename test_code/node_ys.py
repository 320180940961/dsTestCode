# 【修改点】: 导入新的标准消息类型和转换工具
from threading import Thread, Event
from datetime import datetime
import os
import json
import rospy
import copy
from sensor_msgs.msg import NavSatFix  # 使用标准GPS消息
from nav_msgs.msg import Odometry      # 使用标准里程计消息
from ros_ht_msg.msg import ht_control  # ht_control 保持不变
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# 导航状态常量
NAV_STATUS_IDLE = "闲置"
NAV_STATUS_RUNNING = "导航中"
NAV_STATUS_DONE = "已完成"


class Naver(Thread):
    def __init__(self, recorder, mode="record", dir_mode="forward", mvto_point=None):
        """
        初始化Naver
        
        :param recorder: PathRecorder实例
        :param mode: 模式 ("record"记录或"navigation"导航)
        :param dir_mode: 导航方向 ("forward"正序/"reverse"逆序)
        :param mvto_point: 指定目标点索引
        """
        Thread.__init__(self)
        self.recorder = recorder
        self.mode = mode
        self.dir_mode = dir_mode
        self.stop_event = Event()
        self.log = []
        self.status = NAV_STATUS_IDLE
        self.current_position = None
        self.current_target = None
        self.current_target_index = None
        
        # 工作副本管理
        self.points = copy.deepcopy(self.recorder.load())  # 独立的工作副本
        self.modified = False  # 标记是否修改过点

        # 【修改点】: 初始化用于存储两个话题最新消息的变量
        self.latest_navsat_fix = None
        self.latest_odometry = None

        # 【修改点】: 创建两个订阅者，分别对应两个话题
        rospy.Subscriber("/gps/filtered", NavSatFix, self.navsat_fix_callback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback)

        # 导航线程使用新创建而不是继承
        self.nav_thread = None
            
        rospy.loginfo(f"Naver {mode} mode initialized")
    
    def run(self):
        rospy.spin()

    # 【修改点】: 新增 NavSatFix 消息的回调函数
    def navsat_fix_callback(self, msg):
        """存储最新的 /gps/filtered (NavSatFix) 消息"""
        self.latest_navsat_fix = msg

    # 【修改点】: 新增 Odometry 消息的回调函数
    def odometry_callback(self, msg):
        """存储最新的 /odometry/filtered (Odometry) 消息"""
        self.latest_odometry = msg

    # 【修改点】: 这是核心修改，融合两个数据源
    def _get_combined_position(self):
        """
        融合 NavSatFix 和 Odometry 数据来构成一个完整的数据点。
        如果任一数据源缺失，则返回 None。
        """
        if self.latest_navsat_fix is None or self.latest_odometry is None:
            return None

        # 从 Odometry 获取 x, y 和 orientation (quaternion)
        odom_pose = self.latest_odometry.pose.pose
        x = odom_pose.position.x
        y = odom_pose.position.y
        
        q = odom_pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        
        # 从四元数计算 Yaw 角
        try:
            _, _, yaw_rad = euler_from_quaternion(quaternion)
            yaw_deg = math.degrees(yaw_rad)
        except Exception as e:
            rospy.logwarn_throttle(5, f"从四元数计算Yaw失败: {e}")
            yaw_deg = 0.0

        # 从 NavSatFix 获取经纬度
        lat = self.latest_navsat_fix.latitude
        lng = self.latest_navsat_fix.longitude
        
        # 格式化并返回完整数据
        return {
            "lat": lat,
            "lng": lng,
            "yaw": yaw_deg,
            "x": x,
            "y": y,
            "angle": yaw_deg,  # 保持旧格式兼容性
            "time": datetime.now().strftime("%H:%M:%S")
        }

    # 【修改点】: 修改 get_current_position 以使用新的数据融合方法
    def get_current_position(self):
        """获取当前位置"""
        combined_position = self._get_combined_position()
        
        if combined_position:
            self.current_position = combined_position
            return combined_position
        else:
            # 仅在无法获取真实数据时提供模拟数据用于测试
            rospy.logwarn_throttle(5, "无法获取完整的GPS和里程计数据，将使用模拟数据或无数据")
            return None # 或者返回 self._mock_gps() 用于测试

    def _mock_gps(self):
        """模拟GPS位置（用于测试）"""
        # 使用固定测试点
        return {
            "lat": 31.123456,
            "lng": 121.654321,
            "yaw": 45.0,
            "x": 1.0,
            "y": 2.0,
            "angle": 45.0,
            "time": datetime.now().strftime("%H:%M:%S")
        }
    
    def add_point(self, comment=""):
        """添加点"""
        point = self.get_current_position()
        if not point:
            rospy.logwarn("无法获取当前位置")
            return self.points
            
        point["comment"] = comment
        self.points.append(point)
        self.modified = True
        
        rospy.loginfo(f"添加点: ({point['lat']:.6f}, {point['lng']:.6f})")
        return self.points

    def insert_point(self, index, comment=""):
        """在指定位置插入点"""
        if index < 0 or index > len(self.points):
            rospy.logwarn(f"无效索引: {index}")
            return None
            
        point = self.get_current_position()
        if not point:
            rospy.logwarn("无法获取当前位置")
            return self.points
            
        point["comment"] = comment
        self.points.insert(index, point)
        self.modified = True
        
        rospy.loginfo(f"在位置{index}插入点")
        return self.points

    def delete_point(self, index):
        """删除指定点"""
        if index < 0 or index >= len(self.points):
            rospy.logwarn(f"无效索引: {index}")
            return 0
            
        self.points.pop(index)
        self.modified = True
        
        rospy.loginfo(f"删除点 {index}")
        return len(self.points)
        
    def save_points(self):
        """保存点到文件"""
        if not self.modified:
            rospy.loginfo("无修改，跳过保存")
            return
            
        self.recorder.save(self.points)
        self.modified = False
        rospy.loginfo(f"保存{len(self.points)}个点到文件")
    
    def _prepare_navigation(self, mvto_point=None):
        """导航准备"""
        if not self.points:
            raise ValueError("路径点为空，无法导航")
        
        # 准备导航点序列
        self.nav_points = self.points  # 默认使用所有点
        
        # 检查导航方向
        if self.dir_mode == "reverse":
            self._prepare_reverse_navigation()
        elif mvto_point is not None:
            self._prepare_mvto_navigation(mvto_point)
        elif self.dir_mode == "forward":
            rospy.loginfo("使用默认的正向导航")
            
        rospy.loginfo(f"导航准备完成 ({self.dir_mode}方向, {len(self.nav_points)}点)")

    def _prepare_reverse_navigation(self):
        """准备逆序导航数据"""
        current_pos = self.get_current_position()
        
        if current_pos:
            start_index = self._find_nearest_point(current_pos)
            self.log.append(f"定位成功: 从{start_index}号点开始返回")
        else:
            start_index = len(self.points) - 1
            self.log.append("使用默认起点: 从终点开始返回")

        if start_index < 0 or start_index >= len(self.points):
            rospy.logerr(f"start_index {start_index} 超出范围")
            start_index = len(self.points) - 1
        
        # 准备从指定点开始的逆序导航
        self.nav_points = self.points[start_index::-1] if start_index >= 0 else self.points[::-1]

    def _prepare_mvto_navigation(self, target_index):
        """准备指定目标点导航"""
        if 0 <= target_index < len(self.points): 
            self.log.append(f" 指定目标点: 前往{target_index}号点")
            # 准备从指定点开始的正向导航
            self.nav_points = self.points[target_index:]
        else:
            raise ValueError(f"目标点索引{target_index}超出范围")

    def _find_nearest_point(self, current_pos):
        """找到距离当前位置最近的路径点索引"""
        min_dist = float('inf')
        nearest_index = 0
        
        for i, point in enumerate(self.points): 
            dist = self._calc_distance(
                current_pos['lat'], current_pos['lng'],
                point['lat'], point['lng']
            )
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
                
        return nearest_index

    def _calc_distance(self, lat1, lon1, lat2, lon2):
        """计算两点间距离（米）的简化方法"""
        # 简化为平面距离计算（实际项目应使用大圆距离公式）
        return 111000 * np.sqrt((lat2-lat1)**2  + (lon2-lon1)**2)

    def start_navigation(self):
        """启动导航线程"""
        # 确保没有正在运行的导航线程
        if self.nav_thread and self.nav_thread.is_alive():
            self.stop_navigation()
            
        # 创建新的导航线程
        self.status = NAV_STATUS_RUNNING
        self.stop_event.clear()  # 清除停止标志
        
        # 获取起始位置
        current_pos = self.get_current_position() or self.points[0]
        
        # 启动新线程
        self.nav_thread = Thread(target=self._navigate, args=(self.nav_points, current_pos))
        self.nav_thread.daemon = True  # 设置为守护线程
        self.nav_thread.start()
        
        rospy.loginfo(f"{self.dir_mode} 导航已启动")
    
    def stop_navigation(self):
        """停止导航线程"""
        if self.nav_thread and self.nav_thread.is_alive():
            self.stop_event.set()  # 设置停止标志
            self.nav_thread.join(timeout=2.0)  # 等待线程结束
            
        self.status = NAV_STATUS_IDLE
        rospy.loginfo("导航已停止")
    
    def _navigate(self, points, start_pos):
        """
        导航核心函数，可接收任意点序列
        
        :param points: 要导航的路径点序列
        :param start_pos: 起始位置
        """
        # 记录日志
        if self.dir_mode == "forward":
            self.log.append(f"正向导航 (共{len(points)}个点)")
        elif self.dir_mode == "reverse":
            self.log.append(f"逆向导航 (剩余{len(points)}段)")
        else:
            self.log.append(f"自定义导航序列 (点数量: {len(points)})")
        
        current_pos = start_pos
        
        # 执行导航
        for target in points:
            if self.stop_event.is_set(): 
                rospy.loginfo("导航被中断")
                break
                
            # 更新目标信息
            self.current_target = target
            
            # 查找目标点在原始列表中的索引
            try:
                self.current_target_index = self.points.index(target)
                idx_info = f"{self.current_target_index}/{len(self.points)}"
            except ValueError:
                self.current_target_index = None
                idx_info = "未知索引"
            
            # 计算运动参数
            motion = self.compute_motion_params(current_pos,  target)
            
            # 执行移动
            self.execute_motion(motion) 
            
            # 更新当前位置
            current_pos = self.get_current_position()
            self.current_position = current_pos
            
            # 记录日志
            if current_pos:
                self.log.append(f" 到达 {idx_info} 号点: " +
                                f"({target['lat']:.6f}, {target['lng']:.6f})")
            else:
                self.log.append(f" 到达 {idx_info} 号点 (无位置更新)")
                
            # 添加小延迟避免过度消耗CPU
            rospy.sleep(0.1)
        
        # 导航结束处理
        self.status = NAV_STATUS_DONE
        self.log.append(f"{self.dir_mode} 导航完成")
        self.current_target = None
        self.current_target_index = None
        self.stop_robot()

    def compute_motion_params(self, start, target, V=100, Kp=1.0):
        """计算运动参数"""
        lat1, lon1, yaw = float(start["lat"]), float(start["lng"]), float(start.get("yaw", 0))
        lat2, lon2 = float(target["lat"]), float(target["lng"])

        lat1, lon1 = np.radians(lat1), np.radians(lon1)
        lat2, lon2 = np.radians(lat2), np.radians(lon2)

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        target_angle = np.arctan2(dlon, dlat)
        yaw_error = (target_angle - yaw + np.pi) % (2 * np.pi) - np.pi

        omega = Kp * yaw_error
        distance = np.sqrt((dlat * 6371000)**2 + (dlon * 6371000)**2)

        return omega, distance

    def execute_motion(self, motion):
        """执行移动"""
        omega, distance = motion
        commander = BasePlateCommand([(omega, distance)], self.stop_event)
        commander.start()
    
    def stop_robot(self):
        """停止机器人运动"""
        # 发送停止指令
        commander = BasePlateCommand([(0, 0)], Event())
        commander.stop_robot()
        rospy.loginfo_once("导航完成，车辆停止")

class BasePlateCommand(Thread):
    """底盘运动控制线程"""
    def __init__(self, motion_sequence, stop_event, V=100, L=174):
        super().__init__()
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.rate = rospy.Rate(10)
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.stop_event = stop_event
        self.is_complete = False  # 添加完成标志
        
    def run(self):
        """执行运动序列，避免不必要的停止"""
        for i, (omega, distance) in enumerate(self.motion_sequence):
            if self.stop_event.is_set():
                break
            
            # 仅对第一个动作需要旋转
            if i == 0 and abs(omega) > 0.1:  # 忽略微小旋转
                self.rotate(omega)
                
            self.move_straight(distance)
            
            # 仅对最后一个动作需要停止
            if i == len(self.motion_sequence) - 1:
                self.stop_robot()
                
        # 所有动作完成后设置完成标志
        self.is_complete = True
        
    def rotate(self, omega):
        """优化旋转逻辑"""
        if abs(omega) < 0.1:  # 如果旋转角度很小则跳过
            rospy.loginfo(f"旋转角度过小({omega:.2f} rad)，跳过旋转")
            return
            
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = self.L if omega > 0 else -self.L
        control.stop = 0
        
        # 简化逻辑，使用固定控制时间
        duration = min(5.0, max(0.5, abs(omega) * 2))  # 控制时间范围为0.5-5秒
        start_time = rospy.Time.now()
        
        rospy.loginfo(f"开始旋转: 角度={omega:.2f} rad, 方向={'左' if omega < 0 else '右'}, 预计时间={duration:.1f}s")
        
        while (rospy.Time.now() - start_time) < rospy.Duration(duration) and not self.stop_event.is_set():
            self.pub.publish(control)
            self.rate.sleep()
            
    def move_straight(self, distance):
        """直线运动，优化停止逻辑"""
        if distance < 0.2:  # 距离<0.2米则跳过
            rospy.loginfo(f"移动距离过短({distance:.2f}米)，跳过移动")
            return
            
        control = ht_control()
        control.mode = 1
        control.x = self.V
        control.y = 0
        control.stop = 0
        
        # 基于距离计算移动时间
        duration = max(0.5, min(5.0, distance / (self.V / 10)))  # 约束时间范围0.5-5秒
        
        rospy.loginfo(f"开始直线移动: 距离={distance:.2f}米, 预计时间={duration:.1f}s")
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < rospy.Duration(duration) and not self.stop_event.is_set():
            self.pub.publish(control)
            self.rate.sleep()
            
    def stop_robot(self):
        """仅在实际需要时停止"""
        # 在execute_motion中已经做了限制，这里可以更简化
        if not self.is_complete:  # 如果运动未完成则发送停止信号
            control = ht_control()
            control.mode = 1
            control.x = 0
            control.y = 0
            control.stop = 1
            self.pub.publish(control)
            rospy.loginfo("车辆停止")
            # rospy.loginfo_once("车辆停止")

class PathRecorder:
    """路径记录器，管理路径点的存储和加载"""
    def __init__(self, listname):
        self.listname = listname.split('.')[0]   # 确保无后缀 
        base_dir = os.path.dirname(os.path.abspath(__file__)) 
        self.json_dir = os.path.join(base_dir, "paths_json")
        self.path_file = os.path.join(self.json_dir, f"{self.listname}.json") 
        
        # 确保目录存在 
        if not os.path.exists(self.json_dir): 
            os.makedirs(self.json_dir) 
        
    def load(self):
        """从文件加载路径点"""
        if not os.path.exists(self.path_file):
            return []  # 文件不存在时返回空列表
            
        try:
            with open(self.path_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"加载路径失败: {str(e)}")
            return []
    
    def save(self, points):
        """保存路径点到文件"""
        try:
            with open(self.path_file, "w", encoding="utf-8") as f:
                json.dump(points, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            rospy.logerr(f"保存路径失败: {str(e)}")
            return False