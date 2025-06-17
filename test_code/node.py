from threading import Thread, Event
from datetime import datetime
import os, json
import rospy
import copy
from ros_ht_msg.msg import gps_filtered, ht_control
import numpy as np

# 导航状态常量
NAV_STATUS_IDLE = "闲置"
NAV_STATUS_RUNNING = "导航中"
NAV_STATUS_DONE = "已完成"

# 为什么GPS数据没有更新？
# 1.可能是因为在main_loop中没有给ROS处理回调的机会（因为阻塞在win.getch()）。
# 解决方案：在调用win.getch()时设置一个超时，然后在这个超时内处理ROS回调。

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

        # GPS相关初始化
        self.current_gps = None
        rospy.Subscriber("/gps/filtered", gps_filtered, self.gps_callback)

        # 导航线程使用新创建而不是继承
        self.nav_thread = None
            
        rospy.loginfo(f"Naver {mode} mode initialized")

    def gps_callback(self, msg):
        """ROS GPS数据回调"""
        self.current_gps = msg
        self.current_position = self._format_gps_data(msg)
    
    def _format_gps_data(self, msg):
        """格式化GPS数据"""
        return {
            "lat": msg.latitude,
            "lng": msg.longitude,
            "yaw": msg.yaw,
            "x": msg.x,
            "y": msg.y,
            "angle": msg.angle,
            "time": datetime.now().strftime("%H:%M:%S")
        }

    def get_current_position(self):
        """获取当前位置"""
        # 模拟位置用于测试
        if not self.current_gps:
            return self._mock_gps()
        return self._format_gps_data(self.current_gps)
    
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
    
    def run(self):
        """重写 Thread 的 run 方法，作为线程的主要入口点"""
        rospy.loginfo("Naver 主线程启动")
        
        if self.mode == "navigation":
            self._perform_navigation()
        elif self.mode == "record":
            self._perform_recording()
        else:
            rospy.logwarn(f"未知模式: {self.mode}")
        
        rospy.loginfo("Naver 主线程结束")

    def _perform_navigation(self):
        """执行导航任务"""
        rospy.loginfo(f"准备在 {self.dir_mode} 模式下导航")
        
        # 准备导航点序列
        self._prepare_navigation()
        
        # 获取起始位置
        current_pos = self.get_current_position() or self.points[0]
        
        # 执行导航逻辑（与原有的 _navigate 功能一致）
        self.status = NAV_STATUS_RUNNING
        self.stop_event.clear()
        
        rospy.loginfo(f"导航开始 ({len(self.nav_points)} 点)")
        
        for target in self.nav_points:
            if self.stop_event.is_set():
                rospy.loginfo("导航被中断")
                break
                
            # 更新当前目标点
            self.current_target = target
            
            # 查找目标点在原始列表中的索引
            try:
                self.current_target_index = self.points.index(target)
                idx_info = f"{self.current_target_index}/{len(self.points)}"
            except ValueError:
                self.current_target_index = None
                idx_info = "未知索引"
            
            # 计算运动参数
            motion = self.compute_motion_params(self.current_position, target)
            
            # 执行移动
            self.execute_motion(motion)
            
            # 更新当前位置
            self.current_position = self.get_current_position()
            
            # 记录日志
            if self.current_position:
                rospy.loginfo(f"到达 {idx_info} 号点: ({target['lat']:.6f}, {target['lng']:.6f})")
            else:
                rospy.loginfo(f"到达 {idx_info} 号点 (无位置更新)")
                
            # 添加小延迟
            rospy.sleep(0.1)
        
        # 导航结束处理
        self.status = NAV_STATUS_DONE
        rospy.loginfo(f"{self.dir_mode} 导航完成")
        self.current_target = None
        self.current_target_index = None
        self.stop_robot()
    
    def _perform_recording(self):
        """记录模式的主循环"""
        rospy.loginfo("记录模式激活")
        self.status = NAV_STATUS_RUNNING
        
        # 记录模式不需要持续运行线程，但保持线程活跃
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            rospy.sleep(1)  # 每秒钟检查一次停止信号
            
        self.status = NAV_STATUS_IDLE
        rospy.loginfo("记录模式结束")

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
        # 确保处于导航模式
        if self.mode != "navigation":
            rospy.logwarn("只能在导航模式下启动导航")
            return False
            
        # 如果线程未运行，启动它
        if not self.is_alive():
            self.start()
            return True
        return False
    
    def stop_navigation(self):
        """停止导航线程"""
        # 设置停止标志
        self.stop_event.set()
        
        # 等待线程结束
        if self.is_alive():
            self.join(timeout=2.0)
        
        self.status = NAV_STATUS_IDLE
        rospy.loginfo("导航已停止")
        return True
    

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