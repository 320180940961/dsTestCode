#!/usr/bin/env python3
import rospy
import json
import numpy as np
from ros_ht_msg.msg import ht_control, gps_filtered

class TankMotion:
    def __init__(self, file_path, V=100, L=174, Kp=1.0):
        self.V = V
        self.L = L
        self.Kp = Kp
        self.motion_sequence = []
        self.current_gps = None
        self.origin_point = None

        self.gps_sub = rospy.Subscriber('/gps/filtered', gps_filtered, self.gps_callback)
        self.pub = rospy.Publisher("/gps/filtered", gps_filtered, queue_size=10)

        # 读取 JSON 文件
        with open(file_path, "r") as file:
            self.data_dic = json.load(file)
            self.data_points = self.data_dic["data"]

        # 等待初始 GPS
        rospy.loginfo("等待获取初始 GPS 数据...")
        start_time = rospy.get_time()
        while not self.current_gps and not rospy.is_shutdown():
            if rospy.get_time() - start_time > 15:
                rospy.logerr("没有 GPS 话题信息，程序终止！")
                rospy.signal_shutdown("没有 GPS 话题信息")
                return
            rospy.sleep(1)

        # 保存初始位置
        self.origin_point = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }
        self.compute_motion_sequence(self.data_points)

    def gps_callback(self, gps_msg):
        self.current_gps = gps_msg
        rospy.loginfo_throttle(5, f"[GPS] 当前: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.yaw}")

    def compute_motion_sequence(self, points):
        motion_sequence = []

        current_point = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }

        for target in points:
            motion_params = self.compute_motion_params(
                current_point["latitude"], current_point["longitude"], current_point["yaw"],
                target["latitude"], target["longitude"]
            )
            motion_sequence.append(motion_params)

            current_point = target  # 更新当前点
            current_point["yaw"] = 0  # 假设无 yaw 更新

        self.motion_sequence = motion_sequence

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
        try:
            lat, lon, yaw, target_lat, target_lon = map(float, [lat, lon, yaw, target_lat, target_lon])
        except ValueError as e:
            raise ValueError(f"数据错误: {e}, received {lat}, {lon}, {yaw}, {target_lat}, {target_lon}")

        lat1, lon1 = np.radians(lat), np.radians(lon)
        lat2, lon2 = np.radians(target_lat), np.radians(target_lon)

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        target_angle = np.arctan2(delta_lon, delta_lat)

        yaw_error = target_angle - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        delta = self.Kp * yaw_error

        # 计算距离
        distance = np.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2) * 6371000  # 地球半径
        return delta, distance  # 返回角度偏差 & 距离

    def return_to_origin(self):
        """生成返航运动序列：从当前位置 -> 初始位置"""
        rospy.loginfo("生成返航路径...")

        if not self.origin_point:
            rospy.logwarn("没有记录初始位置，无法返航")
            return

        return_point = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }

        # 记录停止点
        stop_data = {
            "stop_point": {
                "latitude": return_point["latitude"],
                "longitude": return_point["longitude"],
                "yaw": return_point["yaw"]
            }
        }

        with open("/home/ayx/ht_ws/src/ros_ht_msg/test_code/stop_point.json", "w") as f:
            json.dump(stop_data, f, indent=4)

        # 创建返航路径
        return_path = [self.origin_point]
        self.compute_motion_sequence(return_path)  # 生成新运动序列

        rospy.loginfo("返航路径生成完成，准备执行返航")
        return self.motion_sequence


class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L

    def run(self):
        for omega, distance in self.motion_sequence:
            self.rotate(omega)
            self.move_straight(distance)

    def rotate(self, omega):
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = self.L if omega > 0 else -self.L
        control.stop = 0
        self.pub.publish(control)
        rospy.loginfo(f"旋转中... 角速度: {omega:.3f} rad/s")
        duration = abs(omega) / self.L
        for _ in range(int(duration)):
            self.pub.publish(control)
            rospy.sleep(1)
        rospy.sleep(duration % 1)
        self.stop()

    def move_straight(self, distance):
        control = ht_control()
        control.mode = 1
        control.x = self.V
        control.y = 0
        control.stop = 0
        self.pub.publish(control)
        rospy.loginfo(f"直线行驶中... 距离: {distance:.2f}m")
        duration = distance / (self.V / 1000)
        for _ in range(int(duration)):
            self.pub.publish(control)
            rospy.sleep(1)
        rospy.sleep(duration % 1)
        self.stop()

    def stop(self):
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = 0
        control.stop = 1
        self.pub.publish(control)
        rospy.loginfo("车辆停止")

if __name__ == "__main__":
    try:
        rospy.init_node("return_home", anonymous=True)
        json_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        tank = TankMotion(json_file)
        command = BasePlateCommand(tank.motion_sequence)
        command.run()

        rospy.sleep(2)  # 模拟任务完成或接收到指令

        # ===== 开始返航 =====
        return_seq = tank.return_to_origin()
        if return_seq:
            command = BasePlateCommand(return_seq)
            command.run()

    except rospy.ROSInterruptException:
        pass
