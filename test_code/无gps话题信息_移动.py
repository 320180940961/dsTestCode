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
        self.pub = rospy.Publisher("/gps/filtered", gps_filtered, queue_size=10)
        self.current_gps = None  # 当前 GPS 数据
        self.gps_sub = rospy.Subscriber('/gps/filtered', gps_filtered, self.gps_callback)
        self.motion_sequence = []

        # 读取 JSON 文件
        with open(file_path, "r") as file:
            self.data_dic = json.load(file)
            self.data_points = self.data_dic["data"]

        rospy.loginfo("等待获取初始 GPS 数据...")
        timeout = rospy.Time.now() + rospy.Duration(5)  # 设置 5 秒超时
        while not self.current_gps and rospy.Time.now() < timeout and not rospy.is_shutdown():
            rospy.sleep(0.1)

        if self.current_gps:
            self.compute_motion_sequence()
        else:
            print("无 GPS 信息，使用 JSON 文件路径数据")
            self.motion_sequence = [(float(point["latitude"]), float(point["longitude"])) for point in self.data_points]

    def gps_callback(self, gps_msg):
        """GPS 数据回调函数"""
        self.current_gps = gps_msg
        rospy.loginfo(f"收到 GPS 数据: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.yaw}")

    def compute_motion_sequence(self):
        """计算运动序列"""
        start_point = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }
        target_point = self.data_points[0]

        motion_params = self.compute_motion_params(
            start_point["latitude"], start_point["longitude"], start_point["yaw"],
            target_point["latitude"], target_point["longitude"]
        )
        self.motion_sequence.append(motion_params)
        rospy.loginfo(f"运动序列: {self.motion_sequence}")

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
        """计算运动参数"""
        lat, lon, yaw, target_lat, target_lon = map(float, [lat, lon, yaw, target_lat, target_lon])
        
        lat1, lon1 = np.radians(lat), np.radians(lon)
        lat2, lon2 = np.radians(target_lat), np.radians(target_lon)

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        target_angle = np.arctan2(delta_lon, delta_lat)

        yaw_error = target_angle - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        delta = self.Kp * yaw_error
        return float(self.V), float(delta)

class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz 频率
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.run()

    def run(self):
        for omega, distance in self.motion_sequence:
            self.rotate(float(omega))
            self.move_straight(float(distance))

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
            self.rate.sleep()
        rospy.sleep(duration % 1)  # 处理小数部分
        self.stop()

    def move_straight(self, distance):
        control = ht_control()
        control.mode = 1
        control.x = self.V
        control.y = 0
        control.stop = 0
        self.pub.publish(control)
        rospy.loginfo(f"直线行驶中... 目标距离: {distance:.3f} m")
        duration = distance / (self.V / 1000)
        for _ in range(int(duration)):
            self.pub.publish(control)
            rospy.sleep(1)
        rospy.sleep(duration % 1)  # 处理小数部分
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
        rospy.init_node("base_plate_controller", anonymous=True)
        json_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        tank = TankMotion(json_file)
        motion_sequence = tank.motion_sequence
        BasePlateCommand(motion_sequence)
    except rospy.ROSInterruptException:
        pass
