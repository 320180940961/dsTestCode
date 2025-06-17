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

        # 等待获取初始 GPS 数据，最长等待15秒
        rospy.loginfo("等待获取初始 GPS 数据...")
        start_time = rospy.get_time()
        while not self.current_gps and not rospy.is_shutdown():
            if rospy.get_time() - start_time > 15:
                rospy.logerr("没有 GPS 话题信息，程序终止！")
                rospy.signal_shutdown("没有 GPS 话题信息")
                return
            rospy.sleep(1)

        # 将当前 GPS 数据插入到 JSON 文件的第一个位置
        self.insert_current_gps_to_json(file_path)

        # 重新加载更新后的 JSON 文件
        with open(file_path, "r") as file:
            self.data_dic = json.load(file)
            self.data_points = self.data_dic["data"]

        self.compute_motion_sequence()

    def gps_callback(self, gps_msg):
        """GPS 数据回调函数"""
        self.current_gps = gps_msg
        rospy.loginfo(f"收到 GPS 数据: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.yaw}")

    def insert_current_gps_to_json(self, file_path):
        """将当前 GPS 数据插入到 JSON 文件的第一个位置"""
        current_gps_data = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }

        # 读取原始 JSON 文件内容
        with open(file_path, "r") as file:
            data_dic = json.load(file)

        # 将当前 GPS 数据插入到第一个位置
        data_dic["data"].insert(0, current_gps_data)

        # 将更新后的数据写回到 JSON 文件
        with open(file_path, "w") as file:
            json.dump(data_dic, file, indent=4)

        rospy.loginfo(f"已将当前 GPS 数据插入到 {file_path} 文件中。")

    def compute_motion_sequence(self):
        """计算运动序列"""
        motion_sequence = []
        for i in range(len(self.data_points) - 1):
            lat = self.data_points[i]["latitude"]
            lon = self.data_points[i]["longitude"]
            yaw = self.data_points[i]["yaw"]
            target_lat = self.data_points[i + 1]["latitude"]
            target_lon = self.data_points[i + 1]["longitude"]

            # 计算运动参数
            motion_params = self.compute_motion_params(lat, lon, yaw, target_lat, target_lon)
            motion_sequence.append(motion_params)
        self.motion_sequence = motion_sequence
        rospy.loginfo(f"运动序列: {self.motion_sequence}")

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
        """计算运动参数"""
        try:
            lat, lon, yaw, target_lat, target_lon = map(float, [lat, lon, yaw, target_lat, target_lon])
        except ValueError as e:
            raise ValueError(f"Invalid input data: {e}, received {lat}, {lon}, {yaw}, {target_lat}, {target_lon}")

        lat1, lon1 = np.radians(lat), np.radians(lon)
        lat2, lon2 = np.radians(target_lat), np.radians(target_lon)

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        target_angle = np.arctan2(delta_lon, delta_lat)

        yaw_error = target_angle - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        delta = self.Kp * yaw_error

        return self.V, delta

class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.run()

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
        rospy.init_node("gps_filtered", anonymous=True)
        json_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        tank = TankMotion(json_file)
        if not tank.motion_sequence:
            rospy.logerr("运动序列为空，程序终止！")
            rospy.signal_shutdown("没有 GPS 信息，运动序列为空")
        else:
            BasePlateCommand(tank.motion_sequence)
    except rospy.ROSInterruptException:
        pass