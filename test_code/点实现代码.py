#!/usr/bin/env python3
import rospy
import json
import numpy as np
from ros_ht_msg.msg import ht_control

class TankMotion:
    def __init__(self, file_path, V=100, L=174, Kp=1.0):
        self.V = V
        self.L = L
        self.Kp = Kp

        # 读取 JSON 文件
        with open(file_path, "r") as file:
            self.data_dic = json.load(file)
            self.data_points = self.data_dic["data"]

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
        """计算运动参数"""
        # 转换数据类型
        try:
            lat, lon, yaw, target_lat, target_lon = map(float, [lat, lon, yaw, target_lat, target_lon])
        except ValueError as e:
            raise ValueError(f"Invalid input data: {e}, received {lat}, {lon}, {yaw}, {target_lat}, {target_lon}")

        # 角度转换为弧度
        lat1, lon1 = np.radians(lat), np.radians(lon)
        lat2, lon2 = np.radians(target_lat), np.radians(target_lon)

        # 计算目标方向
        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        target_angle = np.arctan2(delta_lon, delta_lat)

        # 计算角度误差
        yaw_error = target_angle - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  # 归一化到 [-π, π]

        # 计算转向角
        delta = self.Kp * yaw_error

        return self.V, delta

    def get_motion_sequence(self):
        """获取运动序列"""
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

        return motion_sequence

class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        """
        车辆运动控制类
        :param motion_sequence: 计算出的运动参数序列 [(omega, D), ...]
        :param V: 行驶速度
        :param L: 旋转速度
        """
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        rospy.init_node("base_plate_controller", anonymous=True)
        self.rate = rospy.Rate(1)  # 1Hz 频率
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.run()

    def run(self):
        """
        持续执行运动指令：先旋转后直行
        """
        for omega, distance in self.motion_sequence:
            self.rotate(omega)
            self.move_straight(distance)

    def rotate(self, omega):
        """
        控制履带车旋转
        :param omega: 角速度 (rad/s)
        """
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = self.L if omega > 0 else -self.L
        control.stop = 0
        self.pub.publish(control)
        rospy.loginfo(f"旋转中... 角速度: {omega:.3f} rad/s")

        rotate_time = abs(omega) / self.L
        rospy.sleep(rotate_time)  # 旋转所需时间
        self.stop()

    def move_straight(self, distance):
        """
        控制履带车直线行驶
        :param distance: 目标行驶距离 (m)
        """
        control = ht_control()
        control.mode = 1
        control.x = self.V
        control.y = 0
        control.stop = 0
        self.pub.publish(control)
        rospy.loginfo(f"直线行驶中... 目标距离: {distance:.3f} m")

        move_time = distance / (self.V / 1000)  # cm/s 转换为 m/s
        rospy.sleep(move_time)  # 行驶所需时间
        self.stop()

    def stop(self):
        """
        发送停止指令
        """
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = 0
        control.stop = 1
        self.pub.publish(control)
        rospy.loginfo("车辆停止")


if __name__ == "__main__":
    try:
        json_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        tank = TankMotion(json_file)
        motion_sequence = tank.get_motion_sequence()

        BasePlateCommand(motion_sequence)

    except rospy.ROSInterruptException:
        pass

# 获取/gps话题，将当前位置的gps作为第一个点，json文件的第一个点作为第二个点