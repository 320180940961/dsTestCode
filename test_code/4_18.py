#!/usr/bin/env python3
import rospy
import json
import numpy as np
import threading
import os
from ros_ht_msg.msg import ht_control, gps_filtered

# 全局事件，用于控制线程停止
stop_event = threading.Event()

class TankMotion:
    def __init__(self, file_path, V=100, L=174, Kp=1.0):
        self.V = V
        self.L = L
        self.Kp = Kp
        self.pub = rospy.Publisher("/gps/filtered", gps_filtered, queue_size=10)
        self.current_gps = None
        self.gps_sub = rospy.Subscriber('/gps/filtered', gps_filtered, self.gps_callback)
        self.motion_sequence = []
        self.file_path = file_path

        # 检查文件是否存在
        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                self.data_dic = json.load(file)
                self.data_points = self.data_dic["data"]
        else:
            self.data_points = []

    def gps_callback(self, gps_msg):
        self.current_gps = gps_msg
        rospy.loginfo(f"收到 GPS 数据: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.yaw}")

    def compute_motion_sequence(self):
        start_point = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }
        target_point = self.data_points[0] if self.data_points else start_point

        motion_params = self.compute_motion_params(
            start_point["latitude"], start_point["longitude"], start_point["yaw"],
            target_point["latitude"], target_point["longitude"]
        )
        self.motion_sequence.append(motion_params)
        rospy.loginfo(f"运动序列: {self.motion_sequence}")

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
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

    def save_points(self):
        with open(self.file_path, "w") as file:
            json.dump({"data": self.data_points}, file)
        rospy.loginfo(f"点信息已保存到 {self.file_path}")

    def record_mode(self):
        rospy.loginfo("进入记录模式，按 a 添加点，b 插入点，c 删除点")
        while not rospy.is_shutdown():
            key = input("请输入命令 (a: 添加点, b: 插入点, c: 删除点): ")
            if key == 'a':
                if self.current_gps:
                    self.data_points.append({
                        "latitude": self.current_gps.latitude,
                        "longitude": self.current_gps.longitude
                    })
                    rospy.loginfo("添加点成功")
                else:
                    rospy.loginfo("无 GPS 数据，无法添加点")
            elif key == 'b':
                if self.current_gps:
                    index = int(input("请输入插入点的索引: "))
                    if 0 <= index <= len(self.data_points):
                        self.data_points.insert(index, {
                            "latitude": self.current_gps.latitude,
                            "longitude": self.current_gps.longitude
                        })
                        rospy.loginfo("插入点成功")
                    else:
                        rospy.loginfo("索引无效")
                else:
                    rospy.loginfo("无 GPS 数据，无法插入点")
            elif key == 'c':
                if self.data_points:
                    index = int(input("请输入删除点的索引: "))
                    if 0 <= index < len(self.data_points):
                        del self.data_points[index]
                        rospy.loginfo("删除点成功")
                    else:
                        rospy.loginfo("索引无效")
                else:
                    rospy.loginfo("无点可删除")
            else:
                rospy.loginfo("无效命令")
            self.save_points()

class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz 频率
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L

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

class ListenerThread(threading.Thread):
    def __init__(self, motion_sequence, V=100, L=174):
        super().__init__()
        self.motion_sequence = motion_sequence
        self.V = V
        self.L = L
        self.mode = 1  # 默认闲置模式
        self.command = None

    def run(self):
        rospy.loginfo("进入监听模式")
        while not stop_event.is_set():
            if self.mode == 2:  # 移动到指定目标点模式
                self.move_to_target()
            elif self.mode == 3:  # 回航模式
                self.return_to_start()
            elif self.mode == 4:  # 继续工作模式
                self.continue_working()
            rospy.sleep(0.1)

    def move_to_target(self):
        rospy.loginfo("进入移动到指定目标点模式")
        target_index = int(input("请输入目标点的索引: "))
        if 0 <= target_index < len(self.motion_sequence):
            target_point = self.motion_sequence[target_index]
            self.move_to_point(target_point)
        else:
            rospy.loginfo("目标点索引无效")
        self.mode = 1  # 完成后回到闲置模式

    def return_to_start(self):
        rospy.loginfo("进入回航模式")
        start_point = self.motion_sequence[0]
        self.move_to_point(start_point)
        self.mode = 1  # 完成后回到闲置模式

    def continue_working(self):
        rospy.loginfo("进入继续工作模式")
        self.run()
        self.mode = 1  # 完成后回到闲置模式

    def move_to_point(self, point):
        rospy.loginfo(f"移动到点: {point}")
        # 这里可以调用 BasePlateCommand 的方法来控制小车移动
        # 为了简化，这里仅打印信息
        rospy.loginfo(f"移动到点: {point}")

if __name__ == "__main__":
    try:
        rospy.init_node("base_plate_controller", anonymous=True)
        json_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        tank = TankMotion(json_file)
        motion_sequence = tank.motion_sequence

        listener_thread = ListenerThread(motion_sequence)
        listener_thread.start()

        rospy.loginfo("主线程等待键盘输入")
        while not rospy.is_shutdown():
            key = input("请输入命令 (r: 记录模式, 空格: 停止): ")
            if key == 'r':
                tank.record_mode()
            elif key == ' ':
                stop_event.set()
                rospy.loginfo("停止所有线程")
                break
            else:
                rospy.loginfo("无效命令")

        listener_thread.join()
    except rospy.ROSInterruptException:
        pass