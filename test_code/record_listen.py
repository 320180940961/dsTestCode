#!/usr/bin/env python3
import rospy
import json
import numpy as np
import threading
import os
from ros_ht_msg.msg import ht_control

# 全局事件变量，用于控制线程停止
stop_event = threading.Event()

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

    def run(self):
        """
        持续执行运动指令：先旋转后直行
        """
        for omega, distance in self.motion_sequence:
            if stop_event.is_set():
                self.stop()
                break
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


class RecordMode:
    def __init__(self):
        self.data_points = []

    def add_point(self, lat, lon, yaw):
        self.data_points.append({"latitude": lat, "longitude": lon, "yaw": yaw})

    def insert_point(self, index, lat, lon, yaw):
        if 0 <= index < len(self.data_points):
            self.data_points.insert(index, {"latitude": lat, "longitude": lon, "yaw": yaw})
        else:
            print("插入位置无效")

    def delete_point(self, index):
        if 0 <= index < len(self.data_points):
            del self.data_points[index]
        else:
            print("删除位置无效")

    def save_to_json(self, file_name):
        data_dic = {"data": self.data_points}
        with open(file_name, "w") as file:
            json.dump(data_dic, file, indent=4)
        print(f"数据已保存到 {file_name}")


def record_mode():
    print("进入记录模式")
    file_name = input("请输入新建的文件名（.json）：")
    if not file_name.endswith(".json"):
        file_name += ".json"

    recorder = RecordMode()

    while True:
        print("\n操作选项：")
        print("a - 添加点")
        print("b - 插入点")
        print("c - 删除点")
        print("q - 退出并保存")
        choice = input("请输入操作选项：")

        if choice == "a":
            lat = float(input("请输入纬度："))
            lon = float(input("请输入经度："))
            yaw = float(input("请输入偏航角："))
            recorder.add_point(lat, lon, yaw)
            print("点已添加")
        elif choice == "b":
            index = int(input("请输入插入位置（索引）："))
            lat = float(input("请输入纬度："))
            lon = float(input("请输入经度："))
            yaw = float(input("请输入偏航角："))
            recorder.insert_point(index, lat, lon, yaw)
            print("点已插入")
        elif choice == "c":
            index = int(input("请输入删除位置（索引）："))
            recorder.delete_point(index)
            print("点已删除")
        elif choice == "q":
            recorder.save_to_json(file_name)
            break
        else:
            print("无效的操作选项")


def idle_mode():
    print("进入闲置模式")
    while not stop_event.is_set():
        rospy.sleep(1)

def move_to_target_mode(data_points):
    print("进入移动到指定目标点模式")
    while not stop_event.is_set():
        for point in data_points:
            lat = point["latitude"]
            lon = point["longitude"]
            yaw = point["yaw"]
            print(f"移动到目标点：纬度 {lat}, 经度 {lon}, 偏航角 {yaw}")
            rospy.sleep(1)  # 模拟移动过程
        if stop_event.is_set():
            break

def return_to_home_mode(data_points):
    print("进入回航模式")
    while not stop_event.is_set():
        for point in reversed(data_points):
            lat = point["latitude"]
            lon = point["longitude"]
            yaw = point["yaw"]
            print(f"回航到点：纬度 {lat}, 经度 {lon}, 偏航角 {yaw}")
            rospy.sleep(1)  # 模拟移动过程
        if stop_event.is_set():
            break


def continue_working_mode(data_points):
    print("进入继续工作模式")
    while not stop_event.is_set():
        for point in data_points:
            lat = point["latitude"]
            lon = point["longitude"]
            yaw = point["yaw"]
            print(f"继续工作，当前点：纬度 {lat}, 经度 {lon}, 偏航角 {yaw}")
            rospy.sleep(1)  # 模拟工作过程
        if stop_event.is_set():
            break


def listener_mode():
    print("进入监听模式")
    json_file = input("请输入JSON文件名（.json）：")
    if not json_file.endswith(".json"):
        json_file += ".json"

    if not os.path.exists(json_file):
        print(f"文件 {json_file} 不存在")
        return

    with open(json_file, "r") as file:
        data_dic = json.load(file)
        data_points = data_dic["data"]

    mode_threads = {
        "1": threading.Thread(target=idle_mode),
        "2": threading.Thread(target=move_to_target_mode, args=(data_points,)),
        "3": threading.Thread(target=return_to_home_mode, args=(data_points,)),
        "4": threading.Thread(target=continue_working_mode, args=(data_points,))
    }

    current_mode_thread = None

    while True:
        if stop_event.is_set():
            if current_mode_thread and current_mode_thread.is_alive():
                stop_event.clear()
                current_mode_thread.join()
            current_mode_thread = None
            print("停止当前模式，等待用户输入")
            continue

        if not current_mode_thread or not current_mode_thread.is_alive():
            mode_input = input("请输入模式（1-闲置，2-移动到目标点，3-回航，4-继续工作）：")
            if mode_input in mode_threads:
                if current_mode_thread:
                    stop_event.set()
                    current_mode_thread.join()
                current_mode_thread = mode_threads[mode_input]
                stop_event.clear()
                current_mode_thread.start()
            else:
                print("无效的模式输入")


if __name__ == "__main__":
    try:
        mode_choice = input("请输入模式选择（1-记录模式，2-监听模式）：")
        if mode_choice == "1":
            record_mode()
        elif mode_choice == "2":
            listener_mode()
        else:
            print("无效的模式选择")
    except rospy.ROSInterruptException:
        pass