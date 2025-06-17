#!/usr/bin/env python3
import rospy
import json
import numpy as np
from ros_ht_msg.msg import ht_control, gps_filtered, agrobot_cmd

class TankMotion:
    def __init__(self, file_path=None, V=100, L=174, Kp=1.0):
        self.V = V
        self.L = L
        self.Kp = Kp
        self.motion_sequence = []
        self.current_gps = None
        self.initial_point = None
        self.stop_triggered = False
        self.gps_sub = rospy.Subscriber('/gps/filtered', gps_filtered, self.gps_callback)
        self.cmd_sub = rospy.Subscriber('/Agrobot_cmd', agrobot_cmd, self.cmd_callback)
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.data_points = []

        if file_path is None:
            rospy.logerr("JSON 文件路径未指定，程序终止。")
            rospy.signal_shutdown("JSON 文件路径未指定")
            return

        rospy.loginfo("等待 GPS 数据...")
        start_time = rospy.get_time()
        while self.current_gps is None and not rospy.is_shutdown():
            if rospy.get_time() - start_time > 10:
                rospy.logerr("未能获取 GPS 数据，程序终止。")
                rospy.signal_shutdown("无 GPS 数据")
                return
            rospy.sleep(0.5)

        self.insert_current_gps_to_json(file_path)
        with open(file_path, "r") as file:
            self.data_dic = json.load(file)
            self.data_points = self.data_dic["data"]
        self.initial_point = self.data_points[0]

    def gps_callback(self, gps_msg):
        self.current_gps = gps_msg
        rospy.loginfo(f"收到 GPS 数据: {gps_msg.latitude}, {gps_msg.longitude}, {gps_msg.yaw}")

    def insert_current_gps_to_json(self, file_path):
        if self.current_gps is None:
            rospy.logerr("当前 GPS 数据为空，无法插入到 JSON 文件。")
            return

        current_gps_data = {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "yaw": self.current_gps.yaw
        }

        try:
            with open(file_path, "r") as file:
                data_dic = json.load(file)

            data_dic["data"].insert(0, current_gps_data)

            with open(file_path, "w") as file:
                json.dump(data_dic, file, indent=4)

            rospy.loginfo(f"已将当前 GPS 数据插入到 {file_path} 文件中。")
        except Exception as e:
            rospy.logerr(f"插入 GPS 数据到 JSON 文件时出错: {e}")

    def cmd_callback(self, msg):
        if msg.stop:
            rospy.loginfo("收到停止指令，记录当前位置并返航。")
            self.stop_triggered = True
            stop_point = {
                "latitude": self.current_gps.latitude,
                "longitude": self.current_gps.longitude,
                "yaw": self.current_gps.yaw
            }
            self.data_points.append(stop_point)
            self.data_points.append(self.initial_point)
            self.motion_sequence = self.get_motion_sequence()
            BasePlateCommand(self.motion_sequence, self.V, self.L)

    def get_motion_sequence(self):
        sequence = []
        for i in range(len(self.data_points) - 1):
            start = self.data_points[i]
            end = self.data_points[i + 1]
            rospy.loginfo(f"前往第 {i+1} 个点 -> 第 {i+2} 个点")
            rospy.loginfo(f"目标坐标：纬度 {end['latitude']}, 经度 {end['longitude']}")
            motion_params = self.compute_motion_params(
                start["latitude"], start["longitude"], start["yaw"],
                end["latitude"], end["longitude"]
            )
            sequence.append(motion_params)
        return sequence

    def compute_motion_params(self, lat, lon, yaw, target_lat, target_lon):
        lat1, lon1, yaw = map(float, [lat, lon, yaw])
        lat2, lon2 = map(float, [target_lat, target_lon])
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        target_angle = np.arctan2(delta_lon, delta_lat)

        yaw_error = (target_angle - yaw + np.pi) % (2 * np.pi) - np.pi
        delta = self.Kp * yaw_error

        distance = np.sqrt(delta_lat**2 + delta_lon**2) * 6371000  # 近似地球半径换算

        return (delta, distance)

class BasePlateCommand:
    def __init__(self, motion_sequence, V=100, L=174):
        self.pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)
        self.V = V
        self.L = L
        self.motion_sequence = motion_sequence
        self.paused = False
        self.run()

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def run(self):
        for i, (omega, distance) in enumerate(self.motion_sequence):
            self.rotate(omega)
            self.move_straight(distance)

    def rotate(self, omega):
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = self.L if omega > 0 else -self.L
        control.stop = 0
        duration = abs(omega) / self.L
        rospy.loginfo(f"旋转中... 角速度: {omega:.3f} rad/s")
        end_time = rospy.get_time() + duration
        while rospy.get_time() < end_time and not rospy.is_shutdown():
            if self.paused:
                rospy.loginfo("动作已暂停，等待恢复...")
                rospy.sleep(0.5)
                continue
            self.pub.publish(control)
            rospy.sleep(0.1)
        self.stop()

    def move_straight(self, distance):
        control = ht_control()
        control.mode = 1
        control.x = self.V
        control.y = 0
        control.stop = 0
        duration = distance / (self.V / 1000.0)
        rospy.loginfo(f"直线行驶中... 目标距离: {distance:.3f} m")
        end_time = rospy.get_time() + duration
        while rospy.get_time() < end_time and not rospy.is_shutdown():
            if self.paused:
                rospy.loginfo("动作已暂停，等待恢复...")
                rospy.sleep(0.5)
                continue
            self.pub.publish(control)
            rospy.sleep(0.1)
        self.stop()

    def stop(self):
        control = ht_control()
        control.mode = 1
        control.x = 0
        control.y = 0
        control.stop = 1
        self.pub.publish(control)
        rospy.loginfo("车辆停止")

class AgrobotCommander:
    def __init__(self, json_file):
        self.json_file = json_file
        self.motion_engine = TankMotion(json_file)
        self.driver = BasePlateCommand(self.motion_engine.motion_sequence, self.motion_engine.V, self.motion_engine.L)
        self.stop_received = False
        self.start_position = self.motion_engine.initial_point
        rospy.Subscriber("/Agrobot_cmd", agrobot_cmd, self.stop_callback)
        self.log_file = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/stop_log.json"

    def stop_callback(self, msg):
        if msg.stop:
            rospy.logwarn("收到 Agrobot 停止指令！记录当前坐标并暂停...")
            self.stop_received = True
            self.driver.pause()
            self.save_current_gps_loop()
            self.return_to_home()

    def save_current_gps_loop(self):
        rate = rospy.Rate(1)
        with open(self.log_file, 'w') as f:
            while self.stop_received and not rospy.is_shutdown():
                if self.motion_engine.current_gps:
                    data = {
                        "latitude": self.motion_engine.current_gps.latitude,
                        "longitude": self.motion_engine.current_gps.longitude,
                        "yaw": self.motion_engine.current_gps.yaw,
                        "time": rospy.get_time()
                    }
                    json.dump(data, f, indent=2)
                    rospy.loginfo(f"保存 GPS 停止点: {data}")
                rate.sleep()
                self.stop_received = False

    def return_to_home(self):
        stop_point = {
            "latitude": self.motion_engine.current_gps.latitude,
            "longitude": self.motion_engine.current_gps.longitude,
            "yaw": self.motion_engine.current_gps.yaw,
            "desc": "停止点"
        }

        home_point = {
            "latitude": self.start_position["latitude"],
            "longitude": self.start_position["longitude"],
            "yaw": self.start_position["yaw"],
            "desc": "起始点"
        }

        delta, distance = self.motion_engine.compute_motion_params(
            stop_point["latitude"], stop_point["longitude"], stop_point["yaw"],
            home_point["latitude"], home_point["longitude"]
        )

        rospy.loginfo("执行返航...")
        self.driver = BasePlateCommand([(delta, distance)], self.motion_engine.V, self.motion_engine.L)

    def run(self):
        self.driver = BasePlateCommand(self.motion_engine.motion_sequence, self.motion_engine.V, self.motion_engine.L)

if __name__ == "__main__":
    rospy.init_node("agrobot_return_node", anonymous=True)
    try:
        file_path = "/home/ayx/ht_ws/src/ros_ht_msg/test_code/1.json"
        commander = AgrobotCommander(file_path)
        commander.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
