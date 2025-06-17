#!/usr/bin/env python3
import rospy
import json
import threading
import time
from ros_ht_msg.msg import gps_filtered, ht_control,agrobot_cmd

class AgrobotController:
    def __init__(self, output_json_path="/home/ayx/ht_ws/src/ros_ht_msg/test_code/gps_log.json"):
        self.stop_flag = False
        self.current_gps = None
        self.lock = threading.Lock()
        self.gps_data_list = []
        self.output_json_path = output_json_path

        # 订阅 GPS 和停止命令话题
        rospy.Subscriber("/gps/filtered", gps_filtered, self.gps_callback)
        rospy.Subscriber("/Agrobot_cmd", agrobot_cmd, self.cmd_callback)
        
        self.control_pub = rospy.Publisher("/HT_Control", ht_control, queue_size=10)

        # 启动记录线程
        self.record_thread = threading.Thread(target=self.record_gps_loop)
        self.record_thread.daemon = True
        self.record_thread.start()

        rospy.loginfo("Agrobot Controller 初始化完成，等待命令...")

    def gps_callback(self, msg):
        with self.lock:
            self.current_gps = msg

    def cmd_callback(self, msg):
        if msg.data.strip().lower() == "stop":
            rospy.logwarn("收到停止命令，停止车辆并开始记录GPS位置...")
            self.stop_flag = True
            self.send_stop_command()

    def send_stop_command(self):
        stop_msg = ht_control()
        stop_msg.mode = 1
        stop_msg.x = 0
        stop_msg.y = 0
        stop_msg.stop = 1
        self.control_pub.publish(stop_msg)
        rospy.loginfo("发送停止指令到 /HT_Control")

    def record_gps_loop(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            if self.stop_flag and self.current_gps:
                with self.lock:
                    data = {
                        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "latitude": self.current_gps.latitude,
                        "longitude": self.current_gps.longitude,
                        "yaw": self.current_gps.yaw
                    }
                    self.gps_data_list.append(data)
                    self.write_to_file()
            rate.sleep()

    def write_to_file(self):
        try:
            with open(self.output_json_path, 'w') as f:
                json.dump({"gps_log": self.gps_data_list}, f, indent=4)
                rospy.loginfo("GPS 数据写入文件")
        except Exception as e:
            rospy.logerr(f"写入 JSON 文件出错: {e}")

if __name__ == "__main__":
    rospy.init_node("agrobot_logger", anonymous=True)
    controller = AgrobotController()
    rospy.spin()
