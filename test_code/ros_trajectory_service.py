#!/usr/bin/env python3
import rospy
import os
import logging
import json
import math
from icecream import ic  
from std_msgs import String
import paho.mqtt.client as mqtt 
from ros_ht_msg.msg import ht_control

    # 1、订阅/gps/filtered 发布的话题信息
    # 2、读取给定的json文件信息，遍历json文件的所有信息，输出一共有几个字典，给每一个字典排序。 
    # 3、写一个函数cruises() ，读取json文件，根据angle、latitude、longitude三个参数，计算车的具体速度，并使车可以移动到具体的位置（话题是1s接受一个消息队列），控制车移动的话题是ht_control,ht_control接受参数是uint8 mode、int16 x、int16 y、uint8 stop
    # 4、写一个函数move_to（），根据json文件的信息，在控制板提问车移动到第几个位置并手动输入int类型的点（提示点数要小于json文件中字典总数），控制车移动到具体的数据点。
# 2、将订阅到的/gps/filtered话题所发布的信息的angle、latitude、longitude、x、y、yaw这些参数提取到新的json文件中，在生成前，提问json文件名设置成什么，不输入默认为1.json(1S记录一个、或者按空格记录一个点，定点录制)

# 1、订阅/gps/filtered 发布的话题信息
def gps_callback():
    pub = rospy.Publisher('/gps/filtered',String,queue_size=200)
    rospy.init_node('')

# 计算速度
def calculate_speed(angle1, lat1, lon1, angle2, lat2, lon2, time_diff):
    R = 6371000  # 地球半径（单位：米）
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c  # 计算两点之间的距离
    
    speed = distance / time_diff  # 计算速度 (m/s)
    return speed

# 2、读取给定的json文件信息，遍历json文件的所有信息，输出一共有几个字典，给每一个字典排序。 
def read_json(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    
    if isinstance(data, list):
        print(f"JSON 文件中包含 {len(data)} 个字典")
        for i, dictionary in enumerate(data):
            sorted_dict = dict(sorted(dictionary.items()))
            print(f"字典 {i+1}: {sorted_dict}")
        return data
    else:
        print("JSON 文件格式错误，根对象应为列表！")
        return []
    
# 控制车辆移动
def move_vehicle(mode, x, y, stop):
    control_data = {
        "mode": mode,
        "x": x,
        "y": y,
        "stop": stop
    }
    client.publish(CONTROL_TOPIC, json.dumps(control_data))
    print(f"发送控制命令: {control_data}")
    
# 3、写一个函数cruises() ，读取json文件，根据angle、latitude、longitude三个参数，计算车的具体速度，并使车可以移动到具体的位置（话题是1s接受一个消息队列），控制车移动的话题是ht_control,ht_control接受参数是uint8 mode、int16 x、int16 y、uint8 stop
def cruises(filename):
    data = read_json(filename)
    if not data:
        return
    
    for i in range(len(data) - 1):
        angle1, lat1, lon1 = data[i]["angle"], data[i]["latitude"], data[i]["longitude"]
        angle2, lat2, lon2 = data[i+1]["angle"], data[i+1]["latitude"], data[i+1]["longitude"]
        
        speed = calculate_speed(angle1, lat1, lon1, angle2, lat2, lon2, time_diff=1)
        print(f"计算速度: {speed:.2f} m/s")
        move_vehicle(mode=1, x=int(lon2 * 100000), y=int(lat2 * 100000), stop=0)
        time.sleep(1)
    
    move_vehicle(mode=1, x=int(data[-1]["longitude"] * 100000), y=int(data[-1]["latitude"] * 100000), stop=1)
    print("到达最终位置，停止移动")

def move_to(filename):
    data = read_json(filename)
    if not data:
        return
    
    point_index = int(input(f"请输入要移动到的目标点 (0-{len(data)-1}): "))
    if 0 <= point_index < len(data):
        move_vehicle(mode=1, x=int(data[point_index]["longitude"] * 100000), y=int(data[point_index]["latitude"] * 100000), stop=1)
        print(f"移动到目标点 {point_index}")
    else:
        print("输入超出范围，无法移动！")


if __name__ == "__main__":
    filename = "data.json"
    cruises(filename)
    move_to(filename)