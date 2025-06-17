#!/usr/bin/env python3
import rospy
import logging
import json
from icecream import ic  
from ros_ht_msg.msg import ht_control, Lift_control

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 基类，用于消息发布
class BaseCommand:
    """
    基础命令类，用于创建和发布ROS消息。
    参数:
    pub_topic (str): 发布消息的主题名称。
    msg_type: 消息的类型，需要与发布主题的消息类型匹配。
    属性:
    pub_topic (str): 发布消息的主题名称。
    pub: rospy.Publisher对象，用于发布消息。
    param: ReadParam对象，用于读取参数。
    """
    def __init__(self, pub_topic, msg_type):
        # 初始化ROS发布者和参数读取
        self.pub_topic = pub_topic
        self.pub = rospy.Publisher(self.pub_topic, msg_type, queue_size=100)
    def publish_command(self, command_func, *args, **kwargs):
        """
        发布命令消息。
        参数:
        command_func: 执行命令的函数。
        args: 位置参数，传递给command_func。
        kwargs: 关键字参数，传递给command_func。
        无返回值。
        """
        try:
            # 尝试执行命令函数，并发布消息
            ic(*args, **kwargs)  # ic函数用于验证命令执行
            msg = command_func(*args, **kwargs)
            ic(msg)
            self.pub.publish(msg)
            rospy.loginfo(f"指令发布成功: {str(msg)}")
        except Exception as e:
            # 如果有异常发生，则记录错误日志
            rospy.logerr(f"{str(self.pub_topic)}发布指令失败: {str(e)}")
    def sleep(self, seconds):
        rospy.sleep(seconds)
class BasePlateCommand(BaseCommand):
    """
    BasePlateCommand类负责控制底盘的运动。
    继承自BaseCommand类，初始化时建立与gt_control的连接，并开始移动底盘。
    """
    # def __init__(self):
    def __init__(self):
        super().__init__("/HT_Control", ht_control)  # 调用父类构造函数，初始化命令名称和控制对象

    def _base_plate_command(self, moveparam):
        """
        构造并返回一个控制底盘运动的命令。
        :param moveparam: 运动参数列表 ，包括模式和坐标。
        :return: 控制基板运动的命令对象。
        """
        control = ht_control()  # 创建控制对象
        control.mode = moveparam["运动模式"]  # 设置运动模式
        control.x = moveparam["x"]  # 设置x速度
        control.y = moveparam["y"]  # 设置y速度
        control.stop = moveparam["停止标志"]  # 设置停止标志
        return control

class LiftCommand(BaseCommand):
    def __init__(self):
        super().__init__("/Lift_Control", Lift_control)

    def _lift_command(self, moveparam):
        control = Lift_control()
        control.mode = moveparam["mode"]
        control.data = moveparam["data"]
        control.clear_flag = moveparam["clear_flag"]
        return control

def execute_commands(commands):
    base_plate = BasePlateCommand()
    lift = LiftCommand()

    for command_set in commands:
        for command_str in command_set:
            name, args_str = command_str.split("=")
            args = json.loads(args_str)

            if name == "Move":
                base_plate.publish_command(base_plate._base_plate_command, args)
            elif name == "Lift":
                lift.publish_command(lift._lift_command, args)
            base_plate.sleep(1)  # 添加延迟，确保命令执行

if __name__ == "__main__":
    rospy.init_node("command_node")
    commands = [
        # ['Move={"运动模式": 1, "x": 1, "y": 2, "停止标志": 0}', 'Lift={"mode": "down", "data": 2, "clear_flag": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Lift={"mode": 1 ,"data": 2, "clear_flag": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Move={"运动模式": 1, "x": 1, "y": 1, "停止标志": 0}'],
        ['Lift={"mode": "1", "data": 2, "clear_flag": 0}']
    ]
    execute_commands(commands)
