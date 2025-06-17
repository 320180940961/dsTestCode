#!/usr/bin/env python3
import curses
# curses installed on Linux, manually install on windows as following:
# pip install windows-curses
import rospy
from node import Naver, PathRecorder  # 修改为新的 PathRecorder
import datetime
from enum import Enum
import os
import threading
import time


# All stat can Jump to itself (Start a new command)
# All stat can quit() to force exit (@TODO autosave needed)

def gxyinfo(info):
    # cmds.append(info)
    pass

JUMP_TAB = {
    "RECORD": ["ADD", "INSERT", "DELETE","IDLE"],
    "ADD": ["ADD", "INSERT", "DELETE","IDLE"],
    "INSERT": ["INSERT", "ADD", "DELETE","IDLE"],
    "DELETE": ["DELETE", "ADD", "INSERT","IDLE"],
    "NAV": ["MVTO", "RECORD", "IDLE"],
    "MVTO": ["MVTO", "NAV", "IDLE"],
    "IDLE": ["RECORD", "NAV"],
}

# 导航器没有正确初始化​​：
# 在main.py中的MVTO命令处理分支，没有正确创建Naver实例
# ​​没有启动导航线程​​：
# 即使创建了导航器，也没有调用start_navigation()来启动导航
# ​​索引查找逻辑问题​​：
# 当导航到指定点时，self.points.index(target)方法可能无法找到目标点的索引（如果路径点中有重复的点）
# ​​没有更新nav_naver变量​​：
# 在main.py中没有为MVTO命令设置nav_naver变量，导致导航状态无法跟踪

CMD_TIPS = {
    "RECORD": "listname:string",
    "ADD": "comment:string(optional)",
    "INSERT": "former_point:int | comment:string(optional)",
    "DELETE": "at_point:int",
    "NAV": "direction:int listname:string" "path.json  (正序) 或 -path.json  (逆序)",
    "MVTO": "at_point:int",
    "IDLE": "",
}

class GT_STATUS(Enum):
    RECORD = "0"
    MOV = "1"
    NAV = "1-1"
    CMD = "1-2"
    IDLE = "2"
def auto_stat_doc():
    """
    deprecated, Only for  Hierarchical FSM
    """
    names = [i.name for i in GT_STATUS]
    res={i.name:[] for i in names}
    p,q = 0,1
    for i in range(len(GT_STATUS)-1):
        e1, e2 = GT_STATUS[names[p]], GT_STATUS[names[q]]
        if len(e1.value) == len(e2.value):
            pass



MAX_CMD_ROWS = 15

def get_logo(height, width):

    __logo = r"""  ___       ______      _           _     _____ _____       _____ _____      _____ _____ 
 / _ \      | ___ \    | |         | |   |_   _|_   _|     |  __ |_   _|    |  _  / __  \
/ /_\ \ __ _| |_/ /___ | |__   ___ | |_    | |   | |       | |  \/ | |______| |/' `' / /'
|  _  |/ _` |    // _ \| '_ \ / _ \| __|   | |   | |       | | __  | |______|  /| | / /  
| | | | (_| | |\ | (_) | |_) | (_) | |_   _| |_ _| |_      | |_\ \ | |      \ |_/ ./ /___
\_| |_/\__, \_| \_\___/|_.__/ \___/ \__|  \___/ \___/       \____/ \_/       \___/\_____/
        __/ |                                                                            
       |___/                                                                             """

    __logo_small = r"""    _        ___     _        _     ___ ___        ___ _____     __ ___ 
   /_\  __ _| _ \___| |__ ___| |_  |_ _|_ _|      / __|_   ____ /  |_  )
  / _ \/ _` |   / _ | '_ / _ |  _|  | | | |      | (_ | | ||___| () / / 
 /_/ \_\__, |_|_\___|_.__\___/\__| |___|___|      \___| |_|     \__/___|
       |___/                                                            """

    __logo_text = "AgRobot II  GT-02"
    
    for logo in [__logo, __logo_small, __logo_text]:
        w = max([len(i.strip()) for i in logo.split("\n")])
        h = len(logo.split("\n"))
        if width < w or height//2 < h:
            continue
        else:
            return logo


def setup(win):
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)
    
    win.keypad(False)
    curses.echo()
    
    curses.curs_set(1)
    

def frame_welcome(win,height, width):
    win.nodelay(False)
    __wel_text = "DSLab AgRobot Terminal for GT-02"
    __wel_text2 = "<Press Any to Continue, 'q' to quit>"
    k = None
    
    win.clear()
    win.refresh()
    
    start_y = int((height // 2))
    start_x_wel_text= int((width // 2) - (len(__wel_text) // 2) - len(__wel_text) % 2)
    start_x_wel_text2= int((width // 2) - (len(__wel_text2) // 2) - len(__wel_text2) % 2)

    logo=get_logo(height, width)

    win.attron(curses.color_pair(1))
    for i,_l in enumerate(logo.split("\n")[::-1]):
        start_x_l= int((width // 2) - (len(_l) // 2) - len(_l) % 2)
        win.addstr(start_y - i - 1, start_x_l,_l)
        
    win.attroff(curses.color_pair(1))
    
    
    win.attron(curses.color_pair(2))
    win.attron(curses.A_BOLD)

    # Rendering title
    win.addstr(start_y + 1, start_x_wel_text, __wel_text)

    # Turning off attributes for title
    win.attroff(curses.color_pair(2))
    win.attroff(curses.A_BOLD)
    
    win.addstr(start_y + 3, start_x_wel_text2, __wel_text2)
    
    win.refresh()
    k = win.getch()
    if k == ord("q"):
        exit()
    win.clear()
    win.refresh()
    win.nodelay(True)

def main_loop(win, height, width):
    input_head = ' admin@GT-02 > '
    info_head = '│ Tips: '
    
    max_cmd_rows = MAX_CMD_ROWS if MAX_CMD_ROWS < height - 3 else height - 3
    
    y_split = height - 3
    y_info = height - 2
    y_input = height - 1
    cmds = ["历史日志"]
    
    # 设置路径存储目录
    base_dir = os.path.dirname(os.path.abspath(__file__)) 
    json_dir = os.path.join(base_dir, "paths_json")
    
    # 确保目录存在
    if not os.path.exists(json_dir): 
        os.makedirs(json_dir) 
        cmds.append(f" 创建路径存储目录: {json_dir}")
    
    exit_flag = False
    cur_stat = "IDLE"
    raw_input = ""
    warn = None
    
    # 记录器和导航器实例
    recorder_naver = None  # 记录模式
    nav_naver = None       # 导航模式
    
    # 添加时间跟踪变量
    last_gps_print_time = time.time()
    
    # ROS兼容性的spin_once实现
    def ros_spin_once():
        """兼容不同ROS版本的回调处理方法"""
        try:
            if hasattr(rospy, 'spin_once'):
                rospy.spin_once(timeout=0.1)
            else:
                # 旧版ROS使用替代方法
                start = rospy.get_time()
                while (rospy.get_time() - start) < 0.1 and not rospy.is_shutdown():
                    pass
        except:
            pass
    
    while not exit_flag:
        # 处理ROS回调
        ros_spin_once()
        
        # 更新终端大小
        height, width = win.getmaxyx()
        
        # 限制日志行数
        if len(cmds) > max_cmd_rows * 10:  # 保留足够的历史
            cmds = cmds[-max_cmd_rows:]
        
        # 状态信息
        info_body = "当前状态: {}, 可用命令: {} ".format(cur_stat, "".join(["<%s>  "%i for i in JUMP_TAB[cur_stat]]))
        
        # 1. 显示控制台日志
        # 只显示最近的 max_cmd_rows 行
        start_index = max(0, len(cmds) - max_cmd_rows)
        displayed_logs = cmds[start_index:]
        
        win.clear()
        for i, cmd in enumerate(displayed_logs):
            if i < height - 4:  # 确保不会超出屏幕边界
                win.addstr(i, 0, cmd)
        
        # 2. 添加状态信息区域
        win.addstr(y_split, 0, "┌" + '─'*(width-2))
        
        # 显示GPS信息（如果在记录或导航模式下）
        if recorder_naver or nav_naver:
            current_time = time.time()
            if current_time - last_gps_print_time > 1.0:  # 每秒更新一次
                try:
                    # 添加通用的gps_available检查
                    gps_available = False
                    position_info = None
                    
                    if recorder_naver and recorder_naver.current_gps:
                        p = recorder_naver.current_position
                        position_info = f"位置:({p['lat']:.6f}, {p['lng']:.6f})"
                        gps_available = True
                    elif nav_naver and nav_naver.current_gps:
                        p = nav_naver.current_position
                        position_info = f"位置:({p['lat']:.6f}, {p['lng']:.6f})"
                        gps_available = True
                    
                    if position_info:
                        gps_info = f"  GPS:{position_info}"
                    else:
                        gps_info = "  等待GPS数据..."
                    
                    win.addstr(height-4, 0, gps_info + " "*(width-len(gps_info)-1))
                    last_gps_print_time = current_time
                except Exception as e:
                    # 添加错误日志
                    win.addstr(height-4, 0, f"GPS显示错误: {str(e)[:30]}")
        
        # 显示提示信息或警告
        if warn:
            win.attron(curses.color_pair(2))
            win.addstr(y_info, 0, warn + " "*(width - len(warn) - 1))
            win.attroff(curses.color_pair(2))
        else:
            win.addstr(y_info, 0, info_head + info_body)
        
        # 输入区域
        win.attron(curses.color_pair(4))
        win.addstr(y_input, 0, input_head)
        win.attroff(curses.color_pair(4))
        win.addstr(y_input, len(input_head), raw_input)
        
        win.refresh()
        
        # 3. 输入处理
        try:
            win.timeout(100)  # 100ms超时，避免阻塞
            ch = win.getch()
            if ch == -1:  # 没有按键
                continue
        except:
            continue
        
        # 处理特殊按键
        if ch == curses.KEY_BACKSPACE or ch == 127:
            raw_input = raw_input[:-1]
            continue
        
        # 按Enter键
        if ch == 10:  # Enter键
            pass
        else:
            # 添加字符到输入
            try:
                raw_input += chr(ch)
            except:
                pass
            continue
        
        # 处理命令
        raw_input = raw_input.strip()
        warn = None
        
        # 退出命令
        if raw_input.find("quit()") >= 0:
            exit_flag = True
            cmds.append("正在退出程序...")
            
            # 保存记录点
            if recorder_naver and recorder_naver.modified:
                recorder_naver.save_points()
                cmds.append(f"保存路径 '{recorder_naver.recorder.listname}' ({len(recorder_naver.points)}点)")
            
            # 停止导航线程
            if nav_naver:
                if nav_naver.is_alive():
                    nav_naver.stop_navigation()
                    cmds.append("停止导航线程...")
                    nav_naver.join(timeout=1.0)
                nav_naver = None
            
            win.nodelay(False)  # 恢复阻塞模式以便显示退出消息
            win.getch()  # 等待任意键
            continue
        
        args = raw_input.split()
        raw_input = ""
        
        if len(args) > 1:
            chstat, args = args[0].upper(), args[1:]
        elif len(args) == 1:
            chstat, args = args[0].upper(), None
        else:
            continue
        
        if chstat not in JUMP_TAB.keys():
            warn = f"未知命令: {chstat}"
            continue
        elif chstat not in JUMP_TAB[cur_stat] and chstat != cur_stat:
            warn = f"状态 {cur_stat} 无法转换为 {chstat}"
            continue
        
        # RECORD命令 - 创建记录模式的Naver
        elif chstat == "RECORD":
            if not args:
                warn = "RECORD命令需要路径名称"
                continue
            listname = args[0]
            json_path = os.path.join(json_dir, f"{listname}.json")
            
            # 保存之前的路径
            if recorder_naver:
                if recorder_naver.modified:
                    recorder_naver.save_points()
                    cmds.append(f"保存之前路径 '{recorder_naver.recorder.listname}'")
                recorder_naver = None
            
            cmds.append(f"[RECORD] 启动记录模式: '{listname}'")
            
            try:
                # 创建记录器
                recorder = PathRecorder(listname)
                
                # 创建记录模式Naver
                recorder_naver = Naver(recorder, mode="record")
                
                # 等待GPS数据 - 使用current_gps替代gps_ready
                start_time = time.time()
                gps_timeout = 5.0  # 5秒超时
                gps_available = False
                
                while not gps_available and (time.time() - start_time) < gps_timeout:
                    if recorder_naver.current_gps is not None:
                        gps_available = True
                    else:
                        elapsed = int(time.time() - start_time)
                        cmds.append(f"[GPS] 等待GPS数据...{elapsed}秒")
                        win.refresh()
                        time.sleep(0.5)
                
                if recorder_naver.current_gps is not None:
                    cmds.append("[GPS] GPS数据接收成功")
                else:
                    warn = "[WARN] GPS数据接收超时，使用模拟点"
                    cmds.append(warn)
                
                cur_stat = "RECORD"
                cmds.append(f"准备记录路径 '{listname}'，输入ADD添加点")
                
            except Exception as e:
                warn = f"RECORD启动失败: {str(e)}"
                recorder_naver = None
                cmds.append(f"错误: {str(e)}")
        
        # ADD命令 - 添加点
        elif chstat == "ADD":
            if not recorder_naver or recorder_naver.mode != "record":
                warn = "不在记录模式，请先使用RECORD命令"
                continue
                
            comment = " ".join(args) if args else ""
            try:
                points = recorder_naver.add_point(comment)
                point = recorder_naver.points[-1]
                cmds.append(f"[ADD] 添加点#{len(points)-1}: ({point['lat']:.6f}, {point['lng']:.6f})")
                if comment:
                    cmds.append(f"      备注: {comment}")
            except Exception as e:
                warn = f"添加点失败: {str(e)}"
        
        # INSERT命令 - 插入点
        elif chstat == "INSERT":
            if not recorder_naver or recorder_naver.mode != "record":
                warn = "不在记录模式"
                continue
                
            if not args or not args[0].isdigit():
                warn = "INSERT需要索引号"
                continue
                
            index = int(args[0])
            comment = " ".join(args[1:]) if len(args) > 1 else ""
            
            try:
                points = recorder_naver.insert_point(index, comment)
                cmds.append(f"[INSERT] 在位置#{index}插入点成功 (总点数: {len(points)})")
                point = recorder_naver.points[index]
                cmds.append(f"        位置: ({point['lat']:.6f}, {point['lng']:.6f})")
                if comment:
                    cmds.append(f"        备注: {comment}")
            except Exception as e:
                warn = f"插入点失败: {str(e)}"
        
        # DELETE命令 - 删除点
        elif chstat == "DELETE":
            if not recorder_naver or recorder_naver.mode != "record":
                warn = "不在记录模式"
                continue
                
            if not args or not args[0].isdigit():
                warn = "DELETE需要索引号"
                continue
                
            index = int(args[0])
            try:
                result = recorder_naver.delete_point(index)
                cmds.append(f"[DELETE] 删除点#{index}成功 (剩余: {result}个)")
            except Exception as e:
                warn = f"删除点失败: {str(e)}"
        
        # NAV命令 - 启动导航
        elif chstat == "NAV":
            if not args:
                warn = "NAV需要路径文件名 (如: nav path1 或 nav -path1)"
                continue 
                
            filename = args[0]
            
            # 判断是否逆序 
            is_reverse = filename.startswith('-') 
            listname = filename[1:] if is_reverse else filename 
            
            # 构建完整路径 
            json_path = os.path.join(json_dir, f"{listname}.json")
            
            # 检查文件是否存在 
            if not os.path.exists(json_path): 
                # 获取所有可用路径文件
                available = [f[:-5] for f in os.listdir(json_dir) if f.endswith('.json')] 
                warn = f"路径文件 '{listname}' 不存在！\n可用路径: {', '.join(available) or '无'}"
                continue 
            
            # 停止之前的导航
            if nav_naver:
                nav_naver.stop_navigation()
                nav_naver = None
                cmds.append("[STOP] 停止之前的导航")
            
            try:
                # 设置导航模式 
                dir_mode = "reverse" if is_reverse else "forward"
                
                # 创建导航器
                recorder = PathRecorder(listname)
                nav_naver = Naver(recorder, mode="navigation", dir_mode=dir_mode)
                
                # 等待GPS准备 - 使用current_gps检查
                start_time = time.time()
                gps_timeout = 3.0  # 3秒超时
                gps_available = False
                
                while not gps_available and (time.time() - start_time) < gps_timeout:
                    if nav_naver.current_gps is not None:
                        gps_available = True
                    else:
                        cmds.append(f"[GPS] 准备GPS导航数据...{int(time.time() - start_time)}秒")
                        win.refresh()
                        time.sleep(0.5)
                
                # 启动导航
                if nav_naver.start_navigation():
                    direction = "逆序回航" if is_reverse else "正序巡航"
                    cmds.append(f"[NAV] 启动{direction}: {listname}")
                    cmds.append(f"[PATH] 文件路径: {json_path}")
                    cur_stat = "NAV"
                else:
                    warn = "导航启动失败"
                    nav_naver = None
            except Exception as e:
                warn = f"导航启动失败: {str(e)}"
                nav_naver = None
                cmds.append(f"详细错误: {str(e)}")
        
        # MVTO命令 - 导航到指定点
        elif chstat == "MVTO":
            # 需要有一个记录的路径
            if not recorder_naver or recorder_naver.mode != "record":
                warn = "请先创建记录路径(使用RECORD)"
                continue
                
            if not args or not args[0].isdigit():
                warn = "MVTO需要目标点索引号"
                continue
                
            target_index = int(args[0])
            cmds.append(f"[MVTO] 准备导航到点#{target_index}")
            
            # 停止之前的导航
            if nav_naver:
                nav_naver.stop_navigation()
                nav_naver = None
                cmds.append("[STOP] 停止之前的导航")
                
            try:
                # 创建导航器，指定目标点
                recorder = PathRecorder(recorder_naver.recorder.listname)
                nav_naver = Naver(recorder, mode="navigation", mvto_point=target_index)
                
                # 等待GPS准备 - 使用current_gps检查
                start_time = time.time()
                gps_timeout = 3.0  # 3秒超时
                gps_available = False
                
                while not gps_available and (time.time() - start_time) < gps_timeout:
                    if nav_naver.current_gps is not None:
                        gps_available = True
                    else:
                        cmds.append(f"[GPS] 准备GPS导航数据...{int(time.time() - start_time)}秒")
                        win.refresh()
                        time.sleep(0.5)
                
                # 启动导航
                if nav_naver.start_navigation():
                    cmds.append(f"[MVTO] 启动导航到点#{target_index}")
                    cur_stat = "MVTO"
                else:
                    warn = "MVTO启动失败"
                    nav_naver = None
            except Exception as e:
                warn = f"MVTO启动失败: {str(e)}"
                nav_naver = None
                cmds.append(f"详细错误: {str(e)}")
        
        # IDLE命令 - 返回空闲状态
        elif chstat == "IDLE":
            # 保存记录数据
            if recorder_naver:
                # 检查是否有未保存的点
                if recorder_naver.modified:
                    recorder_naver.save_points()
                    cmds.append(f"[SAVE] 路径 '{recorder_naver.recorder.listname}' 已保存 ({len(recorder_naver.points)}点)")
                recorder_naver = None
                cmds.append("[IDLE] 退出记录模式")
            
            # 停止导航线程
            if nav_naver:
                if nav_naver.is_alive():
                    nav_naver.stop_navigation()
                    cmds.append("[STOP] 正在停止导航线程...")
                else:
                    cmds.append("[IDLE] 导航线程已完成")
                nav_naver = None
            else:
                cmds.append("[IDLE] 当前没有运行中的导航")
            
            cur_stat = "IDLE"
        
        # 更新当前状态
        cur_stat = chstat
        
        win.refresh()
        win.clear()
        

def main(win):
    rospy.init_node("agrobot_return_node", anonymous=True)
    setup(win)
    
    height, width = win.getmaxyx()
    
    frame_welcome(win,height, width)

    main_loop(win,height, width)
    
curses.wrapper(main)