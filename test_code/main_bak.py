#!/usr/bin/env python3
import curses
import rospy
from node_bak import Naver, PathRecorder
import os
import threading

JUMP_TAB = {
    "RECORD": ["ADD", "INSERT", "DELETE", "IDLE"], "ADD": ["ADD", "INSERT", "DELETE", "IDLE"],
    "INSERT": ["INSERT", "ADD", "DELETE", "IDLE"], "DELETE": ["DELETE", "ADD", "INSERT", "IDLE"],
    "NAV": ["MVTO", "RECORD", "IDLE"], "MVTO": ["MVTO", "NAV", "IDLE"], "IDLE": ["RECORD", "NAV"],
}
CMD_TIPS = {
    "RECORD": "listname:string", "ADD": "comment:string(optional)",
    "INSERT": "former_point:int | comment:string(optional)", "DELETE": "at_point:int",
    "NAV": "listname (or -listname for reverse)", "MVTO": "at_point:int", "IDLE": "",
}
MAX_CMD_ROWS = 25

def get_logo(height, width):
    __logo = r"""  ___           ______        _           _     _____ _____         _____ _____       _____ _____ 
 / _ \          | ___ \      | |         | |   |_   _|_   _|       |  __ |_   _|     |  _  / __  \
/ /_\ \ __ _  _ | |_/ /___ | |__   ___ | |_    | |    | |         | |  \/ | |______| |/' `' / /'
|  _  |/ _` || ||  __/ __ |  _ \ / _ \ __| |   | |    | |         | | __  | |______|  /| | / /  
| | | | (_| || || | | (__ | | | |  __/ |_  _| |_ _| |_        | |_\ \ | |        \ |_/ ./ /___
\_| |_/\__,_||_||_|  \___||_| |_|\___|\__| \___/ \___/         \____/ \_/         \___/\_____/
         __/ |                                                                               
        |___/                                                                                """
    __logo_small = r"""    _        ___      _         _     ___ ___          ___ _____       __ ___ 
   /_\  __ _| _ \___| |__ ___| |_   |_ _|_ _|        / __|_   ____ /  |_  )
 / _ \/ _` |   / _ \ '_ / _ \  _|   | | | |         | (_ | | ||___| () / / 
/_/ \_\__, |_|_\___|_.__\___/\__|  |___|___|         \___| |_|      \__/___|
        |___/                                                              """
    __logo_text = "AgRobot II  GT-02"
    for logo in [__logo, __logo_small, __logo_text]:
        logo_lines = logo.split("\n")
        w = max([len(i.rstrip()) for i in logo_lines])
        h = len(logo_lines)
        if width < w or height // 2 < h: continue
        else: return logo
    return __logo_text

def setup(win):
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)
    win.keypad(True) # 开启keypad模式以更好处理特殊键
    curses.echo()
    curses.curs_set(1)

def frame_welcome(win, height, width):
    win.nodelay(False)
    __wel_text = "DSLab AgRobot Terminal for GT-02"
    __wel_text2 = "<Press Any Key to Continue, 'q' to quit>"
    win.clear()
    start_y = height // 2
    start_x_wel_text = (width - len(__wel_text)) // 2
    start_x_wel_text2 = (width - len(__wel_text2)) // 2
    logo = get_logo(height, width)
    logo_lines = logo.split("\n")
    win.attron(curses.color_pair(1))
    for i, line in enumerate(logo_lines):
        start_x_l = (width - len(line)) // 2
        win.addstr(start_y - len(logo_lines) + i, start_x_l, line)
    win.attroff(curses.color_pair(1))
    win.attron(curses.color_pair(2) | curses.A_BOLD)
    win.addstr(start_y + 1, start_x_wel_text, __wel_text)
    win.attroff(curses.color_pair(2) | curses.A_BOLD)
    win.addstr(start_y + 3, start_x_wel_text2, __wel_text2)
    win.refresh()
    k = win.getch()
    if k == ord('q'): exit(0)
    win.clear()
    win.refresh()
    win.nodelay(True)

def main_loop(win, height, width):
    input_head = ' admin@GT-02 > '
    info_head = '│ Tips: '
    
    cmds = ["Welcome to AgRobot Terminal. Enter 'quit()' to exit."]
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    json_dir = os.path.join(base_dir, "paths_json")
    if not os.path.exists(json_dir):
        os.makedirs(json_dir)
        cmds.append(f"[SYSTEM] 创建路径存储目录: {json_dir}")

    exit_flag = False
    cur_stat = "IDLE"
    raw_input = ""
    warn = None
    
    recorder_naver = None
    nav_naver = None
    
    last_log_index = 0
    redraw_flag = True

    while not exit_flag:
        if redraw_flag:
            height, width = win.getmaxyx()
            max_cmd_rows = height - 4
            y_split, y_info, y_input = height - 3, height - 2, height - 1
            
            win.clear() # 使用clear()而不是逐行清理，对于全屏重绘更简单可靠
            
            start_index = max(0, len(cmds) - max_cmd_rows)
            for i, cmd in enumerate(cmds[start_index:]):
                win.addstr(i, 0, str(cmd)[:width-1])

            win.addstr(y_split, 0, "┌" + '─'*(width-2))
            
            if warn:
                win.attron(curses.color_pair(2))
                win.addstr(y_info, 0, ("│ " + warn)[:width-1])
                win.attroff(curses.color_pair(2))
                warn = None 
            else:
                info_body = f"Current: {cur_stat}, Legal: {' '.join([f'<{k}>' for k in JUMP_TAB[cur_stat]])}"
                win.addstr(y_info, 0, (info_head + info_body)[:width-1])
            
            win.attron(curses.color_pair(4))
            win.addstr(y_input, 0, input_head)
            win.attroff(curses.color_pair(4))
            win.addstr(y_input, len(input_head), raw_input)
            
            win.refresh()
            redraw_flag = False

        if nav_naver and last_log_index < len(nav_naver.log):
            new_logs = nav_naver.log[last_log_index:]
            cmds.extend(new_logs)
            last_log_index = len(nav_naver.log)
            redraw_flag = True

        try:
            ch = win.getch()
            if ch == -1: continue
        except curses.error:
            continue
        
        # 【修改点】: 统一处理回车键(Enter)和退格键(Backspace)
        if ch in (curses.KEY_ENTER, 10, 13): # 10是\n, 13是\r
            input_command = raw_input.strip()
            raw_input = ""
            if not input_command: continue
            
            cmds.append(f"{input_head}{input_command}")
            args = input_command.split()
            chstat, params = args[0].upper(), args[1:]
            
            redraw_flag = True

            if chstat == "QUIT()":
                if recorder_naver: recorder_naver.save_points()
                if nav_naver: nav_naver.stop_navigation()
                exit_flag = True
                continue

            if chstat not in JUMP_TAB.get(cur_stat, []):
                warn = f"Error: Command '{chstat}' not allowed from state '{cur_stat}'."
                continue
            
            # --- 命令逻辑 ---
            # (此处的命令处理逻辑保持不变，为简洁省略)
            cur_stat = chstat
            
            if chstat == "RECORD":
                if not params: warn = "Usage: RECORD <listname>"; continue
                listname = params[0]
                if recorder_naver:
                    recorder_naver.save_points()
                    cmds.append(f"[SAVE] 已保存 '{recorder_naver.recorder.listname}'")
                recorder = PathRecorder(listname)
                recorder_naver = Naver(recorder, mode="record")
                cmds.extend(recorder_naver.log)
                last_log_index = len(cmds)

            elif chstat in ("ADD", "INSERT", "DELETE"):
                if not recorder_naver: warn = "Not in RECORD mode"; continue
                comment = " ".join(args) if args else ""
                if chstat == "ADD":
                    points = recorder_naver.add_point(comment)
                    point = recorder_naver.points[-1]
                    recorder_naver.add_point(" ".join(params))
                    cmds.append(f"[ADD] 添加点#{len(points)-1}: ({point['lat']:.6f}, {point['lng']:.6f})")
                elif chstat == "INSERT":
                    if not params or not params[0].isdigit(): warn = "Usage: INSERT <index> [comment]"; continue
                    recorder_naver.insert_point(int(params[0]), " ".join(params[1:]))
                elif chstat == "DELETE":
                    if not params or not params[0].isdigit(): warn = "Usage: DELETE <index>"; continue
                    recorder_naver.delete_point(int(params[0]))
                # 日志会自动轮询，无需手动添加

            elif chstat in ("NAV", "MVTO"):
                listname, mvto_index, is_reverse = None, None, False
                if chstat == "NAV":
                    if not params: warn = "Usage: NAV <listname> or -<listname>"; continue
                    filename = params[0]
                    is_reverse = filename.startswith('-')
                    listname = filename[1:] if is_reverse else filename
                else: # MVTO
                    if not nav_naver: warn = "Not in NAV mode. Use NAV first."; continue
                    if not params or not params[0].isdigit(): warn = "Usage: MVTO <point_index>"; continue
                    mvto_index = int(params[0])
                    listname = nav_naver.recorder.listname
                    if not (0 <= mvto_index < len(nav_naver.points)):
                        warn = f"Index {mvto_index} out of range"; continue
                
                json_path = os.path.join(json_dir, f"{listname}.json")
                if not os.path.exists(json_path):
                    warn = f"Path file '{listname}' not found!"; continue

                if nav_naver: nav_naver.stop_navigation()
                
                dir_mode = "reverse" if chstat == "NAV" and is_reverse else "forward"
                recorder = PathRecorder(listname)
                nav_naver = Naver(recorder, mode="navigation", dir_mode=dir_mode, mvto_point=mvto_index)
                
                nav_naver._prepare_navigation(mvto_point=mvto_index)
                nav_naver.start_navigation()
                
                last_log_index = 0
                cmds.extend(nav_naver.log)
                last_log_index = len(cmds)

            elif chstat == "IDLE":
                if recorder_naver:
                    recorder_naver.save_points()
                    cmds.append(f"[SAVE] 路径 '{recorder_naver.recorder.listname}' 已保存")
                    recorder_naver = None
                if nav_naver:
                    nav_naver.stop_navigation()
                    cmds.extend(nav_naver.log[last_log_index:])
                    nav_naver = None
                cmds.append("[IDLE] 进入闲置状态")
                last_log_index = 0

        elif ch in (curses.KEY_BACKSPACE, 127): # 127是Backspace的ASCII码
             raw_input = raw_input[:-1]
             redraw_flag = True
        elif ch <= 255 and chr(ch).isprintable(): #只接受可打印字符
            raw_input += chr(ch)
            redraw_flag = True

def main(win):
    """主函数，包裹curses应用"""
    rospy.init_node("agrobot_terminal_node", anonymous=True)
    setup(win)
    height, width = win.getmaxyx()
    try:
        frame_welcome(win, height, width)
        main_loop(win, height, width)
    except curses.error as e:
        pass

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        print(f"An error occurred: {e}")