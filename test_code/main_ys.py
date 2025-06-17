#!/usr/bin/env python3
import curses
# curses installed on Liunx, manually install on windows as following:
# pip install windows-curses
import rospy
from node import Naver,PathRecorder
import datetime
from enum import Enum
import os


# All stat can Jump to itself (Start a new command)
# All stat can quit() to force exit (@TODO autosave needed)

JUMP_TAB = {
    "RECORD": ["ADD", "INSERT", "DELETE","IDLE"],
    "ADD": ["ADD", "INSERT", "DELETE","IDLE"],
    "INSERT": ["INSERT", "ADD", "DELETE","IDLE"],
    "DELETE": ["DELETE", "ADD", "INSERT","IDLE"],
    "NAV": ["MVTO", "RECORD", "IDLE"],
    "MVTO": ["MVTO", "NAV", "IDLE"],
    "IDLE": ["RECORD", "NAV"],
}

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
        # print(f"Logo width {w} height {h}\nScreen width {width} half height {height//2}")
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
    # curses.nocbreak()
    
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



def main_loop(win,height, width):
    
    
    input_head = ' admin@GT-02 > '
    info_head = '│ Tips: '
    
    max_cmd_rows = MAX_CMD_ROWS if MAX_CMD_ROWS < height-3 else height -3
    
    y_split = height - 3
    y_info = height - 2
    y_input = height - 1
    cmds = ["History Logs"]
    # 设置paths_json文件夹路径（与代码同级）
    base_dir = os.path.dirname(os.path.abspath(__file__)) 
    json_dir = os.path.join(base_dir,  "paths_json")
    
    # 确保paths_json文件夹存在 
    if not os.path.exists(json_dir): 
        os.makedirs(json_dir) 
        cmds.append(f" 创建路径存储目录: {json_dir}")
    exit_flag = False
    
    cur_stat = "IDLE"
    
    raw_input = ""
    
    warn = None
    

    puber = None
    
    while not exit_flag:
        
        height, width = win.getmaxyx()
        
        info_body = "Current {}, Leagal Command: {} ".format(cur_stat, "".join(["<%s:%s>  "%(i,CMD_TIPS[i]) for i in JUMP_TAB[cur_stat]]))
        if puber: 
            cmds = puber.log[-max_cmd_rows:]
            cmds.append(f"[STATUS] 当前状态：{puber.status}")
            if puber.current_target_index is not None:
                cmds.append(f"[TARGET] 正在前往目标点编号：{puber.current_target_index}")
        if puber and puber.current_position and puber.current_target:
            p = puber.current_position
            t = puber.current_target
            cmds.append(f"[GPS] 当前：({p['lat']:.6f}, {p['lng']:.6f}) → 目标：({t['lat']:.6f}, {t['lng']:.6f})")

        for i,cmd in enumerate(cmds):
            win.addstr(i, 0, cmd)
            
    
        win.addstr(y_split, 0, "┌" + '─'*(width-2))
        if not warn:
            win.addstr(y_info, 0,info_head + info_body)
        else:
            win.attron(curses.color_pair(2))
            win.addstr(y_info, 0, warn)
            win.attroff(curses.color_pair(2))
            
        win.attron(curses.color_pair(4))
        win.addstr(y_input, 0, input_head)
        win.attroff(curses.color_pair(4))
        win.addstr(y_input, len(input_head), raw_input)
        
        
        win.refresh()

        ch = win.getch()
        if ch == -1:
            continue
        
        ch = curses.keyname(ch).decode("utf-8")
        win.clear()
        
        # Backspace
        if ch in ["H","^?"]:
            raw_input=raw_input[:-1]
            continue
        # Enter
        elif ch != "^J":
            raw_input += ch
            continue
        
        # Handling raw_input
        
        raw_input = raw_input.strip()
        warn = None
        
        if raw_input.find("quit()") >=0:
            exit_flag=True
            if puber:
                puber.stop.set()
            
        args = raw_input.split()
        
        raw_input = ""
        
        if len(args)>1:
            chstat, args = args[0].upper(), args[1:]
        elif len(args)==1:
            chstat, args = args[0].upper(), None
        else:continue
        
        if chstat not in JUMP_TAB.keys():
            continue
        elif chstat not in JUMP_TAB[cur_stat] and chstat != cur_stat:
            warn = f"{cur_stat} can not change into {chstat}"
            continue
        
        elif chstat == "RECORD":
            if not args:
                warn = f"CMD {chstat} need listname"
                continue
            listname = args[0]
            json_path = os.path.join(json_dir,f"{listname}.json")

            if os.path.exists(json_path):
                warn = f"路径文件'{listname}.josn'已经存在，请重新输入"
                continue
            else:
                recorder = PathRecorder(args[0])
                recorder._save()
                cmds.append(f"[RECORD] Path list '{args[0]}' loaded with {len(recorder.points)} points.")
                cmds.append(f"[RECORD]新建文件'{listname}.josn'成功")

        elif chstat == "ADD":
            if recorder is None:
                warn = "RECORD mode not initialized. Use RECORD <name> first."
                continue
            comment = " ".join(args) if args else ""
            recorder.add_point(comment)
            cmds.append(f"[ADD] Point added. Total: {len(recorder.points)}")
        elif chstat == "INSERT":
            if recorder is None:
                    warn = "RECORD mode not initialized."
                    continue
            if not args or not args[0].isdigit():
                warn = "INSERT needs index:int and optional comment"
                continue
            index = int(args[0])
            comment = " ".join(args[1:]) if len(args) > 1 else ""
            recorder.insert_point(index, comment)
            cmds.append(f"[INSERT] at {index}. Total: {len(recorder.points)}")
        elif chstat == "DELETE":
            if recorder is None:
                warn = "RECORD mode not initialized."
                continue
            if not args or not args[0].isdigit():
                warn = "DELETE needs index:int"
                continue
            index = int(args[0])
            recorder.delete_point(index)
            cmds.append(f"[DELETE] Removed index {index}. Total: {len(recorder.points)}")

        elif chstat == "NAV":
            if not args:
                warn = "NAV 需要路径文件名 (如: nav path1 或 nav -path1)"
                continue 
                
            filename = args[0]
            
            # 判断是否逆序 
            is_reverse = filename.startswith('-') 
            listname = filename[1:] if is_reverse else filename 
            
            # 构建完整路径 
            json_path = os.path.join(json_dir,  f"{listname}.json")
            
            # 检查文件是否存在 
            if not os.path.exists(json_path): 
                # 获取所有可用路径文件（显示时不带路径和.json后缀）
                available = [f[:-5] for f in os.listdir(json_dir)  if f.endswith('.json')] 
                warn = f"路径文件 '{listname}' 不存在！\n可用路径: {', '.join(available) or '无'}"
                continue 
                
            try:
                # 设置导航模式 
                dir_mode = "reverse" if is_reverse else "forward"
                puber = Naver(dir_mode, listname)
                puber.start() 
                
                # 显示导航信息 
                direction = "逆序回航" if is_reverse else "正序巡航"
                cmds.append(f"[NAV]  启动{direction}: {listname}")
                cmds.append(f"[PATH]  文件路径: {json_path}")
                
            except Exception as e:
                warn = f"导航启动失败: {str(e)}"

        elif chstat == "IDLE":
            if puber:
                if puber.is_alive():
                    puber.stop.set()  # 通知线程停止
                    cmds.append("[IDLE] 已发送停止导航信号")
                else:
                    cmds.append("[IDLE] 导航线程已完成，无需停止")
            else:
                cmds.append("[IDLE] 当前没有运行中的导航线程")

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
