#!/usr/bin/env python3
import curses
import datetime
from enum import Enum
from node_new1 import Naver, PathRecorder

# 最大命令行显示行数：定义从某个状态可跳转到哪些命令
MAX_CMD_ROWS = 15

# 状态跳转表
JUMP_TAB = {
    'RECORD': ['ADD', 'INSERT', 'DELETE'],
    'ADD': ['ADD', 'INSERT', 'DELETE'],
    'INSERT': ['INSERT', 'ADD', 'DELETE'],
    'DELETE': ['DELETE', 'ADD', 'INSERT'],
    'NAV': ['MVTO', 'RECORD', 'IDLE'],
    'MVTO': ['MVTO', 'NAV', 'IDLE'],
    'IDLE': ['RECORD', 'NAV', 'CAPTURE'],
    'CAPTURE': ['CAPTURE', 'IDLE'],
    'AUTO': ['IDLE']
}

# 命令提示：每个命令的参数格式说明
CMD_TIPS = {
    'RECORD': 'listname:string',
    'ADD': 'comment:string(optional)',
    'INSERT': 'former_point:int | comment:string(optional)',
    'DELETE': 'at_point:int',
    'NAV': 'direction:string listname:string',
    'MVTO': 'at_point:int',
    'IDLE': '',
    'CAPTURE': '交互式记录路径点',
    'AUTO': 'listname:string 自动导航'
}

# 命令配置字典：统一定义 handler、调用方法、必选/可选参数，以及日志模板
COMMANDS = {
    'RECORD': {
        'handler': PathRecorder.get_instance,   # 获取或创建 PathRecorder 单例
        'method': 'load',                       # 调用 load 方法加载路径
        'required': ['-listname'],              # 必需参数
        'optional': [],
        'log': "[RECORD] Path '{listname}' loaded, total {count}."
    },
    'ADD': {
        'handler': PathRecorder.get_instance,
        'method': 'add_point',
        'required': [],
        'optional': ['--comment'],              # 可选参数
        'log': "[ADD] Point added, total {count}."
    },
    'INSERT': {
        'handler': PathRecorder.get_instance,
        'method': 'insert_point',
        'required': ['-index'],
        'optional': ['--comment'],
        'log': "[INSERT] Inserted at {index}, total {count}."
    },
    'DELETE': {
        'handler': PathRecorder.get_instance,
        'method': 'delete_point',
        'required': ['-index'],
        'optional': [],
        'log': "[DELETE] Removed {index}, total {count}."
    },
    'NAV': {
        'handler': Naver.get_instance,
        'method': 'start_nav',
        'required': ['-dir', '-listname'],
        'optional': [],
        'log': "[NAV] Start {dir} on '{listname}'."
    },
    'MVTO': {
        'handler': Naver.get_instance,
        'method': 'mvto',
        'required': ['-at_point'],
        'optional': [],
        'log': "[MVTO] Move to {at_point}."
    },
    'IDLE': {
        'handler': Naver.get_instance,
        'method': 'stop_nav',
        'required': [],
        'optional': [],
        'log': "[IDLE] Stop signal sent."
    },
    'CAPTURE': {
        'handler': PathRecorder.get_instance,
        'method': 'capture_points',
        'required': [],
        'optional': ['--name'],
        'log': "[CAPTURE] Capturing to '{name}'."
    },
    'AUTO': {
        'handler': Naver.get_instance,
        'method': 'auto_nav',
        'required': ['-listname'],
        'optional': [],
        'log': "[AUTO] Auto-nav '{listname}'."
    }
}

def parse_args(args, config):
    """
    解析命令行参数：
    - 校验并提取必选参数(config['required'])
    - 查找并提取可选参数(config['optional'])
    - 返回字典 parsed，key 为参数名(去掉前缀 '-')
    """
    parsed = {}
    # 检查必选参数是否存在
    for req in config['required']:
        if req not in args:
            raise ValueError(f"Missing required {req}")
        idx = args.index(req)
        parsed[req.strip('-')] = args[idx+1]
    # 查找可选参数
    for opt in config['optional']:
        if opt in args:
            idx = args.index(opt)
            parsed[opt.strip('-')] = args[idx+1]
    return parsed

def get_logo(height, width):
    """
    根据当前终端尺寸，选择合适大小的 ASCII logo。"""
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
    logos = [__logo, __logo_small, __logo_text]

    # 按照从大到小顺序，找到第一个能够在窗口中完整展示的 logo
    for logo in logos:
        w = max(len(line) for line in logo.splitlines())
        h = len(logo.splitlines())
        if width >= w and height//2 >= h:
            return logo
    return logos[-1]

def setup(win):
    """
    初始化 curses 配色和输入模式。"""
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_CYAN)
    win.keypad(False)
    curses.echo()
    curses.curs_set(1)

def frame_welcome(win, height, width):
    """
    欢迎界面，展示 logo 和提示，直到用户按键继续。"""
    win.nodelay(False)
    logo = get_logo(height, width)
    text1 = "DSLab AgRobot Terminal for GT-02"
    text2 = "<Press Any to Continue, 'q' to quit>"
    win.clear(); win.refresh()

    # 颜色1 渲染 logo
    win.attron(curses.color_pair(1))
    for idx, line in enumerate(logo.splitlines()[::-1]):
        y = height//2 - idx -1
        x = (width - len(line))//2
        win.addstr(y, x, line)
    win.attroff(curses.color_pair(1))

    # 颜色2 + A_BOLD 渲染标题
    win.attron(curses.color_pair(2)|curses.A_BOLD)
    win.addstr(height//2+1, (width-len(text1))//2, text1)
    win.attroff(curses.color_pair(2)|curses.A_BOLD)

    # 渲染继续提示
    win.addstr(height//2+3, (width-len(text2))//2, text2)
    win.refresh()

    # 按 'q' 退出，否则继续
    if win.getch() == ord('q'):
        exit()
    win.clear(); win.refresh()
    win.nodelay(True)

def main_loop(win, height, width):
    """
    主循环：
    1. 动态渲染日志区、提示区、输入区
    2. 非阻塞读取按键，编辑 raw_input
    3. 回车后拆分命令，校验状态跳转，分发执行
    4. 更新日志 cmds 并刷新界面
    5. 支持 quit() 退出和导航线程 stop
    """
    raw = ''                # 当前未提交的输入缓存
    cmds = ['History Logs'] # 日志/历史队列
    cur = 'IDLE'            # 当前状态机状态
    win.nodelay(True)       # 非阻塞模式读取按键
    while True:
        height, width = win.getmaxyx()
        # 1) 渲染历史cmds
        for i, line in enumerate(cmds[-MAX_CMD_ROWS:]):
            win.addstr(i, 0, line[:width-1])
        # 2) 渲染分隔线与提示
        win.addstr(height-3, 0, '┌'+'─'*(width-2))
        tips = f"Current {cur}. Available cmds: {','.join(COMMANDS.keys())}"
        win.addstr(height-2, 0, tips[:width-1])
        # 3) 渲染输入行
        win.attron(curses.color_pair(4))
        win.addstr(height-1, 0, ' admin@GT-02 > ')
        win.attroff(curses.color_pair(4))
        win.addstr(height-1, 15, raw)
        win.refresh()
        ch = win.getch()
        if ch == -1:
            continue
        key = curses.keyname(ch).decode('utf-8')
        if key == '^H':         # Backspace
            raw = raw[:-1]
        elif key == '^J':       # Enter
            if not raw.strip():
                raw = ''
                continue
            parts = raw.strip().split()
            raw = ''
            cmd = parts[0].upper()
            args = parts[1:]
            if cmd == 'QUIT()':
                break
            if cmd not in COMMANDS:
                cmds.append(f"Unknown {cmd}")
                continue

            # 解析参数并分发调用
            cfg = COMMANDS[cmd]
            try:
                params = parse_args(args, cfg)
                inst = cfg['handler'](**params)
                getattr(inst, cfg['method'])(**params)
                c = len(inst.points) if hasattr(inst, 'points') else ''
                cmds.append(cfg['log'].format(count=c, **params))
                cur = cmd
            except Exception as e:
                cmds.append(f"Error: {e}")
        else:
            raw += key
        win.clear()

# 主入口
def main(win):
    setup(win)
    h, w = win.getmaxyx()
    frame_welcome(win, h, w)
    main_loop(win, h, w)

if __name__ == '__main__':
    curses.wrapper(main)