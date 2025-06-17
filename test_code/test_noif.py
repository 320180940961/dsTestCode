from test_danli import get_config_manager

# 1. 定义各操作函数
def handle_add(x, y):
    return x + y

def handle_sub(x, y):
    return x - y

def handle_mul(x, y):
    return x * y

# 获取 ConfigManager 的单例实例
config_manager = get_config_manager()

# 2. 构建操作映射表（字典驱动）
dispatch_table = {
    'add': handle_add,
    'sub': handle_sub,
    'mul': handle_mul,
    # 'mul': "handle_mul", # 
    'load_config': config_manager.load_config,  # 注意：这是一个绑定方法
    # 'load_config': "ConfigManager->load_config"   ,  # 注意：这是一个绑定方法
}


# 3. 动态分发函数
def calculate(op, *args, **kwargs):
    try:
        func = dispatch_table[op]  # O(1) 查找
    except KeyError:
        raise ValueError(f"Unknown operation: {op}")
    return func(*args, **kwargs)  # 动态调用

# 调用示例
print(calculate('mul', 3, 4))         # 输出 12
calculate('load_config')              # 输出 "正在加载配置文件..."
print(config_manager.settings)        # 输出 {'theme': 'dark', 'language': 'zh-CN'}
