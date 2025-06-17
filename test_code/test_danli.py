class ConfigManager:
    _instance = None  # 类变量保存唯一实例
    
    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            # 初始化配置（只执行一次）
            cls._instance.load_config()
        return cls._instance
    
    def load_config(self):
        print("正在加载配置文件...")
        self.settings = {"theme": "dark", "language": "zh-CN"}

def get_config_manager():
    return ConfigManager()

# 测试
config1 = ConfigManager()  # 输出"正在加载配置文件..."
config2 = ConfigManager()

print(config1 is config2)  # True（两个变量指向同一个实例）
print(config1.settings["theme"])  # dark