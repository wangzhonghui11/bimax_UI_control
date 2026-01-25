# config.py
"""
配置文件 - 机器人和服务配置
"""

# 机器人配置
ROBOTS = {
    "ROBOT0 (DOMAIN=0)": {"domain": "0", "ip": "192.0.0.0"},
    "ROBOT7 (DOMAIN=80)": {"domain": "80", "ip": "192.168.0.107"},
    "ROBOT4 (DOMAIN=50)": {"domain": "50", "ip": "192.168.2.196"},
    "ROBOT9 (DOMAIN=100)": {"domain": "100", "ip": "192.168.0.109"},
}

# ROS2配置
ROS_CONFIG = {
    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
}
# 相机配置
CAMERA_CONFIG = {
    # 相机话题
    "topics": {
        "color": "/camera/color/image_raw",      # 彩色相机
        "depth": "/camera/depth/image_rect_raw",  # 深度相机
        "ir": "/camera/infra1/image_rect_raw",    # 红外相机
    },
    
    # 默认设置
    "default_camera": "color",      # 默认相机类型
    "default_timeout": 3.0,         # 默认超时时间（秒）
    "default_width": 640,           # 默认显示宽度
    "default_quality": 85,          # JPEG压缩质量(0-100)
    
    # 保存设置
    "default_save_dir": "/tmp/camera_captures",  # 默认保存目录
    "filename_format": "{camera_type}_{timestamp}.jpg",  # 文件名格式
    
    # 捕获设置
    "max_queue_size": 3,            # 图像队列最大大小
    "max_burst_count": 20,          # 最大连续捕获张数
    "min_burst_interval": 0.5,      # 最小连续捕获间隔（秒）
}
# 夹爪动作配置
GRASP_COMMANDS = {
    "move_grasp": "move&grasp_all",
    "only_grasp": "onlygrasp_all", 
    "grasp_slipper": "grasp_slipper",
    "putinto_basket": "putinto_basket",
    "putinto_trash_bin": "putinto_trash_bin",
    "put_down": "put_down",
}

# 移动命令配置
MOVEMENT_COMMANDS = {
    "forward": (0.1, 0.0),
    "backward": (-0.1, 0.0),
    "left": (0.0, 0.1745),
    "right": (0.0, -0.1745),
    "stop": (0.0, 0.0),
}

# 机械臂控制参数
ARM_PARAMS = {
    "home": [
        {"q": 0.1, "mode": None},
        {"q": 0.0, "mode": 0},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.1, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
    ],
    "fold": [
        {"q": 0.1, "mode": None},
        {"q": 1.0, "mode": 0},
        {"q": -1.1, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.1, "mode": None},
        {"q": -1.0, "mode": None},
        {"q": 1.1, "mode": None},
        {"q": 0.0, "mode": None},
    ]
}

# 夹爪参数
JAW_PARAMS = {
    "close": {"pos": 90.0, "mode": 0},
    "open": {"pos": 45.0, "mode": 1},
}

# 服务名称配置
SERVICE_NAMES = {
    "magnet": "/magnet_control",
    "catcher": "/catcher_control",
    "mop": "/mop_control",
    "reset": "/MotorFaultReset",
    "init": "/MotorFaultSet",
    "wash": "/station/control/wash",
    "dust": "/station/control/dust",
    "dry": "/station/control/dry",
    "grasp_action": "/function/arm/grasp",
    "camera_save": "/camera/save_images",  # 新增相机保存服务

}

# 话题名称配置
TOPIC_NAMES = {
    "cmd_vel": "/cmd_vel",
    "gripper": "/gripper_position",
    "arm_command": "/bimaxArmCommandValues",
    "robot_state": "/bimaxArmStateValues",  # 注意：这里使用的是RobotState消息

}