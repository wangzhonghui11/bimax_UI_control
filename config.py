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
}

# 话题名称配置
TOPIC_NAMES = {
    "cmd_vel": "/cmd_vel",
    "gripper": "/gripper_position",
    "arm_command": "/bimaxArmCommandValues",
}