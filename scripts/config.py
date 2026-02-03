# config.py
"""
配置文件 - 机器人和服务配置
"""

# 机器人配置
ROBOTS = {
    "ROBOT0 (DOMAIN=0)": {"domain": "0", "ip": "0.0.0.0"},
    "ROBOT4 (DOMAIN=50)": {"domain": "50", "ip": "192.168.2.196"},
    "ROBOT5 (DOMAIN=60)": {"domain": "60", "ip": "192.168.2.197"},
    "ROBOT6 (DOMAIN=70)": {"domain": "70", "ip": "192.168.2.198"},
    "ROBOT7 (DOMAIN=80)": {"domain": "80", "ip": "192.168.2.199"},
    "ROBOT9 (DOMAIN=100)": {"domain": "100", "ip": "192.168.2.201"},
    "ROBOT10 (DOMAIN=110)": {"domain": "110", "ip": "192.168.2.195"},
    "ROBOT11 (DOMAIN=120)": {"domain": "120", "ip": "192.168.2.135"},
    "ROBOT12 (DOMAIN=130)": {"domain": "130", "ip": "192.168.2.69"},
    "ROBOT14 (DOMAIN=150)": {"domain": "150", "ip": "192.168.2.238"},
}

# ROS2配置
ROS_CONFIG = {
    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
}
# 完整的用户命令配置
USER_COMMANDS = {
    # ========== 通用功能 ==========
    "CANCEL": "CANCEL",              # 停止动作
    "BACK": "BACK",                  # 回到场地中央
    
    # 工具取放
    "VAC_PLACE": "VAC_PLACE",        # 放吸尘器
    "VAC_TAKE": "VAC_TAKE",          # 取吸尘器
    "MOP_PLACE": "MOP_PLACE",        # 放拖布
    "MOP_TAKE": "MOP_TAKE",          # 取拖布
    "MOP_CLEAN": "MOP_CLEAN",        # 洗拖布
    
    # ========== 1号场地（主持人） ==========
    "SHOW": "SHOW",                  # 开场动作展示
    "PICK": "PICK",                  # 抓物品放置
    "PICK_SLIPPER": "PICK_SLIPPER",  # 抓拖鞋放置
    "VAC": "VAC",                    # 识别吸尘
    "CHANGE_MOP": "CHANGE_MOP",      # 集尘放置吸尘器取拖布不回洗
    "MOP": "MOP",                    # 脏污识别擦拭
    
    # ========== 2号场地（巡航抓取清洁） ==========
    "PATROL_PICK_CLEAN": "PATROL_PICK_CLEAN",  # 全流程
    "PATROL_PICK": "PATROL_PICK",              # 识别抓取
    "PATROL_CLEAN": "PATROL_CLEAN",            # 巡航清洁
    "PATROL_VAC": "PATROL_VAC",                # 巡航吸尘
    "PATROL_MOP": "PATROL_MOP",                # 巡航擦拭
    
    # ========== 3号场地（复杂场景） ==========
    "COMPLEX_CLEAN": "COMPLEX_CLEAN",          # 全流程
    "COMPLEX_1_CLEAN": "COMPLEX_1_CLEAN",      # 1号凳子处理
    "COMPLEX_2_CLEAN": "COMPLEX_2_CLEAN",      # 2号凳子处理
    "COMPLEX_3_CLEAN": "COMPLEX_3_CLEAN",      # 电视柜处理
    
    # ========== 4号场地（大面清洁） ==========
    "WHOLE_CLEAN": "WHOLE_CLEAN",              # 全流程
    "WHOLE_VAC": "WHOLE_VAC",                  # 弓形吸尘
    "WHOLE_MOP": "WHOLE_MOP",                  # 弓形擦地
    "EDGE_MOP": "EDGE_MOP",                    # 延边擦地
}
# 相机配置
CAMERA_CONFIG = {
    # 相机话题
    "topics": {
        "color": "/camera/color/image_raw",      # 彩色相机
        "depth": "/camera/depth/image_raw/compressedDepth",  # 深度相机
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
    "forward": (0.15, 0.0),
    "backward": (-0.15, 0.0),
    "left": (0.0, 0.32),
    "right": (0.0, -0.32),
    "stop": (0.0, 0.0),
}

# 机械臂控制参数
ARM_PARAMS = {
    "home": [
        {"q": 0.12, "mode": None},
        {"q": 0.0, "mode": 0},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.12, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.0, "mode": None},
    ],
    "fold": [
        {"q": 0.12, "mode": None},
        {"q": 1.1, "mode": 0},
        {"q": -1.1, "mode": None},
        {"q": 0.0, "mode": None},
        {"q": 0.12, "mode": None},
        {"q": -1.1, "mode": None},
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
    "water": "/station/control/water",
    "grasp_action": "/function/arm/grasp",

}

# 话题名称配置
TOPIC_NAMES = {
    "cmd_vel": "/cmd_vel",
    "gripper": "/gripper_position",
    "arm_command": "/bimaxArmCommandValues",
    "robot_state": "/bimaxArmStateValues",  # 注意：这里使用的是RobotState消息
    "user_command": "/user/command",     # 用户命令话题
}
# 在文件末尾添加 ROS2 Action 配置：
# ROS2 Action配置
ROS2_ACTIONS = {
    # 机械臂抓取动作
    "arm_grasp": {
        "action_name": "/function/arm/grasp",
        "action_type": "bimax_msgs/action/BimaxFunction",
        "command": "activate",
        "timeout": 10.0,
        "description": "激活机械臂抓取"
    }   
}

# SSH 可选目标（下拉多选来源）
# 你可以放机器人ip，也可以放网关/上位机ip
SSH_HOSTS = {
    "Robot0": "0.0.0.0",
    "Robot4": "192.168.2.196",
    "Robot5": "192.168.2.197",
    "Robot6": "192.168.2.198",
    "Robot7": "192.168.2.199",
    "Robot9": "192.168.2.201",
    "Robot10": "192.168.2.195",
    "Robot11": "192.168.2.135",
    "Robot12": "192.168.2.69",
    "Robot14": "192.168.2.238",
}

SSH_CONFIG = {
    "enabled": True,
    "default_host_label": "Robot7",   # 默认下拉选项 key
    "port": 22,
    "username": "bimax",
    "password": "123",           
}

SSH_PRESET_COMMANDS = {
    "SYS_INFO": "uname -a",
    "PS_ROS": "ps -ef | grep -E 'ros2|bimax' | grep -v grep",
    "ROS_TOPIC_LIST": "source /opt/ros/humble/setup.bash >/dev/null 2>&1; ros2 topic list",
    "BIMAX_START": 'bash -lc "nohup bash /home/bimax/workspace/bimax_ws/src/bimax_main_entry/run.sh >/tmp/bimax_run.log 2>&1 &"',
    "BIMAX_KILL": 'bash -lc "bash /home/bimax/workspace/bimax_ws/src/bimax_main_entry/kill.sh"',
}