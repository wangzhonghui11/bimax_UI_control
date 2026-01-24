# robot_node.py
"""
ROS2节点类
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from bimax_msgs.action import BimaxFunction
from bimax_msgs.srv import MagnetControl, CatcherControl, MopControl
from bimax_msgs.msg import JawCommand, RobotCommand, MotorCommand
from std_srvs.srv import Trigger, SetBool
from config import SERVICE_NAMES, TOPIC_NAMES


class RobotNode(Node):
    def __init__(self, domain_id):
        super().__init__(f'robot_ctrl_{domain_id}')
        
        # 初始化发布者和订阅者
        self._init_publishers()
        self._init_clients()
        self._init_action_clients()
        
        # 等待服务连接
        self.wait_for_services()
    
    def _init_publishers(self):
        """初始化发布者"""
        self.pub = self.create_publisher(Twist, TOPIC_NAMES['cmd_vel'], 10)
        self.msg = Twist()
        self.jaw_publisher = self.create_publisher(JawCommand, TOPIC_NAMES['gripper'], 10)
        self.arm_publisher = self.create_publisher(RobotCommand, TOPIC_NAMES['arm_command'], 10)
    
    def _init_clients(self):
        """初始化服务客户端"""
        # 基础服务
        self.magnet_client = self.create_client(MagnetControl, SERVICE_NAMES['magnet'])
        self.catcher_client = self.create_client(CatcherControl, SERVICE_NAMES['catcher'])
        self.mop_client = self.create_client(MopControl, SERVICE_NAMES['mop'])
        
        # 电机故障服务
        self.reset_client = self.create_client(Trigger, SERVICE_NAMES['reset'])
        self.init_client = self.create_client(Trigger, SERVICE_NAMES['init'])
        
        # 基站服务
        self.wash_client = self.create_client(SetBool, SERVICE_NAMES['wash'])
        self.dust_client = self.create_client(SetBool, SERVICE_NAMES['dust'])
        self.dry_client = self.create_client(SetBool, SERVICE_NAMES['dry'])
    
    def _init_action_clients(self):
        """初始化Action客户端"""
        self.action_client = ActionClient(self, BimaxFunction, SERVICE_NAMES['grasp_action'])
        self.get_logger().info("等待action服务器...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        self.get_logger().info("✅ Action服务器已连接")
    
    def wait_for_services(self):
        """等待各种服务"""
        services = [
            ("电磁铁", self.magnet_client),
            ("吸尘器", self.catcher_client),
            ("拖布", self.mop_client),
            ("电机重置", self.reset_client),
            ("电机初始化", self.init_client),
            ("基站清洗", self.wash_client),
            ("基站吸尘", self.dust_client),
            ("基站干燥", self.dry_client),
        ]
        
        for name, client in services:
            if client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info(f"✅ {name}服务已连接")
            else:
                self.get_logger().warn(f"⚠️ {name}服务未连接")