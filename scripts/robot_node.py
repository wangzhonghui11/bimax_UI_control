# robot_node.py
"""
ROS2节点类
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from bimax_msgs.action import BimaxFunction
from bimax_msgs.srv import MagnetControl, CatcherControl, MopControl
from bimax_msgs.msg import JawCommand, RobotCommand, MotorCommand, RobotState, MotorState
from std_srvs.srv import Trigger, SetBool ,Empty
from .config import SERVICE_NAMES, TOPIC_NAMES
import threading
from .camera_handler import CameraHandler

class RobotNode(Node):
    def __init__(self, domain_id):
        super().__init__(f'robot_ctrl_{domain_id}')
        
        # 初始化发布者和订阅者
        self._init_publishers()
        self._init_subscribers()
        self._init_clients()
        self._init_action_clients()
        # 存储状态数据
        # 存储状态数据和接收时间
        self.robot_state_data = None
        self.last_receive_time = None  # 最后接收时间
        self.robot_state_lock = threading.Lock()
        # 添加相机处理器
        self.camera_handler = None
        self._init_camera_handler() 

        # 超时设置（秒）
        self.state_timeout = 2.0  # 2秒超时      
        # 等待服务连接
        self.wait_for_services()
    def _init_camera_handler(self):
        """初始化相机处理器"""
        self.camera_handler = CameraHandler(self)
        self.get_logger().info("📷 相机处理器已附加到节点")    
    def _init_publishers(self):
        """初始化发布者"""
        self.pub = self.create_publisher(Twist, TOPIC_NAMES['cmd_vel'], 10)
        self.msg = Twist()
        self.jaw_publisher = self.create_publisher(JawCommand, TOPIC_NAMES['gripper'], 10)
        self.arm_publisher = self.create_publisher(RobotCommand, TOPIC_NAMES['arm_command'], 10)
    def _init_subscribers(self):
        """初始化订阅者"""
        # 订阅机器人状态（注意：使用RobotState消息）
        self.robot_state_subscriber = self.create_subscription(
            RobotState,
            TOPIC_NAMES['robot_state'],
            self._robot_state_callback,
            10
        )
        self.get_logger().info(f"已订阅机器人状态话题: {TOPIC_NAMES['robot_state']}")    
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
        # 相机服务（新增）
        self.camera_save_client = self.create_client(Empty, SERVICE_NAMES['camera_save'])   
    def _init_action_clients(self):
        """初始化Action客户端"""
        self.action_client = ActionClient(self, BimaxFunction, SERVICE_NAMES['grasp_action'])
        self.get_logger().info("等待action服务器...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        self.get_logger().info("✅ Action服务器已连接")
    def _robot_state_callback(self, msg):
        """机器人状态回调函数"""
        current_time = time.time()
        
        with self.robot_state_lock:
            self.robot_state_data = msg
            self.last_receive_time = current_time
            
    def get_robot_state(self):
        """获取当前机器人状态，如果超时则返回None"""
        with self.robot_state_lock:
            current_time = time.time()
            
            # 检查是否有数据和是否超时
            if self.robot_state_data is None or self.last_receive_time is None:
                return None
            
            time_since_last = current_time - self.last_receive_time
            if time_since_last > self.state_timeout:
                # 数据已超时，记录日志
                self.get_logger().debug(f"状态数据已超时: {time_since_last:.1f}秒 > {self.state_timeout}秒")
                return None
            
            # 数据有效
            return self.robot_state_data  
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
            ("相机保存", self.camera_save_client),  # 新增
        ]
        
        for name, client in services:
            if client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info(f"✅ {name}服务已连接")
            else:
                self.get_logger().info(f"✅ {name}服务已连接")