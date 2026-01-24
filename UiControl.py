#!/usr/bin/env python3
"""
最小改动 - 修复下拉菜单状态
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
import gradio as gr
import threading
import subprocess
import time
from bimax_msgs.action import BimaxFunction
from bimax_msgs.srv import MagnetControl, CatcherControl, MopControl
from bimax_msgs.msg import JawCommand, RobotCommand  # 新增机械臂消息
from bimax_msgs.msg import MotorCommand  # 用于RobotCommand的数组
from std_srvs.srv import Trigger, SetBool  # 添加SetBool服务

# 方法1: 使用FastDDS并增加缓冲区
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
# os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '1'
# 机器人配置
ROBOTS = {
    "ROBOT0 (DOMAIN=0)": {"domain": "0", "ip": "192.0.0.0"},
    "ROBOT7 (DOMAIN=80)": {"domain": "80", "ip": "192.168.0.107"},
    "ROBOT4 (DOMAIN=50)": {"domain": "50", "ip": "192.168.2.196"},
    "ROBOT9 (DOMAIN=100)": {"domain": "100", "ip": "192.168.0.109"},
}

class RobotController:
    def __init__(self):
        self.current_robot = "ROBOT0 (DOMAIN=0)"
        self.domain_id = "0"
        self.ip = "192.0.0.0"
        self.node = None
        self.setup_ros2()
        self.command_grasp = {
            "move_grasp": "move&grasp_all",
            "only_grasp": "onlygrasp_all", 
            "grasp_slipper": "grasp_slipper",
            "putinto_basket": "putinto_basket",
            "putinto_trash_bin": "putinto_trash_bin",
            "put_down": "put_down",
        }
    def setup_ros2(self):
        os.environ['ROS_DOMAIN_ID'] = self.domain_id
        if rclpy.ok():
            rclpy.shutdown()
        
        rclpy.init()
        self.node = RobotNode(self.domain_id)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
    
    def ping_test(self):
        try:
            result = subprocess.run(
                ["ping", "-c", "2", "-W", "1", self.ip],
                capture_output=True,
                text=True,
                timeout=3
            )
            return "✅ 在线" if result.returncode == 0 else "❌ 离线"
        except:
            return "❌ 超时"
    
    def switch_robot(self, robot_name):
        if robot_name in ROBOTS:
            self.current_robot = robot_name
            self.domain_id = ROBOTS[robot_name]["domain"]
            self.ip = ROBOTS[robot_name]["ip"]
            
            if self.node:
                self.node.destroy_node()
                rclpy.shutdown()
            
            self.setup_ros2()
            return f"✅ 已切换到: {robot_name}"
        return f"❌ 切换失败"
    def send_arm_home(self, action_name="机械臂回零"):
        """发送机械臂回零命令"""
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            # 创建回零命令
            msg = RobotCommand()
            
            # 创建8个电机命令
            motor_commands = []
            
            # 回零参数: [{q: 0.1}, {q: 0.0, mode: 0},{q: 0.0},{q: 0.0},{q: 0.1},{q: 0.0},{q: 0.0},{q: 0.0}]
            motor_params = [
                {"q": 0.1, "mode": None},    # 电机1
                {"q": 0.0, "mode": 0},       # 电机2
                {"q": 0.0, "mode": None},    # 电机3
                {"q": 0.0, "mode": None},    # 电机4
                {"q": 0.1, "mode": None},    # 电机5
                {"q": 0.0, "mode": None},    # 电机6
                {"q": 0.0, "mode": None},    # 电机7
                {"q": 0.0, "mode": None},    # 电机8
            ]
            
            for params in motor_params:
                motor_cmd = MotorCommand()
                motor_cmd.q = params["q"]
                if params["mode"] is not None:
                    motor_cmd.mode = params["mode"]
                motor_commands.append(motor_cmd)
            
            msg.motor_command = motor_commands
            
            # 发布消息
            start_time = time.time()
            self.node.arm_publisher.publish(msg)
            elapsed = time.time() - start_time
            
            self.node.get_logger().info(f"发布机械臂回零命令")
            
            return f"✅ 机械臂回零命令已发布 ({elapsed:.1f}s): {action_name}"
                
        except Exception as e:
            return f"❌ 机械臂回零异常: {action_name} - {str(e)[:50]}"
    def send_arm_fold(self, action_name="机械臂收臂"):
        """发送机械臂收臂命令"""
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            # 创建收臂命令
            msg = RobotCommand()
            
            # 创建8个电机命令
            motor_commands = []
            
            # 收臂参数: [{q: 0.1}, {q: 1.0, mode: 0},{q: -1.1},{q: 0.0},{q: 0.1},{q: -1.0},{q: 1.1},{q: 0.0}]
            motor_params = [
                {"q": 0.1, "mode": None},    # 电机1
                {"q": 1.0, "mode": 0},       # 电机2
                {"q": -1.1, "mode": None},   # 电机3
                {"q": 0.0, "mode": None},    # 电机4
                {"q": 0.1, "mode": None},    # 电机5
                {"q": -1.0, "mode": None},   # 电机6
                {"q": 1.1, "mode": None},    # 电机7
                {"q": 0.0, "mode": None},    # 电机8
            ]
            
            for params in motor_params:
                motor_cmd = MotorCommand()
                motor_cmd.q = params["q"]
                if params["mode"] is not None:
                    motor_cmd.mode = params["mode"]
                motor_commands.append(motor_cmd)
            
            msg.motor_command = motor_commands
            
            # 发布消息
            start_time = time.time()
            self.node.arm_publisher.publish(msg)
            elapsed = time.time() - start_time
            
            self.node.get_logger().info(f"发布机械臂收臂命令")
            
            return f"✅ 机械臂收臂命令已发布 ({elapsed:.1f}s): {action_name}"
                
        except Exception as e:
            return f"❌ 机械臂收臂异常: {action_name} - {str(e)[:50]}"
    def send_jaw_command(self, jaw_pos_cmd, jaw_mode, action_name=""):
        """发送夹爪控制命令"""
        desc = action_name if action_name else f"夹爪控制(位置={jaw_pos_cmd}°, 模式={jaw_mode})"
        
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            # 创建消息
            msg = JawCommand()
            msg.jaw_pos_cmd = jaw_pos_cmd
            msg.jaw_mode = jaw_mode
            
            # 发布消息
            start_time = time.time()
            self.node.jaw_publisher.publish(msg)
            elapsed = time.time() - start_time
            
            self.node.get_logger().info(f"发布夹爪命令: 位置={jaw_pos_cmd}°, 模式={jaw_mode}")
            
            return f"✅ 夹爪命令已发布 ({elapsed:.1f}s): {desc}"
                
        except Exception as e:
            return f"❌ 夹爪异常: {desc} - {str(e)[:50]}"
    def send_magnet_command(self, left_state, right_state, action_name=""):
        """发送电磁铁控制命令 - 同步版本（对应C++逻辑）"""
        desc = action_name if action_name else f"电磁铁控制(左={left_state}, 右={right_state})"
        
        try:
            # 1. Validate values are 0 or 1 (C++代码中的验证)
            left_valid = 1 if left_state == 1 else 0
            right_valid = 1 if right_state == 1 else 0
            
            print(f"[RosClass] 开始调用setMagnetSwitch left={left_valid} right={right_valid}")
            
            # 2. 创建请求
            request = MagnetControl.Request()
            request.left_magnet_state = left_valid
            request.right_magnet_state = right_valid
            
            # 3. 同步调用服务
            future = self.node.magnet_client.call_async(request)
            
            # 4. 等待响应，超时1秒
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            # 5. 检查状态
            if future.done():
                try:
                    response = future.result()
                    
                    # 6. 根据response.success判断结果
                    if response.success:
                        self.node.get_logger().info(
                            f"[RosClass] setMagnetSwitch left={left_valid} right={right_valid}"
                        )
                        return f"✅ 成功: {desc}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        self.node.get_logger().error(
                            f"[RosClass] setMagnetSwitch command failed: {error_msg}"
                        )
                        return f"❌ 服务返回失败: {desc} - {error_msg}"
                        
                except Exception as e:
                    self.node.get_logger().error(f"[RosClass] 获取响应异常: {e}")
                    return f"❌ 响应异常: {desc} - {str(e)[:50]}"
            else:
                self.node.get_logger().error("[RosClass] setMagnetSwitch service call timed out")
                return f"❌ 调用超时: {desc}"
                
        except Exception as e:
            self.node.get_logger().error(f"[RosClass] 调用异常: {e}")
            return f"❌ 调用异常: {desc} - {str(e)[:50]}"
    def send_cmd(self, cmd_type):
        if not self.node:
            return f"❌ 节点未就绪"
        
        commands = {
            "forward": (0.1, 0.0),
            "backward": (-0.1, 0.0),
            "left": (0.0, 0.1745),
            "right": (0.0, -0.1745),
            "stop": (0.0, 0.0),
        }
        
        if cmd_type in commands:
            linear, angular = commands[cmd_type]
            self.node.msg.linear.x = linear
            self.node.msg.angular.z = angular
            self.node.pub.publish(self.node.msg)
            return f"✅ {cmd_type}"
        return f"❌ 未知命令"
    def send_catcher_command(self, catcher_gear, catcher_state, action_name=""):
        """发送吸尘器控制命令"""
        desc = action_name if action_name else f"吸尘器控制(档位={catcher_gear}, 状态={catcher_state})"
        
        if not self.node or not self.node.catcher_client.service_is_ready():
            return f"❌ 吸尘器服务未就绪"
        
        try:
            # 创建请求
            request = CatcherControl.Request()
            request.catcher_gear = catcher_gear
            request.catcher_state = catcher_state
            
            # 同步调用
            start_time = time.time()
            future = self.node.catcher_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            elapsed = time.time() - start_time
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        return f"✅ 吸尘器成功 ({elapsed:.1f}s): {desc}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ 吸尘器失败 ({elapsed:.1f}s): {desc} - {error_msg}"
                except Exception as e:
                    return f"❌ 吸尘器响应错误 ({elapsed:.1f}s): {desc}"
            else:
                return f"❌ 吸尘器超时: {desc}"
                
        except Exception as e:
            return f"❌ 吸尘器异常: {desc} - {str(e)[:50]}"
    
    def send_mop_command(self, mop_motor_pwm, mop_state, action_name=""):
        """发送拖布控制命令"""
        desc = action_name if action_name else f"拖布控制(PWM={mop_motor_pwm}, 状态={mop_state})"
        
        if not self.node or not self.node.mop_client.service_is_ready():
            return f"❌ 拖布服务未就绪"
        
        try:
            # 创建请求
            request = MopControl.Request()
            request.mop_motor_pwm = mop_motor_pwm
            request.mop_state = mop_state
            
            # 同步调用
            start_time = time.time()
            future = self.node.mop_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            elapsed = time.time() - start_time
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        return f"✅ 拖布成功 ({elapsed:.1f}s): {desc}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ 拖布失败 ({elapsed:.1f}s): {desc} - {error_msg}"
                except Exception as e:
                    return f"❌ 拖布响应错误 ({elapsed:.1f}s): {desc}"
            else:
                return f"❌ 拖布超时: {desc}"
                
        except Exception as e:
            return f"❌ 拖布异常: {desc} - {str(e)[:50]}"
    def reset_motor_faults(self):
        """重置电机错误并初始化错误码"""
        if not self.node:
            return "❌ 节点未就绪"
        
        try:
            result_msgs = []
            
            # 1. 重置电机错误
            request = self.node.Trigger.Request()
            
            # 先调用重置
            if self.node.reset_client.service_is_ready():
                future = self.node.reset_client.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                
                if future.done():
                    try:
                        response = future.result()
                        if response.success:
                            result_msgs.append("✅ 电机错误重置成功")
                        else:
                            result_msgs.append(f"❌ 电机错误重置失败: {response.message}")
                    except Exception as e:
                        result_msgs.append(f"❌ 重置响应错误: {str(e)[:50]}")
                else:
                    result_msgs.append("❌ 重置服务调用超时")
            else:
                result_msgs.append("❌ 重置服务未就绪")
            
            time.sleep(0.2)  # 短暂延时
            
            # 2. 初始化错误码
            if self.node.init_client.service_is_ready():
                future = self.node.init_client.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                
                if future.done():
                    try:
                        response = future.result()
                        if response.success:
                            result_msgs.append("✅ 错误码初始化成功")
                        else:
                            result_msgs.append(f"❌ 错误码初始化失败: {response.message}")
                    except Exception as e:
                        result_msgs.append(f"❌ 初始化响应错误: {str(e)[:50]}")
                else:
                    result_msgs.append("❌ 初始化服务调用超时")
            else:
                result_msgs.append("❌ 初始化服务未就绪")
            
            return "\n".join(result_msgs)
                
        except Exception as e:
            return f"❌ 电机故障处理异常: {str(e)[:50]}"
    def send_action_command(self, command_name):
        """发送action命令"""
        if command_name not in self.command_grasp:
            return f"❌ 未知命令: {command_name}"
        
        # 创建goal
        goal_msg = BimaxFunction.Goal()
        goal_msg.command = self.command_grasp[command_name]
        
        self.node.get_logger().info(f"发送action: {command_name} -> {goal_msg.command}")
        
        # 发送goal
        future = self.node.action_client.send_goal_async(goal_msg)
        
        # 可以添加回调处理结果
        # future.add_done_callback(self.goal_response_callback)
        
        return f"📤 已发送: {command_name}"
    def call_station_service(self, service_name, data_value=True, action_name=""):
        """调用基站服务"""
        desc = action_name if action_name else f"{service_name}控制"
        
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            # 根据服务名称获取客户端
            client = None
            if service_name == "wash":
                client = self.node.wash_client
            elif service_name == "dust":
                client = self.node.dust_client
            elif service_name == "dry":
                client = self.node.dry_client
            else:
                return f"❌ 未知服务: {service_name}"
            
            if not client or not client.service_is_ready():
                return f"❌ {desc}服务未就绪"
            
            # 创建请求
            request = SetBool.Request()
            request.data = bool(data_value)
            
            # 调用服务
            start_time = time.time()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            elapsed = time.time() - start_time
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        return f"✅ {desc}成功 ({elapsed:.1f}s): 状态={'开启' if data_value else '关闭'}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ {desc}失败 ({elapsed:.1f}s): {error_msg}"
                except Exception as e:
                    return f"❌ {desc}响应错误 ({elapsed:.1f}s): {str(e)[:50]}"
            else:
                return f"❌ {desc}超时"
                
        except Exception as e:
            return f"❌ {desc}异常: {str(e)[:50]}"
class RobotNode(Node):
    def __init__(self, domain_id):
        super().__init__(f'robot_ctrl_{domain_id}')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.msg = Twist()
        # 夹爪控制发布者
        self.jaw_publisher = self.create_publisher(JawCommand, '/gripper_position', 10)
        self.action_client = ActionClient(self, BimaxFunction, '/function/arm/grasp')
        # 机械臂控制发布者
        self.arm_publisher = self.create_publisher(RobotCommand, '/bimaxArmCommandValues', 10)
        # 等待服务器
        self.get_logger().info("等待action服务器...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        self.get_logger().info("✅ Action服务器已连接")
        # 各种服务客户端
        self.MagnetControl = MagnetControl
        self.magnet_client = self.create_client(self.MagnetControl, '/magnet_control')
        
        self.CatcherControl = CatcherControl
        self.catcher_client = self.create_client(self.CatcherControl, '/catcher_control')
        self.Trigger = Trigger        
        self.reset_client = self.create_client(self.Trigger, '/MotorFaultReset')
        # 初始化服务客户端
        self.init_client = self.create_client(self.Trigger, '/MotorFaultSet')       
        self.MopControl = MopControl
        self.mop_client = self.create_client(self.MopControl, '/mop_control')
        # 基站服务
        self.SetBool = SetBool
        self.wash_client = self.create_client(self.SetBool, '/station/control/wash')
        self.dust_client = self.create_client(self.SetBool, '/station/control/dust')
        self.dry_client = self.create_client(self.SetBool, '/station/control/dry')
        # 等待服务连接
        self.wait_for_services()        
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
# 创建控制器
controller = RobotController()

# UI界面
# ... 前面的代码保持不变 ...

with gr.Blocks() as demo:
    gr.Markdown("# 🤖 机器人遥控")
    
    with gr.Tabs():
        with gr.TabItem("🎮 基础控制"):
            # Ping测试
            with gr.Row():
                ping_btn = gr.Button("📡 Ping测试", variant="secondary")
                ping_status = gr.Textbox("点击测试", label="网络状态")
            
            # 机器人选择
            with gr.Row():
                robot_select = gr.Dropdown(
                    list(ROBOTS.keys()),
                    value="ROBOT0 (DOMAIN=0)",  # 修正初始值
                    label="选择机器人"
                )
                switch_btn = gr.Button("切换", variant="primary")
            
            status = gr.Textbox("✅ 已连接", label="状态")
            
            gr.Markdown("---")
        with gr.TabItem("🔧 硬件控制检测"):
            # 控制面板
            with gr.Column():
                gr.Markdown("### 🔋 轮组控制")
                with gr.Row():
                    btn_w = gr.Button("⬆️ 前进", size="lg")
                with gr.Row():
                    btn_a = gr.Button("⬅️ 左转")
                    btn_s = gr.Button("⏹️ 停止", variant="secondary")
                    btn_d = gr.Button("➡️ 右转")
                with gr.Row():
                    btn_x = gr.Button("⬇️ 后退", size="lg")
            
            cmd_output = gr.Textbox("准备就绪", label="命令")
            # 电磁铁控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🔋 电磁铁控制")
                    
                    with gr.Row():
                        btn_magnet_on = gr.Button("🔋 充磁", variant="primary")
                        btn_magnet_off = gr.Button("🔌 退磁", variant="secondary")
                    
                    magnet_output = gr.Textbox("准备就绪", label="电磁铁状态")
            
            gr.Markdown("---")
            
            # 吸尘器控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🌀 吸尘器控制")
                    
                    with gr.Row():
                        btn_catcher_on = gr.Button("🌀 吸尘器开", variant="primary")
                        btn_catcher_off = gr.Button("🌀 吸尘器关", variant="secondary")
                    
                    catcher_output = gr.Textbox("准备就绪", label="吸尘器状态")
            
            gr.Markdown("---")
            
            # 拖布控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🧹 拖布控制")
                    
                    with gr.Row():
                        btn_mop_on = gr.Button("🧹 拖布开", variant="primary")
                        btn_mop_off = gr.Button("🧹 拖布关", variant="secondary")
                    
                    mop_output = gr.Textbox("准备就绪", label="拖布状态")
            
            gr.Markdown("---")
            
            # 夹爪控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🦾 夹爪控制")
                    
                    with gr.Row():
                        btn_jaw_close = gr.Button("🤏 关夹爪", variant="primary")
                        btn_jaw_open = gr.Button("🦾 开夹爪", variant="secondary")
                    
                    jaw_output = gr.Textbox("准备就绪", label="夹爪状态")
            
            gr.Markdown("---")
            
            # 机械臂控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🤖 机械臂控制")
                    
                    with gr.Row():
                        btn_arm_home = gr.Button("🏠 臂回零", variant="primary")
                        btn_arm_fold = gr.Button("📦 左右收臂", variant="secondary")
                    
                    arm_output = gr.Textbox("准备就绪", label="机械臂状态")
            
            gr.Markdown("---")
            # 电机故障处理
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### ⚙️ 电机故障处理")
                    
                    # with gr.Row():
                    btn_motor_reset = gr.Button("🔄 重置并初始化电机", variant="primary")
                    
                motor_output = gr.Textbox("点击按钮重置电机错误并初始化", label="电机状态")
            
            gr.Markdown("---")            
            # 机械臂动作控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("# 🤖 机械臂动作控制")
                    
                    with gr.Row():
                        btn1 = gr.Button("移动并抓取", variant="primary")
                        btn2 = gr.Button("仅抓取", variant="primary")
                        btn3 = gr.Button("抓拖鞋", variant="primary")
                    
                    with gr.Row():
                        btn4 = gr.Button("放入篮子", variant="primary")
                        btn5 = gr.Button("放入垃圾桶", variant="primary")
                        btn6 = gr.Button("放下", variant="primary")
                    
                    grasp_output = gr.Textbox("准备就绪", label="状态")
        with gr.TabItem("🏠 基站控制"):
            gr.Markdown("### 测试基站各项功能")
            
            # 清洗功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🚿 清洗功能")
                    
                    with gr.Row():
                        btn_wash_on = gr.Button("💦 开启清洗", variant="primary", size="lg")
                        btn_wash_off = gr.Button("⏹️ 关闭清洗", variant="secondary", size="lg")
                    
                    wash_output = gr.Textbox("准备测试清洗功能", label="清洗状态")
            
            gr.Markdown("---")
            
            # 吸尘功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🌪️ 吸尘功能")
                    
                    with gr.Row():
                        btn_dust_on = gr.Button("🌀 开启吸尘", variant="primary", size="lg")
                        btn_dust_off = gr.Button("⏹️ 关闭吸尘", variant="secondary", size="lg")
                    
                    dust_output = gr.Textbox("准备测试吸尘功能", label="吸尘状态")
            
            gr.Markdown("---")
            
            # 干燥功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🔥 干燥功能")
                    
                    with gr.Row():
                        btn_dry_on = gr.Button("🔥 开启干燥", variant="primary", size="lg")
                        btn_dry_off = gr.Button("⏹️ 关闭干燥", variant="secondary", size="lg")
                    
                    dry_output = gr.Textbox("准备测试干燥功能", label="干燥状态")
            
            gr.Markdown("---")
            

# ... 事件绑定代码保持不变 ...
    # 事件绑定
    def do_ping():
        return controller.ping_test()
    
    def switch(robot):
        result = controller.switch_robot(robot)
        # 修复：返回机器人选择，保持下拉菜单状态
        return result, robot  # 第二个返回值保持下拉菜单选择
    def magnet_on():
        return controller.send_magnet_command(1, 1, "电磁铁充磁") 
    def magnet_off():
        return controller.send_magnet_command(0, 0, "电磁铁退磁") 
    def catcher_on():
        return controller.send_catcher_command(1, 1, "吸尘器开启")  
    def catcher_off():
        return controller.send_catcher_command(1, 0, "吸尘器关闭")
    def mop_on():
        return controller.send_mop_command(2000, 1, "拖布开启") 
    def mop_off():
        return controller.send_mop_command(2000, 0, "拖布关闭")  
    def jaw_close():
        """关夹爪 - 位置90°, 模式0"""
        return controller.send_jaw_command(90.0, 0, "关夹爪")
    def jaw_open():
        """开夹爪 - 位置45°, 模式1"""
        return controller.send_jaw_command(45.0, 1, "开夹爪") 
    def arm_home():
        """机械臂回零"""
        return controller.send_arm_home("机械臂回零")
    
    def arm_fold():
        """机械臂收臂"""
        return controller.send_arm_fold("机械臂收臂") 

    # 基站控制函数
    def wash_on():
        return controller.call_station_service("wash", True, "清洗功能")
    
    def wash_off():
        return controller.call_station_service("wash", False, "清洗功能")
    
    def dust_on():
        return controller.call_station_service("dust", True, "吸尘功能")
    
    def dust_off():
        return controller.call_station_service("dust", False, "吸尘功能")
    
    def dry_on():
        return controller.call_station_service("dry", True, "干燥功能")
    
    def dry_off():
        return controller.call_station_service("dry", False, "干燥功能") 
    ping_btn.click(do_ping, outputs=ping_status)
    
    # 关键修复：switch_btn点击时，输出到status和robot_select
    btn_motor_reset.click(
        lambda: controller.reset_motor_faults(),
        outputs=motor_output
    )
    switch_btn.click(
        switch,
        inputs=robot_select,
        outputs=[status, robot_select]  # 输出到下拉菜单，保持状态
    )
    
    # 下拉菜单变化时不切换，只更新提示
    robot_select.change(
        lambda x: f"准备切换到: {x}",
        inputs=robot_select,
        outputs=status
    )
    
    btn_w.click(lambda: controller.send_cmd("forward"), outputs=cmd_output)
    btn_x.click(lambda: controller.send_cmd("backward"), outputs=cmd_output)
    btn_a.click(lambda: controller.send_cmd("left"), outputs=cmd_output)
    btn_d.click(lambda: controller.send_cmd("right"), outputs=cmd_output)
    btn_s.click(lambda: controller.send_cmd("stop"), outputs=cmd_output)
    # 电磁铁控制
    btn_magnet_on.click(magnet_on, outputs=magnet_output)
    btn_magnet_off.click(magnet_off, outputs=magnet_output)
    
    # 吸尘器控制
    btn_catcher_on.click(catcher_on, outputs=catcher_output)
    btn_catcher_off.click(catcher_off, outputs=catcher_output)
     # 机械臂控制
    btn_arm_home.click(arm_home, outputs=arm_output)
    btn_arm_fold.click(arm_fold, outputs=arm_output)   
    # 拖布控制
    btn_mop_on.click(mop_on, outputs=mop_output)
    btn_mop_off.click(mop_off, outputs=mop_output)
    # 夹爪控制
    btn_jaw_close.click(jaw_close, outputs=jaw_output)
    btn_jaw_open.click(jaw_open, outputs=jaw_output)
    # 基站控制页面
    btn_wash_on.click(wash_on, outputs=wash_output)
    btn_wash_off.click(wash_off, outputs=wash_output)
    btn_dust_on.click(dust_on, outputs=dust_output)
    btn_dust_off.click(dust_off, outputs=dust_output)
    btn_dry_on.click(dry_on, outputs=dry_output)
    btn_dry_off.click(dry_off, outputs=dry_output)
     # 绑定事件
    btn1.click(lambda: controller.send_action_command("move_grasp"), outputs=grasp_output)
    btn2.click(lambda: controller.send_action_command("only_grasp"), outputs=grasp_output)
    btn3.click(lambda: controller.send_action_command("grasp_slipper"), outputs=grasp_output)
    btn4.click(lambda: controller.send_action_command("putinto_basket"), outputs=grasp_output)
    btn5.click(lambda: controller.send_action_command("putinto_trash_bin"), outputs=grasp_output)
    btn6.click(lambda: controller.send_action_command("put_down"), outputs=grasp_output)   
    # 页面加载时自动ping
    demo.load(do_ping, outputs=ping_status)

if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=7860)