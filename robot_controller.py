# robot_controller.py
"""
机器人控制器类 - 主要业务逻辑
"""

import os
import rclpy
import threading
import subprocess
import time
from robot_node import RobotNode
from bimax_msgs.action import BimaxFunction
from bimax_msgs.srv import MagnetControl, CatcherControl, MopControl
from bimax_msgs.msg import JawCommand, RobotCommand, MotorCommand
from std_srvs.srv import Trigger, SetBool
from config import ROBOTS, GRASP_COMMANDS, MOVEMENT_COMMANDS, ARM_PARAMS, JAW_PARAMS, ROS_CONFIG


class RobotController:
    def __init__(self):
        # 设置ROS环境
        for key, value in ROS_CONFIG.items():
            os.environ[key] = value
        
        self.current_robot = "ROBOT0 (DOMAIN=0)"
        self.domain_id = "0"
        self.ip = "192.0.0.0"
        self.node = None
        self.setup_ros2()
        self.command_grasp = GRASP_COMMANDS
    
    def setup_ros2(self):
        """设置ROS2环境并启动节点"""
        os.environ['ROS_DOMAIN_ID'] = self.domain_id
        if rclpy.ok():
            rclpy.shutdown()
        
        rclpy.init()
        self.node = RobotNode(self.domain_id)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
    
    def ping_test(self):
        """Ping测试机器人网络连接"""
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
        """切换机器人"""
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
    
    def send_arm_command(self, command_type, action_name=""):
        """发送机械臂控制命令"""
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            if command_type not in ARM_PARAMS:
                return f"❌ 未知机械臂命令: {command_type}"
            
            msg = RobotCommand()
            motor_commands = []
            
            for params in ARM_PARAMS[command_type]:
                motor_cmd = MotorCommand()
                motor_cmd.q = params["q"]
                if params["mode"] is not None:
                    motor_cmd.mode = params["mode"]
                motor_commands.append(motor_cmd)
            
            msg.motor_command = motor_commands
            
            start_time = time.time()
            self.node.arm_publisher.publish(msg)
            elapsed = time.time() - start_time
            
            action_desc = action_name if action_name else f"机械臂{command_type}"
            self.node.get_logger().info(f"发布{action_desc}命令")
            
            return f"✅ {action_desc}命令已发布 ({elapsed:.1f}s)"
                
        except Exception as e:
            return f"❌ 机械臂{command_type}异常: {str(e)[:50]}"
    
    def send_arm_home(self, action_name="机械臂回零"):
        """发送机械臂回零命令"""
        return self.send_arm_command("home", action_name)
    
    def send_arm_fold(self, action_name="机械臂收臂"):
        """发送机械臂收臂命令"""
        return self.send_arm_command("fold", action_name)
    
    def send_jaw_command(self, command_type, action_name=""):
        """发送夹爪控制命令"""
        if command_type not in JAW_PARAMS:
            return f"❌ 未知夹爪命令: {command_type}"
        
        params = JAW_PARAMS[command_type]
        desc = action_name if action_name else f"夹爪{command_type}"
        
        if not self.node:
            return f"❌ 节点未就绪"
        
        try:
            msg = JawCommand()
            msg.jaw_pos_cmd = params["pos"]
            msg.jaw_mode = params["mode"]
            
            start_time = time.time()
            self.node.jaw_publisher.publish(msg)
            elapsed = time.time() - start_time
            
            self.node.get_logger().info(f"发布夹爪命令: 位置={params['pos']}°, 模式={params['mode']}")
            
            return f"✅ {desc}命令已发布 ({elapsed:.1f}s)"
                
        except Exception as e:
            return f"❌ {desc}异常: {str(e)[:50]}"
    
    def send_magnet_command(self, left_state, right_state, action_name=""):
        """发送电磁铁控制命令"""
        desc = action_name if action_name else f"电磁铁控制(左={left_state}, 右={right_state})"
        
        try:
            left_valid = 1 if left_state == 1 else 0
            right_valid = 1 if right_state == 1 else 0
            
            request = MagnetControl.Request()
            request.left_magnet_state = left_valid
            request.right_magnet_state = right_valid
            
            future = self.node.magnet_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        self.node.get_logger().info(f"setMagnetSwitch left={left_valid} right={right_valid}")
                        return f"✅ 成功: {desc}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ 服务返回失败: {desc} - {error_msg}"
                except Exception as e:
                    return f"❌ 响应异常: {desc} - {str(e)[:50]}"
            else:
                return f"❌ 调用超时: {desc}"
                
        except Exception as e:
            return f"❌ 调用异常: {desc} - {str(e)[:50]}"
    
    def send_cmd(self, cmd_type):
        """发送移动命令"""
        if not self.node:
            return f"❌ 节点未就绪"
        
        if cmd_type in MOVEMENT_COMMANDS:
            linear, angular = MOVEMENT_COMMANDS[cmd_type]
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
            request = CatcherControl.Request()
            request.catcher_gear = catcher_gear
            request.catcher_state = catcher_state
            
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
            request = MopControl.Request()
            request.mop_motor_pwm = mop_motor_pwm
            request.mop_state = mop_state
            
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
            request = Trigger.Request()
            
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
            
            time.sleep(0.2)
            
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
        
        goal_msg = BimaxFunction.Goal()
        goal_msg.command = self.command_grasp[command_name]
        
        self.node.get_logger().info(f"发送action: {command_name} -> {goal_msg.command}")
        
        future = self.node.action_client.send_goal_async(goal_msg)
        
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