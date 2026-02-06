# robot_controller.py
"""
机器人控制器类 - 主要业务逻辑
"""

import datetime
import os
import rclpy
import threading
import subprocess
import time
from .robot_node import RobotNode
from bimax_msgs.action import BimaxFunction
from bimax_msgs.srv import MagnetControl, CatcherControl, MopControl,LedControl
from bimax_msgs.msg import JawCommand, RobotCommand, MotorCommand, RobotState, MotorState
from std_srvs.srv import Trigger, SetBool,Empty
from .config import ROBOTS, GRASP_COMMANDS, MOVEMENT_COMMANDS, ARM_PARAMS, JAW_PARAMS, ROS_CONFIG
from .command_handler import CommandHandler  # 导入命令处理器
from .ssh_command_client import SSHCommandClient
from .config import SSH_CONFIG, SSH_PRESET_COMMANDS

class RobotController:
    def __init__(self,domain_id,ip):
        # 设置ROS环境
        for key, value in ROS_CONFIG.items():
            os.environ[key] = value
        
        self.current_robot = "ROBOT0"
        self.domain_id = domain_id
        self.ip = ip
        self.node = None
        self.setup_ros2()
        self.command_grasp = GRASP_COMMANDS
        self.ssh_client = None
        self.ssh_username = None
        self.ssh_password = None    
    def setup_ros2(self):
        """设置ROS2环境并启动节点"""
        # os.environ['ROS_DOMAIN_ID'] = self.domain_id
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
                timeout=2
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
    def setup_ssh(self, host: str, username: str, password: str, port: int = 22):
        if self.ssh_client:
            try:
                self.ssh_client.close()
            except Exception:
                pass

        self.ssh_client = SSHCommandClient(
            host=host,
            port=port,
            username=username,
            password=password,
            timeout=5.0,
        )
        return f"✅ SSH 已配置: {username}@{host}:{port}"

    def ssh_run_preset(self, preset_name: str):
        if not SSH_CONFIG.get("enabled", False):
            return "❌ SSH 未启用"
        if not self.ssh_client:
            return "❌ SSH 未配置（请先选择IP并点击配置SSH）"

        cmd = SSH_PRESET_COMMANDS.get(preset_name)
        if not cmd:
            return f"❌ 未知预设命令: {preset_name}"

        try:
            result = self.ssh_client.exec(cmd, timeout=15.0)
            header = f"[{preset_name}] host={self.ssh_client.host} exit={result.exit_code}\n$ {cmd}\n"
            if result.ok:
                return (header + result.stdout).strip() or (header + "(no output)")
            return (header + "STDOUT:\n" + result.stdout + "\nSTDERR:\n" + result.stderr).strip()
        except Exception as e:
            return f"❌ SSH 执行异常: {str(e)[:200]}"
    def get_simple_robot_status(self):
        """获取简化的机器人状态"""
        if not self.node:
            return "❌ 节点未就绪"
        
        try:
            robot_state = self.node.get_robot_state()
            
            if robot_state is None:
                return "⏳ 等待机器人状态数据..."
            
            lines = []
            lines.append(f"🕐 更新时间: {datetime.now().strftime('%H:%M:%S')}")
            lines.append("")
            
            # 电机命名
            motor_names = ["左升降", "左臂1", "左臂2", "左臂3", "右升降", "右臂1", "右臂2", "右臂3"]
            
            for i, motor in enumerate(robot_state.motor_state):
                motor_name = motor_names[i] if i < len(motor_names) else f"电机{i}"
                
                if motor.error_id == 0:
                    status = "🟢"
                elif motor.error_id == -1:
                    status = "⚪"
                else:
                    status = f"🔴{motor.error_id}"
                
                lines.append(f"{motor_name}: 位置={motor.q:.3f}  {status}")
            
            # 统计
            total = len(robot_state.motor_state)
            normal = sum(1 for m in robot_state.motor_state if m.error_id == 0)
            errors = sum(1 for m in robot_state.motor_state if m.error_id != 0)
            
            lines.append("")
            lines.append(f"📈 正常: {normal}/{total}, 故障: {errors}/{total}")
            
            return "\n".join(lines)
            
        except Exception as e:
            return f"❌ 获取状态异常: {str(e)[:50]}"    
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
    def send_motor_zero_command(self, green_state, yellow_state, action_name=""):
        """发送LED控制命令"""
        desc = action_name if action_name else f"LED控制(绿色={green_state}, 黄色={yellow_state})"
        
        try:  
            # 创建请求
            start_time = time.time()
            request = LedControl.Request()
            request.green_state = int(green_state)
            request.yellow_state = int(yellow_state)
       
            # 异步调用服务
            future = self.node.motor_zero_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        self.node.get_logger().info(f"setLedSwitch green={green_state} yellow={yellow_state}")
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ 服务返回失败: {desc} - {error_msg}"
                except Exception as e:
                    return f"❌ 响应异常: {desc} - {str(e)[:50]}"
            else:
                return f"❌ 调用超时: {desc}" 
            time.sleep(1)
            request.green_state = int(1)
            request.yellow_state = int(1)    
            # 异步调用服务
            future = self.node.motor_zero_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            elapsed = time.time() - start_time                 
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        self.node.get_logger().info(f"setLedSwitch green={green_state} yellow={yellow_state}")
                        return f"✅ 电机控制成功 ({elapsed:.1f}s): {desc}"
                    else:
                        error_msg = response.message if hasattr(response, 'message') else "未知错误"
                        return f"❌ 服务返回失败: {desc} - {error_msg}"
                except Exception as e:
                    return f"❌ 响应异常: {desc} - {str(e)[:50]}"
            else:
                return f"❌ 调用超时: {desc}"    
        except Exception as e:
            return f"❌ 调用异常: {desc} - {str(e)[:50]}" 
               

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
            
            time.sleep(2)
            
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
            elif service_name == "water":
                client = self.node.water_client
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
    
    def capture_camera_image(self, camera_type=None, resize_width=None):
        """捕获相机图像 - 接口方法"""
        if not self.node or not self.node.camera_handler:
            return "❌ 相机处理器未就绪", None, None
        
        try:
            success, message, base64_data, cv_image = self.node.camera_handler.capture_single_image(
                camera_type=camera_type,
                resize_width=resize_width
            )
            
            if success:
                return f"✅ {message}", base64_data, cv_image
            else:
                return f"❌ {message}", base64_data, cv_image
                
        except Exception as e:
            error_msg = f"❌ 捕获异常: {str(e)[:50]}"
            return error_msg, None, None
    
    def capture_and_save_image(self, camera_type=None, save_dir=None, filename=None, resize_width=None):
        """捕获并保存图像 - 接口方法"""
        if not self.node or not self.node.camera_handler:
            return "❌ 相机处理器未就绪", None, None, None
        
        try:
            success, message, base64_data, file_path = self.node.camera_handler.capture_and_save(
                camera_type=camera_type,
                save_dir=save_dir,
                filename=filename,
                resize_width=resize_width
            )
            
            if success:
                return f"✅ {message}", base64_data, None, file_path
            else:
                return f"❌ {message}", base64_data, None, file_path
                
        except Exception as e:
            error_msg = f"❌ 捕获异常: {str(e)[:50]}"
            return error_msg, None, None, None
    
    def get_camera_status(self):
        """获取相机状态 - 接口方法"""
        if not self.node or not self.node.camera_handler:
            return {"initialized": False, "message": "相机处理器未就绪"}
        
        try:
            status = self.node.camera_handler.get_status()
            status["initialized"] = True
            return status
        except Exception as e:
            return {
                "initialized": False,
                "message": f"获取状态异常: {str(e)[:50]}"
            }
    
    def get_last_camera_image(self):
        """获取最后捕获的图像 - 接口方法"""
        if not self.node or not self.node.camera_handler:
            return None
        
        return self.node.camera_handler.get_last_capture()
    
    def send_cancel(self):
        """停止动作"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.cancel()
        return message
    
    def send_back(self):
        """回到场地中央"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.back()
        return message
    
    def send_vac_place(self):
        """放吸尘器"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.vac_place()
        return message
    
    def send_vac_take(self):
        """取吸尘器"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.vac_take()
        return message
    
    def send_mop_place(self):
        """放拖布"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.mop_place()
        return message
    
    def send_mop_take(self):
        """取拖布"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.mop_take()
        return message
    
    def send_mop_clean(self):
        """洗拖布"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.mop_clean()
        return message
    
    # ========== 1号场地（主持人） ==========
    def send_show(self):
        """开场动作展示"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.show()
        return message
    
    def send_pick(self):
        """抓物品放置"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.pick()
        return message
    
    def send_pick_slipper(self):
        """抓拖鞋放置"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.pick_slipper()
        return message
    
    def send_vac(self):
        """识别吸尘"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.vac()
        return message
    
    def send_change_mop(self):
        """集尘放置吸尘器取拖布不回洗"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.change_mop()
        return message
    
    def send_mop(self):
        """脏污识别擦拭"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.mop()
        return message
    
    # ========== 2号场地（巡航抓取清洁） ==========
    def send_patrol_pick_clean(self):
        """全流程"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.patrol_pick_clean()
        return message
    
    def send_patrol_pick(self):
        """识别抓取"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.patrol_pick()
        return message
    
    def send_patrol_clean(self):
        """巡航清洁"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.patrol_clean()
        return message
    
    def send_patrol_vac(self):
        """巡航吸尘"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.patrol_vac()
        return message
    
    def send_patrol_mop(self):
        """巡航擦拭"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.patrol_mop()
        return message
    
    # ========== 3号场地（复杂场景） ==========
    def send_complex_clean(self):
        """全流程"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.complex_clean()
        return message
    
    def send_complex_1_clean(self):
        """1号凳子处理"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.complex_1_clean()
        return message
    
    def send_complex_2_clean(self):
        """2号凳子处理"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.complex_2_clean()
        return message
    
    def send_complex_3_clean(self):
        """电视柜处理"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.complex_3_clean()
        return message
    
    # ========== 4号场地（大面清洁） ==========
    def send_whole_clean(self):
        """全流程"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.whole_clean()
        return message
    
    def send_whole_vac(self):
        """弓形吸尘"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.whole_vac()
        return message
    
    def send_whole_mop(self):
        """弓形擦地"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.whole_mop()
        return message
    
    def send_edge_mop(self):
        """延边擦地"""
        if not self.node.command_handler:
            return "❌ 命令处理器未初始化"
        success, message = self.node.command_handler.edge_mop()
        return message
    def send_arm_grasp_action(self, command="activate"):
        """发送机械臂抓取Action"""
        if not self.node:
            return "❌ 节点未就绪，无法发送机械臂动作"
        
        success, message = self.node.send_arm_grasp_action(command)
        return message