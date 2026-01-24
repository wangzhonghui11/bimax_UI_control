# ui_interface.py
"""
用户界面构建
"""

import gradio as gr
import time
from config import ROBOTS
from robot_controller import RobotController


class RobotUI:
    def __init__(self):
        self.controller = RobotController()
        self._setup_event_handlers()
    
    def _setup_event_handlers(self):
        """设置事件处理器"""
        # 基础控制函数
        self.do_ping = lambda: self.controller.ping_test()
        
        def switch(robot):
            result = self.controller.switch_robot(robot)
            return result, robot
        
        self.switch = switch
        
        # 电磁铁控制
        self.magnet_on = lambda: self.controller.send_magnet_command(1, 1, "电磁铁充磁")
        self.magnet_off = lambda: self.controller.send_magnet_command(0, 0, "电磁铁退磁")
        
        # 吸尘器控制
        self.catcher_on = lambda: self.controller.send_catcher_command(1, 1, "吸尘器开启")
        self.catcher_off = lambda: self.controller.send_catcher_command(1, 0, "吸尘器关闭")
        
        # 拖布控制
        self.mop_on = lambda: self.controller.send_mop_command(2000, 1, "拖布开启")
        self.mop_off = lambda: self.controller.send_mop_command(2000, 0, "拖布关闭")
        
        # 夹爪控制
        self.jaw_close = lambda: self.controller.send_jaw_command("close", "关夹爪")
        self.jaw_open = lambda: self.controller.send_jaw_command("open", "开夹爪")
        
        # 机械臂控制
        self.arm_home = lambda: self.controller.send_arm_home("机械臂回零")
        self.arm_fold = lambda: self.controller.send_arm_fold("机械臂收臂")
        
        # 电机故障重置
        self.motor_reset = lambda: self.controller.reset_motor_faults()
        
        # 移动控制
        self.move_forward = lambda: self.controller.send_cmd("forward")
        self.move_backward = lambda: self.controller.send_cmd("backward")
        self.move_left = lambda: self.controller.send_cmd("left")
        self.move_right = lambda: self.controller.send_cmd("right")
        self.move_stop = lambda: self.controller.send_cmd("stop")
        
        # 机械臂动作
        self.action1 = lambda: self.controller.send_action_command("move_grasp")
        self.action2 = lambda: self.controller.send_action_command("only_grasp")
        self.action3 = lambda: self.controller.send_action_command("grasp_slipper")
        self.action4 = lambda: self.controller.send_action_command("putinto_basket")
        self.action5 = lambda: self.controller.send_action_command("putinto_trash_bin")
        self.action6 = lambda: self.controller.send_action_command("put_down")
        
        # 基站控制
        self.wash_on = lambda: self.controller.call_station_service("wash", True, "清洗功能")
        self.wash_off = lambda: self.controller.call_station_service("wash", False, "清洗功能")
        self.dust_on = lambda: self.controller.call_station_service("dust", True, "吸尘功能")
        self.dust_off = lambda: self.controller.call_station_service("dust", False, "吸尘功能")
        self.dry_on = lambda: self.controller.call_station_service("dry", True, "干燥功能")
        self.dry_off = lambda: self.controller.call_station_service("dry", False, "干燥功能")
    
    def create_ui(self):
        """创建用户界面"""
        with gr.Blocks() as demo:
            gr.Markdown("# 🤖 机器人遥控")
            
            with gr.Tabs():
                # 基础控制页面
                self._create_basic_control_tab()
                
                # 硬件控制检测页面
                self._create_hardware_control_tab()
                
                # 基站控制页面
                self._create_station_control_tab()
            
            # 绑定事件
            self._bind_events(demo)
            
            # 页面加载时自动ping
            demo.load(self.do_ping, outputs=self.ping_status)
        
        return demo
    
    def _create_basic_control_tab(self):
        """创建基础控制页面"""
        with gr.TabItem("🎮 基础控制"):
            # Ping测试
            with gr.Row():
                self.ping_btn = gr.Button("📡 Ping测试", variant="secondary")
                self.ping_status = gr.Textbox("点击测试", label="网络状态")
            
            # 机器人选择
            with gr.Row():
                self.robot_select = gr.Dropdown(
                    list(ROBOTS.keys()),
                    value="ROBOT0 (DOMAIN=0)",
                    label="选择机器人"
                )
                self.switch_btn = gr.Button("切换", variant="primary")
            
            self.status = gr.Textbox("✅ 已连接", label="状态")
            
            gr.Markdown("---")
    
    def _create_hardware_control_tab(self):
        """创建硬件控制检测页面"""
        with gr.TabItem("🔧 硬件控制检测"):
            # 控制面板
            with gr.Column():
                gr.Markdown("### 🔋 轮组控制")
                with gr.Row():
                    self.btn_w = gr.Button("⬆️ 前进", size="lg")
                with gr.Row():
                    self.btn_a = gr.Button("⬅️ 左转")
                    self.btn_s = gr.Button("⏹️ 停止", variant="secondary")
                    self.btn_d = gr.Button("➡️ 右转")
                with gr.Row():
                    self.btn_x = gr.Button("⬇️ 后退", size="lg")
            
            self.cmd_output = gr.Textbox("准备就绪", label="命令")
            
            # 电磁铁控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🔋 电磁铁控制")
                    
                    with gr.Row():
                        self.btn_magnet_on = gr.Button("🔋 充磁", variant="primary")
                        self.btn_magnet_off = gr.Button("🔌 退磁", variant="secondary")
                    
                    self.magnet_output = gr.Textbox("准备就绪", label="电磁铁状态")
            
            gr.Markdown("---")
            
            # 吸尘器控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🌀 吸尘器控制")
                    
                    with gr.Row():
                        self.btn_catcher_on = gr.Button("🌀 吸尘器开", variant="primary")
                        self.btn_catcher_off = gr.Button("🌀 吸尘器关", variant="secondary")
                    
                    self.catcher_output = gr.Textbox("准备就绪", label="吸尘器状态")
            
            gr.Markdown("---")
            
            # 拖布控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🧹 拖布控制")
                    
                    with gr.Row():
                        self.btn_mop_on = gr.Button("🧹 拖布开", variant="primary")
                        self.btn_mop_off = gr.Button("🧹 拖布关", variant="secondary")
                    
                    self.mop_output = gr.Textbox("准备就绪", label="拖布状态")
            
            gr.Markdown("---")
            
            # 夹爪控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🦾 夹爪控制")
                    
                    with gr.Row():
                        self.btn_jaw_close = gr.Button("🤏 关夹爪", variant="primary")
                        self.btn_jaw_open = gr.Button("🦾 开夹爪", variant="secondary")
                    
                    self.jaw_output = gr.Textbox("准备就绪", label="夹爪状态")
            
            gr.Markdown("---")
            
            # 机械臂控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🤖 机械臂控制")
                    
                    with gr.Row():
                        self.btn_arm_home = gr.Button("🏠 臂回零", variant="primary")
                        self.btn_arm_fold = gr.Button("📦 左右收臂", variant="secondary")
                    
                    self.arm_output = gr.Textbox("准备就绪", label="机械臂状态")
            
            gr.Markdown("---")
            
            # 电机故障处理
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### ⚙️ 电机故障处理")
                    self.btn_motor_reset = gr.Button("🔄 重置并初始化电机", variant="primary")
                
                self.motor_output = gr.Textbox("点击按钮重置电机错误并初始化", label="电机状态")
            
            gr.Markdown("---")
            
            # 机械臂动作控制
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("# 🤖 机械臂动作控制")
                    
                    with gr.Row():
                        self.btn1 = gr.Button("移动并抓取", variant="primary")
                        self.btn2 = gr.Button("仅抓取", variant="primary")
                        self.btn3 = gr.Button("抓拖鞋", variant="primary")
                    
                    with gr.Row():
                        self.btn4 = gr.Button("放入篮子", variant="primary")
                        self.btn5 = gr.Button("放入垃圾桶", variant="primary")
                        self.btn6 = gr.Button("放下", variant="primary")
                    
                    self.grasp_output = gr.Textbox("准备就绪", label="状态")
    
    def _create_station_control_tab(self):
        """创建基站控制页面"""
        with gr.TabItem("🏠 基站控制"):
            gr.Markdown("### 测试基站各项功能")
            
            # 清洗功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🚿 清洗功能")
                    
                    with gr.Row():
                        self.btn_wash_on = gr.Button("💦 开启清洗", variant="primary", size="lg")
                        self.btn_wash_off = gr.Button("⏹️ 关闭清洗", variant="secondary", size="lg")
                    
                    self.wash_output = gr.Textbox("准备测试清洗功能", label="清洗状态")
            
            gr.Markdown("---")
            
            # 吸尘功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🌪️ 吸尘功能")
                    
                    with gr.Row():
                        self.btn_dust_on = gr.Button("🌀 开启吸尘", variant="primary", size="lg")
                        self.btn_dust_off = gr.Button("⏹️ 关闭吸尘", variant="secondary", size="lg")
                    
                    self.dust_output = gr.Textbox("准备测试吸尘功能", label="吸尘状态")
            
            gr.Markdown("---")
            
            # 干燥功能
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 🔥 干燥功能")
                    
                    with gr.Row():
                        self.btn_dry_on = gr.Button("🔥 开启干燥", variant="primary", size="lg")
                        self.btn_dry_off = gr.Button("⏹️ 关闭干燥", variant="secondary", size="lg")
                    
                    self.dry_output = gr.Textbox("准备测试干燥功能", label="干燥状态")
    
    def _bind_events(self, demo):
        """绑定所有事件"""
        # 基础控制页面事件
        self.ping_btn.click(self.do_ping, outputs=self.ping_status)
        
        self.switch_btn.click(
            self.switch,
            inputs=self.robot_select,
            outputs=[self.status, self.robot_select]
        )
        
        self.robot_select.change(
            lambda x: f"准备切换到: {x}",
            inputs=self.robot_select,
            outputs=self.status
        )
        
        # 移动控制事件
        self.btn_w.click(self.move_forward, outputs=self.cmd_output)
        self.btn_x.click(self.move_backward, outputs=self.cmd_output)
        self.btn_a.click(self.move_left, outputs=self.cmd_output)
        self.btn_d.click(self.move_right, outputs=self.cmd_output)
        self.btn_s.click(self.move_stop, outputs=self.cmd_output)
        
        # 电磁铁控制
        self.btn_magnet_on.click(self.magnet_on, outputs=self.magnet_output)
        self.btn_magnet_off.click(self.magnet_off, outputs=self.magnet_output)
        
        # 吸尘器控制
        self.btn_catcher_on.click(self.catcher_on, outputs=self.catcher_output)
        self.btn_catcher_off.click(self.catcher_off, outputs=self.catcher_output)
        
        # 拖布控制
        self.btn_mop_on.click(self.mop_on, outputs=self.mop_output)
        self.btn_mop_off.click(self.mop_off, outputs=self.mop_output)
        
        # 夹爪控制
        self.btn_jaw_close.click(self.jaw_close, outputs=self.jaw_output)
        self.btn_jaw_open.click(self.jaw_open, outputs=self.jaw_output)
        
        # 机械臂控制
        self.btn_arm_home.click(self.arm_home, outputs=self.arm_output)
        self.btn_arm_fold.click(self.arm_fold, outputs=self.arm_output)
        
        # 电机故障重置
        self.btn_motor_reset.click(self.motor_reset, outputs=self.motor_output)
        
        # 机械臂动作控制
        self.btn1.click(self.action1, outputs=self.grasp_output)
        self.btn2.click(self.action2, outputs=self.grasp_output)
        self.btn3.click(self.action3, outputs=self.grasp_output)
        self.btn4.click(self.action4, outputs=self.grasp_output)
        self.btn5.click(self.action5, outputs=self.grasp_output)
        self.btn6.click(self.action6, outputs=self.grasp_output)
        
        # 基站控制
        self.btn_wash_on.click(self.wash_on, outputs=self.wash_output)
        self.btn_wash_off.click(self.wash_off, outputs=self.wash_output)
        self.btn_dust_on.click(self.dust_on, outputs=self.dust_output)
        self.btn_dust_off.click(self.dust_off, outputs=self.dust_output)
        self.btn_dry_on.click(self.dry_on, outputs=self.dry_output)
        self.btn_dry_off.click(self.dry_off, outputs=self.dry_output)