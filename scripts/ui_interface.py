# ui_interface.py
"""
用户界面构建
"""

import gradio as gr
import time
from .config import ROBOTS
from .robot_controller import RobotController
import threading
from .config import CAMERA_CONFIG


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
        self.camera_save = lambda: self.controller.save_camera_images("保存相机图像")        
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

                self._create_status_monitor_tab()                
            # 绑定事件
            self._bind_events(demo)
            
            # 页面加载时自动ping
            demo.load(self.do_ping, outputs=self.ping_status)
        
        return demo
    def _create_status_monitor_tab(self):
        """创建状态监控页面（包含相机控制）"""
        with gr.TabItem("📊 状态监控"):
            with gr.Tabs():
                # 子Tab 1: 机器人状态监控
                with gr.TabItem("🤖 机器人状态"):
                    self._create_simple_status_monitor_tab()
                
                # 子Tab 2: 相机控制
                with gr.TabItem("📷 相机监控"):
                    self._create_camera_control_subtab()
    def _create_simple_status_monitor_tab(self):
        """创建简化的状态监控页面"""
        with gr.TabItem("📊 状态监控"):
            gr.Markdown("# 📊 机器人状态监控")
            gr.Markdown("实时监控机器人各电机状态（位置 + 错误码）")
            
            # 控制面板
            with gr.Row():
                with gr.Column(scale=2):
                    btn_refresh_status = gr.Button("🔄 刷新状态", variant="primary")
                    btn_start_auto_refresh = gr.Button("▶️ 开始自动刷新", variant="primary")
                    btn_stop_auto_refresh = gr.Button("⏸️ 停止自动刷新", variant="secondary")
                
                with gr.Column(scale=1):
                    refresh_interval = gr.Slider(
                        minimum=1,
                        maximum=10,
                        value=3,
                        step=1,
                        label="自动刷新间隔(秒)"
                    )
            
            gr.Markdown("---")
            
            # 状态显示区域
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 📈 左臂状态")
                    # 创建电机状态显示组件
                    motor_status_components = []
                    for i in range(4):
                        motor_name = ["左升降", "左臂1", "左臂2", "左臂3"][i]
                        motor_status = gr.Textbox(f"{motor_name}: 等待数据", label=f"电机{i}", lines=1)
                        motor_status_components.append(motor_status)
                
                with gr.Column(scale=1):
                    gr.Markdown("### 📈 右臂状态")
                    for i in range(4, 8):
                        motor_name = ["右升降", "右臂1", "右臂2", "右臂3"][i-4]
                        motor_status = gr.Textbox(f"{motor_name}: 等待数据", label=f"电机{i}", lines=1)
                        motor_status_components.append(motor_status)
            
            gr.Markdown("---")
            
            # 状态摘要
            gr.Markdown("### 📊 状态摘要")
            status_summary = gr.Textbox(
                "点击'刷新状态'获取数据",
                label="状态信息",
                lines=3
            )
            
            # 局部变量，用于线程
            running = False
            auto_refresh_thread = None
            
            def refresh_status_func():
                """刷新所有电机状态"""
                if not self.controller.node:
                    error_msg = "❌ 节点未就绪"
                    return [error_msg] * 9
                
                try:
                    robot_state = self.controller.node.get_robot_state()
                    
                    if robot_state is None:
                        waiting_msg = "⏳ 等待数据..."
                        return [waiting_msg] * 9
                    
                    # 电机命名
                    motor_names = ["左升降", "左臂1", "左臂2", "左臂3", 
                                  "右升降", "右臂1", "右臂2", "右臂3"]
                    
                    statuses = []
                    
                    for i, motor in enumerate(robot_state.motor_state):
                        motor_name = motor_names[i] if i < len(motor_names) else f"电机{i}"
                        
                        if motor.error_id == 0:
                            status_icon = "🟢"
                            status_text = "正常"
                        elif motor.error_id == -1:
                            status_icon = "🟢"
                            status_text = "正常"
                        else:
                            status_icon = "🔴"
                            status_text = f"错误{motor.error_id}"
                        
                        status_str = f"{motor_name}: 位置={motor.q:.3f}  {status_icon} {status_text}"
                        statuses.append(status_str)
                    
                    # 生成摘要
                    total = len(robot_state.motor_state)
                    normal = sum(1 for m in robot_state.motor_state if m.error_id == 0)
                    errors = sum(1 for m in robot_state.motor_state if m.error_id != 0)
                    
                    summary = f"🕐 更新时间: {time.strftime('%H:%M:%S')}\n"
                    summary += f"📈 状态统计: 正常 {normal}/{total}, 故障 {errors}/{total}"
                    
                    if errors > 0:
                        error_list = [f"电机{i}:{m.error_id}" for i, m in enumerate(robot_state.motor_state) if m.error_id > 0 ]
                        summary += f"\n⚠️ 故障电机: {', '.join(error_list)}"
                    
                    # 返回所有状态
                    return statuses + [summary]
                    
                except Exception as e:
                    error_msg = f"❌ 获取状态异常: {str(e)[:50]}"
                    return [error_msg] * 9
            
            def start_auto_refresh_func(interval):
                """开始自动刷新"""
                nonlocal running, auto_refresh_thread
                running = True
                
                def auto_refresh_worker():
                    while running:
                        statuses = refresh_status_func()
                        if len(statuses) >= 9:
                            # 这里需要更新UI，但由于线程限制，我们只能通过Gradio的事件系统
                            # 实际应该通过状态变化触发，这里简化处理
                            pass
                        time.sleep(interval)
                
                auto_refresh_thread = threading.Thread(target=auto_refresh_worker, daemon=True)
                auto_refresh_thread.start()
                return "✅ 自动刷新已启动"
            
            def stop_auto_refresh_func():
                """停止自动刷新"""
                nonlocal running, auto_refresh_thread
                running = False
                if auto_refresh_thread:
                    auto_refresh_thread.join(timeout=1.0)
                return "⏸️ 自动刷新已停止"
            
            # 绑定事件
            btn_refresh_status.click(
                refresh_status_func,
                outputs=motor_status_components + [status_summary]
            )
            
            btn_start_auto_refresh.click(
                lambda interval: start_auto_refresh_func(interval),
                inputs=refresh_interval,
                outputs=status_summary
            )
            
            btn_stop_auto_refresh.click(
                stop_auto_refresh_func,
                outputs=status_summary
            )
        
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
            # 相机控制（新增部分）
            with gr.Row():
                with gr.Column(scale=1):
                    gr.Markdown("### 📷 相机控制")
                    
                    with gr.Row():
                        self.btn_camera_save = gr.Button("📸 保存相机图像", variant="primary", size="lg")
                    
                    self.camera_output = gr.Textbox("点击按钮保存相机当前图像", label="相机状态")
            
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
        # 相机控制
        self.btn_camera_save.click(self.camera_save, outputs=self.camera_output)
     
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
        self.refresh_robot_status = lambda: self.controller.get_simple_robot_status()
# ui_interface.py
# 在状态监控页面中使用相机类

    def _create_camera_control_subtab(self):
        """创建相机控制子页面"""
        
        with gr.Column():
            # 标题
            gr.Markdown("### 📷 相机图像捕获")
            
            with gr.Row():
                with gr.Column(scale=1):
                    # 控制面板
                    gr.Markdown("#### 🎛️ 控制面板")
                    
                    # 相机选择
                    camera_type = gr.Dropdown(
                        list(CAMERA_CONFIG["topics"].keys()),
                        value=CAMERA_CONFIG["default_camera"],
                        label="选择相机类型"
                    )
                    
                    # 显示设置
                    resize_width = gr.Slider(
                        320, 1280, CAMERA_CONFIG["default_width"], 160,
                        label="显示宽度(像素)"
                    )
                    
                    # 控制按钮
                    with gr.Row():
                        btn_capture = gr.Button("📸 捕获图像", variant="primary")
                        btn_capture_save = gr.Button("💾 捕获并保存", variant="secondary")
                    
                    # 状态显示
                    status_display = gr.Textbox(
                        f"默认相机: {CAMERA_CONFIG['default_camera']}\n"
                        f"超时: {CAMERA_CONFIG['default_timeout']}秒",
                        label="状态信息",
                        lines=3
                    )
                    
                    # 文件信息
                    file_display = gr.Textbox(
                        "",
                        label="保存信息",
                        lines=2
                    )
                    
                    # 历史按钮
                    with gr.Row():
                        btn_show_last = gr.Button("🔄 最后一张", variant="secondary")
                        btn_status = gr.Button("📊 状态信息", variant="secondary")
                
                with gr.Column(scale=2):
                    # 图片显示区域
                    gr.Markdown("#### 🖼️ 图像预览")
                    
                    # 使用Image组件
                    image_display = gr.Image(
                        label="捕获的图像",
                        height=400
                    )
                    
                    # 最后捕获信息
                    with gr.Row():
                        last_time = gr.Textbox(
                            "从未捕获",
                            label="最后捕获时间",
                            lines=1
                        )
                        last_camera = gr.Textbox(
                            "无",
                            label="相机类型",
                            lines=1
                        )
            
            # 事件处理函数
            def capture_image_handler(cam_type, width):
                """捕获图像处理"""
                message, base64_data, cv_image = self.controller.capture_camera_image(
                    camera_type=cam_type,
                    resize_width=width
                )
                
                # 获取最后捕获信息
                last_data = self.controller.get_last_camera_image()
                if last_data and last_data["timestamp"]:
                    from datetime import datetime
                    time_str = datetime.fromtimestamp(last_data["timestamp"]).strftime("%H:%M:%S")
                    cam_str = last_data["camera_type"] or "未知"
                else:
                    time_str = "从未捕获"
                    cam_str = "无"
                
                if base64_data:
                    # 转换为PIL图像
                    from PIL import Image
                    import io
                    import base64
                    
                    if base64_data.startswith("data:image/jpeg;base64,"):
                        base64_str = base64_data.split(",")[1]
                        image_bytes = base64.b64decode(base64_str)
                        image = Image.open(io.BytesIO(image_bytes))
                        return image, message, "", time_str, cam_str
                
                return None, message, "", time_str, cam_str
            
            def capture_and_save_handler(cam_type, width):
                """捕获并保存处理"""
                message, base64_data, cv_image, file_path = self.controller.capture_and_save_image(
                    camera_type=cam_type,
                    resize_width=width
                )
                
                # 获取最后捕获信息
                last_data = self.controller.get_last_camera_image()
                if last_data and last_data["timestamp"]:
                    from datetime import datetime
                    time_str = datetime.fromtimestamp(last_data["timestamp"]).strftime("%H:%M:%S")
                    cam_str = last_data["camera_type"] or "未知"
                else:
                    time_str = "从未捕获"
                    cam_str = "无"
                
                file_info = f"保存到: {file_path}" if file_path else "未保存"
                
                if base64_data:
                    from PIL import Image
                    import io
                    import base64
                    
                    if base64_data.startswith("data:image/jpeg;base64,"):
                        base64_str = base64_data.split(",")[1]
                        image_bytes = base64.b64decode(base64_str)
                        image = Image.open(io.BytesIO(image_bytes))
                        return image, message, file_info, time_str, cam_str
                
                return None, message, file_info, time_str, cam_str
            
            def show_last_image_handler():
                """显示最后一张图片"""
                last_data = self.controller.get_last_camera_image()
                
                if last_data and last_data.get("base64_data"):
                    from PIL import Image
                    import io
                    import base64
                    from datetime import datetime
                    
                    base64_data = last_data["base64_data"]
                    if base64_data.startswith("data:image/jpeg;base64,"):
                        base64_str = base64_data.split(",")[1]
                        image_bytes = base64.b64decode(base64_str)
                        image = Image.open(io.BytesIO(image_bytes))
                        
                        time_str = datetime.fromtimestamp(last_data["timestamp"]).strftime("%H:%M:%S")
                        cam_str = last_data["camera_type"] or "未知"
                        
                        return image, f"显示最后捕获的图像", "", time_str, cam_str
                
                return None, "没有可用的历史图像", "", "从未捕获", "无"
            
            def show_status_handler():
                """显示状态信息"""
                status = self.controller.get_camera_status()
                
                if status.get("initialized"):
                    last_capture = status.get("last_capture", {})
                    stats = status.get("stats", {})
                    
                    message = (
                        f"📊 相机状态:\n"
                        f"已初始化: ✅\n"
                        f"捕获中: {'是' if status.get('is_capturing') else '否'}\n"
                        f"总捕获: {stats.get('total_captures', 0)}\n"
                        f"成功: {stats.get('successful_captures', 0)}\n"
                        f"最后捕获: {last_capture.get('time_str', '从未')}"
                    )
                    
                    time_str = last_capture.get("time_str", "从未捕获")
                    cam_str = last_capture.get("camera_type", "无")
                    
                    return None, message, "", time_str, cam_str
                else:
                    return None, status.get("message", "未知错误"), "", "未知", "未知"
            
            # 绑定事件
            btn_capture.click(
                capture_image_handler,
                inputs=[camera_type, resize_width],
                outputs=[image_display, status_display, file_display, last_time, last_camera]
            )
            
            btn_capture_save.click(
                capture_and_save_handler,
                inputs=[camera_type, resize_width],
                outputs=[image_display, status_display, file_display, last_time, last_camera]
            )
            
            btn_show_last.click(
                show_last_image_handler,
                outputs=[image_display, status_display, file_display, last_time, last_camera]
            )
            
            btn_status.click(
                show_status_handler,
                outputs=[image_display, status_display, file_display, last_time, last_camera]
            )