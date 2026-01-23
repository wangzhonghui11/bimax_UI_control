import gradio as gr
from ssh import ssh_client

ssh = ssh_client()
user = "bimax"
password = "123"
ip = "192.168.2.196"
ros2_command = "ros2 action send_goal /function/arm/grasp bimax_msgs/action/BimaxFunction \"{{command: '{}'}}\""
vel_command = "ros2 topic pub /function/vel bimax_msgs/msg/BimaxFunction \"{{command: '{}'}}\""

def update_selection(dropdown):
    global ip
    ip = dropdown
    return ip

def update_connection():
    global ssh
    success = ssh.is_connected()
    if success:
        message = "连接成功"
    else:   
        message = "连接失败"

    return message

def connect():
    global ssh
    ssh.connect(ip, user, password)

def move_grasp():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("move&grasp_all"))

def only_grasp():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("onlygrasp_all"))

def grasp_slipper():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("grasp_slipper"))

def putinto_basket():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("putinto_basket"))

def putinto_trash_bin():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("putinto_trash_bin"))

def putdown():
    global ssh
    if ssh.is_connected():
        ssh.run_shell(ros2_command.format("put_down"))

def interrupt():
    global ssh
    if ssh.is_connected():
        ssh.run_shell("\x03")

def wheel_forward():
    global ssh
    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    if ssh.is_connected():
        ssh.run_shell(cmd)

def wheel_backward():
    global ssh
    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    if ssh.is_connected():
        ssh.run_shell(cmd)

def wheel_right():
    global ssh
    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1745}}'"
    if ssh.is_connected():
        ssh.run_shell(cmd)

def wheel_left():
    global ssh
    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1745}}'"
    if ssh.is_connected():
        ssh.run_shell(cmd)

def wheel_stop():
    global ssh
    cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
    if ssh.is_connected():
        ssh.run_shell(cmd)

# 升降电机控制


with gr.Blocks(css="""
      .square-button {
          width: 100px !important;
          height: 100px !important;
          min-width: 100px !important;
          max-width: 100px !important;
          aspect-ratio: 1 !important;
          flex: none !important;
      }
      .centered-row {
          display: flex !important;
          justify-content: center !important;
          width: 100% !important;
      }
  """) as demo:
    gr.Markdown("<h2 align='center'> 连接到机器人 </h2>")
    # 下拉框选择机器
    with gr.Row():
        dropdown = gr.Dropdown(
            choices=[
                ("ROBOT4", "192.168.2.196"),
                ("ROBOT5", "192.168.2.197"),
                ("ROBOT6", "192.168.2.198"),
            ],
            value="ROBOT4",
            info="选择要连接的机器",
            show_label=False
        )

        ip_output = gr.Textbox(info="当前选择的机器IP", show_label=False)

        dropdown.change(
            fn=update_selection,
            inputs=dropdown,
            outputs=ip_output
        )
    with gr.Row():
        connect_button = gr.Button("连接", size="lg", variant="primary")
        connection_status = gr.Textbox(show_label=False)

    gr.Markdown("---")
    gr.Markdown("<h2 align='center'> 方向盘 </h2>")
    gr.Markdown("---")
    with gr.Column():
        with gr.Row(elem_classes="centered-row"):
            forward_button = gr.Button("", icon="icons/forward.png", elem_classes="square-button")
        with gr.Row(elem_classes="centered-row"):
            left_button = gr.Button("", icon="icons/left.png", elem_classes="square-button")
            stop_button = gr.Button("", icon="icons/stop.png", elem_classes="square-button")
            right_button = gr.Button("", icon="icons/right.png", elem_classes="square-button")
        with gr.Row(elem_classes="centered-row"):
            backward_button = gr.Button("", icon="icons/backward.png", elem_classes="square-button")

    """
    gr.Markdown("---")
    gr.Markdown("<h2 align='center'>  左 《《《 升降电机 》》》 右 </h2>")
    gr.Markdown("---")
    with gr.Column():
        with gr.Row(elem_classes="centered-row"):
            up_left_button = gr.Button("", icon="icons/forward.png", elem_classes="square-button")
            up_right_button = gr.Button("", icon="icons/forward.png", elem_classes="square-button")
        with gr.Row(elem_classes="centered-row"):
            stop_left_button = gr.Button("", icon="icons/stop.png", elem_classes="square-button")
            stop_right_button = gr.Button("", icon="icons/stop.png", elem_classes="square-button")
        with gr.Row(elem_classes="centered-row"):
            down_left_button = gr.Button("", icon="icons/backward.png", elem_classes="square-button")
            down_right_button = gr.Button("", icon="icons/backward.png", elem_classes="square-button")
    """

    gr.Markdown("---")
    gr.Markdown("<h2 align='center'> 抓取动作 </h2>")
    gr.Markdown("---")

    with gr.Column():
        with gr.Row():
            move_grasp_button = gr.Button("move&grasp", size="lg", variant="primary")
            only_grasp_button = gr.Button("onlygrasp", size="lg", variant="primary")
            grasp_slipper_button = gr.Button("grasp_slipper", size="lg", variant="primary")
        with gr.Row():
            putinto_basket_button = gr.Button("putinto_basket", size="lg", variant="primary")
            putinto_trash_bin_button = gr.Button("putinto_trash_bin", size="lg", variant="primary")
            putdown_button = gr.Button("putdown", size="lg", variant="primary")
            
        test_button = gr.Button("终止 CONTROL+C", size="lg", variant="primary")

    """****************************************
    *            按钮绑定函数                   *
    ****************************************"""

    connect_button.click(fn=connect)

    move_grasp_button.click(fn=move_grasp)
    only_grasp_button.click(fn=only_grasp)
    grasp_slipper_button.click(fn=grasp_slipper)
    putinto_basket_button.click(fn=putinto_basket)
    putinto_trash_bin_button.click(fn=putinto_trash_bin)
    putdown_button.click(fn=putdown)
    test_button.click(fn=interrupt)
    forward_button.click(fn=wheel_forward)
    backward_button.click(fn=wheel_backward)
    left_button.click(fn=wheel_left)
    right_button.click(fn=wheel_right)
    stop_button.click(fn=wheel_stop)

    timer = gr.Timer(value=0.5)  # 每0.5秒检测一次是否连接
    timer.tick(
        fn=update_connection,
        outputs=connection_status
    )

    

demo.launch(server_name="0.0.0.0")

