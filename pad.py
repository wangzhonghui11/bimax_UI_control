import gradio as gr
from ssh import ssh_client

ssh = ssh_client()
user = "bimax"
password = "123"
ip = "192.168.2.196"
ros2_command = "ros2 action send_goal /function/arm/grasp bimax_msgs/action/BimaxFunction \"{{command: '{}'}}\""

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

with gr.Blocks() as demo:
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

    connect_button.click(
        fn=connect
    )

    move_grasp_button.click(
        fn=move_grasp
    )
    only_grasp_button.click(
        fn=only_grasp
    )
    grasp_slipper_button.click(
        fn=grasp_slipper
    )
    putinto_basket_button.click(
        fn=putinto_basket
    )
    putinto_trash_bin_button.click(
        fn=putinto_trash_bin
    )
    putdown_button.click(
        fn=putdown
    )
    test_button.click(
        fn=interrupt
    )


    timer = gr.Timer(value=0.5)  # 每0.5秒检测一次是否连接
    timer.tick(
        fn=update_connection,
        outputs=connection_status
    )

    

demo.launch(server_name="0.0.0.0")

