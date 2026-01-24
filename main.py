# main.py
"""
主程序入口
"""

from ui_interface import RobotUI

def main():
    """主函数"""
    # 创建UI实例
    robot_ui = RobotUI()
    
    # 创建界面
    demo = robot_ui.create_ui()
    
    # 启动界面
    demo.launch(
        server_name="0.0.0.0", 
        server_port=7860,
        share=False
    )

if __name__ == "__main__":
    main()