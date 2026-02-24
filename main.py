# main_multi_process.py
"""
多进程方案 - 每个用户独立进程，完全隔离
"""

import multiprocessing
import time
import webbrowser
import os
import sys
from scripts.ui_interface import RobotUI
import gradio as gr


def run_user_instance(user_config):
    """在一个独立的进程中运行用户实例"""
    user_id = user_config["id"]
    port = user_config["port"]
    domain_id = user_config["domain_id"]
    host_ip=user_config["ip"]
    print(f"[进程{user_id}] 🚀 启动用户{user_id} (端口: {port}, DOMAIN_ID: {domain_id})...")
    
    # 设置环境变量 - 每个进程独立
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    
    try:
        # 在独立进程中创建UI实例
        robot_ui = RobotUI(id=domain_id,ip=host_ip)
        demo = robot_ui.create_ui()
        demo.title = f"机器人控制 - 用户{user_id} (DOMAIN:{domain_id})"
        
        print(f"[进程{user_id}] ✅ 实例已创建，监听端口: {port}")
        
        # 启动界面
        demo.launch(
            server_name="0.0.0.0", 
            server_port=port,
            share=False,
            quiet=True
        )
        
    except Exception as e:
        print(f"[进程{user_id}] ❌ 启动失败: {e}")
        sys.exit(1)


def main():
    """主函数 - 使用进程启动4个独立实例"""
    print("=" * 50)
    print("🤖 多用户机器人控制系统 (多进程版)")
    print("=" * 50)
    
    # 定义4个用户的配置，每个绑定不同的DOMAIN_ID
    users_config = [
        # {"id": 6,  "port": 7856, "name": "用户1", "domain_id": 70,"ip":"192.168.2.198"},        
        # {"id": 11, "port": 7861, "name": "用户11", "domain_id": 110,"ip":"192.168.2.195"},
        {"id": 12, "port": 7862, "name": "用户12", "domain_id": 120,"ip":"192.168.2.70"},
        # {"id": 13, "port": 7863, "name": "用户13", "domain_id": 130,"ip":"192.168.2.221"},
        # # # {"id": 14, "port": 7864, "name": "用户14", "domain_id": 140,"ip":"192.168.2.238"},
        # {"id": 15,  "port": 7865, "name": "用户15", "domain_id": 150,"ip":"192.168.2.246"},
        {"id": 16,  "port": 7866, "name": "用户16", "domain_id": 160,"ip":"192.168.2.123"},  
        # {"id": 17,  "port": 7867, "name": "用户17", "domain_id": 170,"ip":"192.168.2.199"},    
    ]
    
    print(f"📱 准备启动 {len(users_config)} 个独立进程...")
    print("每个进程独立的DOMAIN_ID:")
    for user in users_config:
        print(f"  👤 {user['name']}: 端口 {user['port']}, DOMAIN_ID {user['domain_id']}")
    print("-" * 50)
    
    # 创建并启动所有进程
    processes = []
    
    for user in users_config:
        process = multiprocessing.Process(
            target=run_user_instance,
            args=(user,),
            name=f"User-{user['id']}"
        )
        process.start()
        processes.append(process)
        
        # 等待一会儿确保端口不冲突
        time.sleep(2)
    
    print(f"\n✅ 所有进程已启动!")
    print("🌐 访问地址:")
    
    for user in users_config:
        url = f"http://localhost:{user['port']}"
        print(f"   👤 {user['name']} (DOMAIN:{user['domain_id']}): {url}")
    
    print("\n" + "=" * 50)
    print("🔄 系统正在运行中...")
    print("🛑 按 Ctrl+C 停止所有进程")
    print("=" * 50)
    
    # 保持主进程运行
    try:
        while True:
            # 检查进程状态
            alive_processes = [p for p in processes if p.is_alive()]
            
            if not alive_processes:
                print("⚠️ 所有进程已停止")
                break
            
            # 如果有进程死亡，重新启动
            for i, process in enumerate(processes):
                if not process.is_alive():
                    print(f"⚠️ 用户{i+1} 进程已停止，正在重启...")
                    user = users_config[i]
                    new_process = multiprocessing.Process(
                        target=run_user_instance,
                        args=(user,),
                        name=f"User-{user['id']}-restart"
                    )
                    new_process.start()
                    processes[i] = new_process
                    time.sleep(2)
            
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\n🛑 正在停止所有进程...")
        
        # 停止所有进程
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=5)
    
    finally:
        print("👋 系统已停止")


if __name__ == "__main__":
    # 在Windows上需要使用这行
    multiprocessing.freeze_support()
    main()