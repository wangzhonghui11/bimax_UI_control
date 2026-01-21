import paramiko
import time
import re
import socket
class ssh_client():
    def __init__(self):
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.channel = None

    def connect(self, host, username, password) -> bool:
        try:
            self.client.connect(host, username=username, password=password)
            self.channel = self.client.invoke_shell()
            print("连接成功")
            return True
        except paramiko.AuthenticationException:
            print("认证失败")
        except paramiko.SSHException as e:
            print(f"SSH连接失败: {e}")
        except socket.timeout:
            print("连接超时")
        except Exception as e:
            print(f"其他错误: {e}")
        self.client.close()
        return False

    def close(self):
        self.channel.send('exit\n')
        self.client.close()
        self.channel = None

    def is_connected(self):
      """检查 SSH 连接是否仍然活动"""
      transport = self.client.get_transport()
      return transport is not None and transport.is_active()

    def run_shell(self, command):
        # 执行shell命令,持续交互
        
        self.running = True
        self.channel.send(f'{command}\n')    # 执行命令
        result = ''                     # 存放结果
        while True:                     # 有可能有多行数据 用循环
            time.sleep(0.1)
            res = self.channel.recv(65535).decode('utf8')
            result += res
            if res.endswith('# ') or res.endswith('$ '):    # 终止条件
                break
        result = re.sub('\x1b.*?m', '', result)     # 移除\xblah[0m这些
        self.running = False
        print(result)
        return  result.strip('\n')  # 移除


        
    
