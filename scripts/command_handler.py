# scripts/command_handler.py
"""
命令处理器 - 专门处理所有/user/command命令
"""
import time
from std_msgs.msg import String
from .config import USER_COMMANDS, TOPIC_NAMES

class CommandHandler:
    def __init__(self, node):
        """
        初始化命令处理器
        
        Args:
            node: ROS2节点实例
        """
        self.node = node
        self._init_publisher()
    
    def _init_publisher(self):
        """初始化命令发布者"""
        try:
            self.command_publisher = self.node.create_publisher(
                String,
                TOPIC_NAMES["user_command"],
                10
            )
            self.node.get_logger().info(f"✅ 命令发布者已创建: {TOPIC_NAMES['user_command']}")
        except Exception as e:
            self.node.get_logger().error(f"❌ 创建命令发布者失败: {str(e)}")
    
    def send_command(self, command_key, description=""):
        """
        发送命令到/user/command话题
        
        Args:
            command_key: 命令键，对应USER_COMMANDS中的键
            description: 命令描述，用于日志显示
        
        Returns:
            tuple: (是否成功, 消息)
        """
        if not self.node or not hasattr(self, 'command_publisher'):
            return False, "❌ 命令发布者未初始化"
        
        try:
            # 获取命令值
            if command_key not in USER_COMMANDS:
                return False, f"❌ 无效的命令键: {command_key}"
            
            command_value = USER_COMMANDS[command_key]
            
            # 创建并发布消息
            msg = String()
            msg.data = command_value
            
            self.command_publisher.publish(msg)
            
            timestamp = time.strftime("%H:%M:%S")
            success_msg = f"✅ [{timestamp}] 已发送: {description} ({command_value})"
            self.node.get_logger().info(success_msg)
            
            return True, success_msg
            
        except Exception as e:
            error_msg = f"❌ 发送命令失败: {str(e)[:50]}"
            self.node.get_logger().error(error_msg)
            return False, error_msg
    
    # ========== 通用功能 ==========
    def cancel(self):
        """停止动作"""
        return self.send_command("CANCEL", "停止动作")
    
    def back(self):
        """回到场地中央"""
        return self.send_command("BACK", "回到场地中央")
    
    # 工具取放
    def vac_place(self):
        """放吸尘器"""
        return self.send_command("VAC_PLACE", "放吸尘器")
    
    def vac_take(self):
        """取吸尘器"""
        return self.send_command("VAC_TAKE", "取吸尘器")
    
    def mop_place(self):
        """放拖布"""
        return self.send_command("MOP_PLACE", "放拖布")
    
    def mop_take(self):
        """取拖布"""
        return self.send_command("MOP_TAKE", "取拖布")
    
    def mop_clean(self):
        """洗拖布"""
        return self.send_command("MOP_CLEAN", "洗拖布")
    
    # ========== 1号场地（主持人） ==========
    def show(self):
        """开场动作展示"""
        return self.send_command("SHOW", "开场动作展示")
    
    def pick(self):
        """抓物品放置"""
        return self.send_command("PICK", "抓物品放置")
    
    def pick_slipper(self):
        """抓拖鞋放置"""
        return self.send_command("PICK_SLIPPER", "抓拖鞋放置")
    
    def vac(self):
        """识别吸尘"""
        return self.send_command("VAC", "识别吸尘")
    
    def change_mop(self):
        """集尘放置吸尘器取拖布不回洗"""
        return self.send_command("CHANGE_MOP", "集尘放置吸尘器取拖布不回洗")
    
    def mop(self):
        """脏污识别擦拭"""
        return self.send_command("MOP", "脏污识别擦拭")
    
    # ========== 2号场地（巡航抓取清洁） ==========
    def patrol_pick_clean(self):
        """全流程"""
        return self.send_command("PATROL_PICK_CLEAN", "巡航抓取清洁全流程")
    
    def patrol_pick(self):
        """识别抓取"""
        return self.send_command("PATROL_PICK", "识别抓取")
    
    def patrol_clean(self):
        """巡航清洁"""
        return self.send_command("PATROL_CLEAN", "巡航清洁")
    
    def patrol_vac(self):
        """巡航吸尘"""
        return self.send_command("PATROL_VAC", "巡航吸尘")
    
    def patrol_mop(self):
        """巡航擦拭"""
        return self.send_command("PATROL_MOP", "巡航擦拭")
    
    # ========== 3号场地（复杂场景） ==========
    def complex_clean(self):
        """全流程"""
        return self.send_command("COMPLEX_CLEAN", "复杂场景全流程")
    
    def complex_1_clean(self):
        """1号凳子处理"""
        return self.send_command("COMPLEX_1_CLEAN", "1号凳子处理")
    
    def complex_2_clean(self):
        """2号凳子处理"""
        return self.send_command("COMPLEX_2_CLEAN", "2号凳子处理")
    
    def complex_3_clean(self):
        """电视柜处理"""
        return self.send_command("COMPLEX_3_CLEAN", "电视柜处理")
    
    # ========== 4号场地（大面清洁） ==========
    def whole_clean(self):
        """全流程"""
        return self.send_command("WHOLE_CLEAN", "大面清洁全流程")
    
    def whole_vac(self):
        """弓形吸尘"""
        return self.send_command("WHOLE_VAC", "弓形吸尘")
    
    def whole_mop(self):
        """弓形擦地"""
        return self.send_command("WHOLE_MOP", "弓形擦地")
    
    def edge_mop(self):
        """延边擦地"""
        return self.send_command("EDGE_MOP", "延边擦地")
    