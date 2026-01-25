# camera_handler.py
"""
相机处理类 - 单独的相机功能模块
"""

import cv2
import numpy as np
import os
import time
import base64
import threading
import queue
from datetime import datetime
from sensor_msgs.msg import Image
from .config import CAMERA_CONFIG


class CameraHandler:
    """相机处理类 - 处理图像捕获、转换、保存和显示"""
    
    def __init__(self, robot_node):
        """
        初始化相机处理器
        
        参数:
            robot_node: ROS2节点实例
        """
        self.node = robot_node
        self.logger = robot_node.get_logger()
        
        # 从配置加载
        self.config = CAMERA_CONFIG
        self.camera_topics = self.config["topics"]
        
        # 线程安全
        self.lock = threading.Lock()
        self.capture_queue = queue.Queue(maxsize=self.config["max_queue_size"])
        
        # 捕获状态
        self.is_capturing = False
        self.image_sub = None
        self.current_topic = None
        
        # 历史记录
        self.history = []
        self.max_history = 20
        
        # 最后捕获的数据
        self.last_capture = {
            "success": False,
            "message": "",
            "cv_image": None,
            "base64_data": None,
            "timestamp": None,
            "camera_type": None,
            "topic": None,
            "file_path": None,
        }
        
        # 统计信息
        self.stats = {
            "total_captures": 0,
            "successful_captures": 0,
            "failed_captures": 0,
            "last_capture_time": None,
        }
        
        self.logger.info("📷 相机处理器初始化完成")
        self.logger.info(f"📷 可用相机: {list(self.camera_topics.keys())}")
    
    def capture_single_image(self, camera_type=None, timeout=None, resize_width=None):
        """
        捕获单张图像
        
        返回: (success, message, base64_data, cv_image)
        """
        # 使用默认值
        camera_type = camera_type or self.config["default_camera"]
        timeout = timeout or self.config["default_timeout"]
        resize_width = resize_width or self.config["default_width"]
        
        # 检查相机类型
        if camera_type not in self.camera_topics:
            return False, f"未知相机类型: {camera_type}", None, None
        
        topic = self.camera_topics[camera_type]
        
        with self.lock:
            if self.is_capturing:
                return False, "正在捕获中，请稍后重试", None, None
            self.is_capturing = True
            self.current_topic = topic
        
        try:
            # 清空队列
            self._clear_queue()
            
            # 创建临时订阅
            self.image_sub = self.node.create_subscription(
                Image,
                topic,
                self._image_callback,
                10
            )
            
            self.logger.info(f"📷 开始捕获 {camera_type} 相机图像")
            self.logger.info(f"📷 话题: {topic}")
            
            # 等待图像
            start_time = time.time()
            cv_image = None
            
            while self.is_capturing and (time.time() - start_time) < timeout:
                try:
                    cv_image = self.capture_queue.get(timeout=0.1)
                    if cv_image is not None:
                        break
                except queue.Empty:
                    continue
            
            # 清理订阅
            if self.image_sub is not None:
                self.node.destroy_subscription(self.image_sub)
                self.image_sub = None
            
            if cv_image is None:
                return False, f"捕获超时 ({timeout}秒)", None, None
            
            # 转换为base64
            try:
                base64_data = self._cv2_to_base64(cv_image, resize_width)
                
                # 更新最后捕获
                self.last_capture.update({
                    "success": True,
                    "message": "图像捕获成功",
                    "cv_image": cv_image,
                    "base64_data": base64_data,
                    "timestamp": time.time(),
                    "camera_type": camera_type,
                    "topic": topic,
                })
                
                # 更新统计
                self.stats["total_captures"] += 1
                self.stats["successful_captures"] += 1
                self.stats["last_capture_time"] = time.time()
                
                # 添加到历史
                self._add_to_history(camera_type, topic, True)
                
                return True, "图像捕获成功", base64_data, cv_image
                
            except Exception as e:
                error_msg = f"图像处理失败: {str(e)}"
                return False, error_msg, None, cv_image
            
        except Exception as e:
            error_msg = f"捕获异常: {str(e)}"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg, None, None
            
        finally:
            with self.lock:
                self.is_capturing = False
                self.current_topic = None
    
    def capture_and_save(self, camera_type=None, save_dir=None, filename=None, resize_width=None):
        """
        捕获并保存图像
        
        返回: (success, message, base64_data, file_path)
        """
        # 先捕获图像
        success, message, base64_data, cv_image = self.capture_single_image(
            camera_type=camera_type,
            resize_width=resize_width
        )
        
        if not success or cv_image is None:
            return success, message, base64_data, None
        
        # 生成文件名
        camera_type = camera_type or self.config["default_camera"]
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{camera_type}_{timestamp}.jpg"
        
        if save_dir is None:
            save_dir = self.config["default_save_dir"]
        
        os.makedirs(save_dir, exist_ok=True)
        file_path = os.path.join(save_dir, filename)
        
        # 保存图像
        try:
            if cv2.imwrite(file_path, cv_image):
                self.logger.info(f"💾 图像保存成功: {file_path}")
                
                # 更新最后捕获
                self.last_capture["file_path"] = file_path
                
                return True, message, base64_data, file_path
            else:
                return False, f"保存失败: {file_path}", base64_data, None
                
        except Exception as e:
            return False, f"保存异常: {str(e)}", base64_data, None
    
    def _image_callback(self, msg):
        """图像回调函数"""
        if not self.is_capturing:
            return
        
        try:
            # 转换为OpenCV格式
            cv_image = self._ros_to_cv2(msg)
            
            if cv_image is not None:
                # 放入队列
                try:
                    self.capture_queue.put_nowait(cv_image)
                except queue.Full:
                    # 队列已满，丢弃最旧的数据
                    try:
                        self.capture_queue.get_nowait()
                        self.capture_queue.put_nowait(cv_image)
                    except queue.Empty:
                        pass
                        
        except Exception as e:
            self.logger.error(f"❌ 图像处理失败: {e}")
    
    def _ros_to_cv2(self, msg):
        """ROS Image消息转OpenCV"""
        try:
            if msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img_array = img_array.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                return img_array.reshape((msg.height, msg.width, 3))
            else:
                # 尝试通用解码
                try:
                    from cv_bridge import CvBridge
                    bridge = CvBridge()
                    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except (ImportError, Exception):
                    self.logger.warning(f"⚠️ 不支持编码: {msg.encoding}")
                    return None
                    
        except Exception as e:
            self.logger.error(f"❌ 图像转换异常: {e}")
            return None
    
    def _cv2_to_base64(self, cv_image, max_width=None):
        """OpenCV图像转base64"""
        if max_width is None:
            max_width = self.config["default_width"]
        
        # 调整大小
        height, width = cv_image.shape[:2]
        if width > max_width:
            scale = max_width / width
            new_width = max_width
            new_height = int(height * scale)
            cv_image = cv2.resize(cv_image, (new_width, new_height))
        
        # 编码为JPEG
        success, buffer = cv2.imencode(
            '.jpg',
            cv_image,
            [cv2.IMWRITE_JPEG_QUALITY, self.config["default_quality"]]
        )
        
        if not success:
            raise ValueError("JPEG编码失败")
        
        # 转换为base64
        image_bytes = buffer.tobytes()
        base64_str = base64.b64encode(image_bytes).decode('utf-8')
        
        return f"data:image/jpeg;base64,{base64_str}"
    
    def _clear_queue(self):
        """清空队列"""
        while not self.capture_queue.empty():
            try:
                self.capture_queue.get_nowait()
            except queue.Empty:
                break
    
    def _add_to_history(self, camera_type, topic, success):
        """添加到历史记录"""
        history_entry = {
            "timestamp": time.time(),
            "camera_type": camera_type,
            "topic": topic,
            "success": success,
        }
        
        self.history.append(history_entry)
        
        # 限制历史记录大小
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history:]
    
    def get_available_cameras(self):
        """获取可用的相机列表"""
        return list(self.camera_topics.keys())
    
    def get_camera_topic(self, camera_type):
        """获取相机的话题"""
        return self.camera_topics.get(camera_type)
    
    def get_last_capture(self):
        """获取最后捕获的数据"""
        return self.last_capture.copy()
    
    def get_status(self):
        """获取状态信息"""
        status = {
            "is_capturing": self.is_capturing,
            "current_topic": self.current_topic,
            "last_capture": self.last_capture.copy(),
            "stats": self.stats.copy(),
            "available_cameras": self.get_available_cameras(),
            "history_count": len(self.history),
        }
        
        # 格式化时间
        if self.last_capture["timestamp"]:
            status["last_capture"]["time_str"] = datetime.fromtimestamp(
                self.last_capture["timestamp"]
            ).strftime("%Y-%m-%d %H:%M:%S")
        
        if self.stats["last_capture_time"]:
            status["stats"]["last_capture_time_str"] = datetime.fromtimestamp(
                self.stats["last_capture_time"]
            ).strftime("%Y-%m-%d %H:%M:%S")
        
        return status
    
    def get_history(self, limit=10):
        """获取历史记录"""
        return self.history[-limit:] if self.history else []
    
    def clear_history(self):
        """清除历史记录"""
        self.history.clear()
        self.logger.info("🗑️ 历史记录已清除")