"""
机械臂滑件控制器（独立类）

- 维护关节目标值（默认 8 关节）
- 提供 set_joint / set_all / publish 接口
- 支持发布节流：拖动 slider 时不会每次都发（默认 10Hz）
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

from bimax_msgs.msg import RobotCommand, MotorCommand


class ArmSliderController:
    def __init__(
        self,
        robot_controller,
        joint_count: int = 8,
        publish_hz: float = 10.0,
        joint_limits: Optional[List[Tuple[float, float]]] = None,
    ):
        """
        Args:
            robot_controller: 你的 RobotController 实例（scripts/robot_controller.py）
            joint_count: 关节数量（你当前是 8）
            publish_hz: 最大发布频率（拖动时节流），例如 10Hz
            joint_limits: 每个关节的 (min, max)；若不传则不做限幅
        """
        self.controller = robot_controller
        self.joint_count = joint_count
        self.publish_period = 0.0 if publish_hz <= 0 else (1.0 / publish_hz)

        self.joint_limits = joint_limits
        self.values: List[float] = [0.0] * joint_count

        self._last_publish_time = 0.0
        self._dirty = False

    def get_values(self) -> List[float]:
        return list(self.values)

    def set_joint(self, index: int, value: float, publish: bool = False) -> str:
        if index < 0 or index >= self.joint_count:
            return f"❌ 关节索引越界: {index}"

        v = float(value)
        v = self._clamp(index, v)
        self.values[index] = v
        self._dirty = True

        if publish:
            return self.publish(throttle=True)
        return f"✅ 已设置关节{index}={v:.4f}"

    def set_all(self, values: List[float], publish: bool = False) -> str:
        if len(values) != self.joint_count:
            return f"❌ 关节数量不匹配: 期望{self.joint_count}, 实际{len(values)}"

        for i, v in enumerate(values):
            self.values[i] = self._clamp(i, float(v))
        self._dirty = True

        if publish:
            return self.publish(throttle=False)
        return "✅ 已更新全部关节目标值（未发布）"

    def publish(self, throttle: bool = True) -> str:
        """发布当前 values 到 /bimaxArmCommandValues"""
        if not self.controller or not self.controller.node:
            return "❌ 节点未就绪"

        if not self._dirty:
            return "ℹ️ 关节未变化，无需发布"

        now = time.time()
        if throttle and self.publish_period > 0:
            if (now - self._last_publish_time) < self.publish_period:
                return "⏳ 发布过于频繁，已节流"

        msg = RobotCommand()
        motor_commands = []
        for v in self.values:
            mc = MotorCommand()
            mc.q = float(v)
            mc.mode = 0
            motor_commands.append(mc)
        msg.motor_command = motor_commands

        self.controller.node.arm_publisher.publish(msg)

        self._last_publish_time = now
        self._dirty = False
        return "✅ 已发布滑件机械臂关节命令"

    def _clamp(self, index: int, value: float) -> float:
        if not self.joint_limits:
            return value
        if index >= len(self.joint_limits):
            return value
        lo, hi = self.joint_limits[index]
        if lo is None or hi is None:
            return value
        return max(lo, min(hi, value))