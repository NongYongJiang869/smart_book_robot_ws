#!/usr/bin/env python3
"""
机械臂控制器（模拟实现）

第一阶段使用模拟实现，后续替换为真实串口通信。
接口与 robot_state_machine.py 的调用方式保持一致。
"""

import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)


class ArmController:
    """
    机械臂操作接口。

    用法:
        arm = ArmController(sim_delay=True)
        arm.extend()
        while arm.get_status() == "active":
            time.sleep(0.2)
        if arm.get_status() == "succeeded":
            print("操作完成")
    """

    def __init__(self, sim_delay: bool = True, sim_duration: float = 1.5):
        """
        Args:
            sim_delay:     True = 模拟延迟，False = 立即完成（测试用）
            sim_duration:  每个操作的模拟耗时（秒）
        """
        self._sim_delay = sim_delay
        self._sim_duration = sim_duration

        self._status = "idle"      # idle | active | succeeded | failed
        self._action = ""          # 当前动作名
        self._start_time = 0.0

    # ── 公共接口 ──────────────────────────────────────

    def extend(self):
        """展开机械臂：从停靠位展开到工作姿态"""
        self._start_action("extend")
        logger.info("🦾 机械臂展开中...")

    def locate_book(self, book_title: str = ""):
        """视觉定位书籍"""
        self._start_action("locate")
        if book_title:
            logger.info(f"👁️ 扫描书架，定位《{book_title}》...")
        else:
            logger.info("👁️ 扫描书架，定位目标书籍...")

    def grasp(self):
        """夹爪抓取"""
        self._start_action("grasp")
        logger.info("✋ 夹爪闭合抓取...")

    def retract(self):
        """机械臂收回"""
        self._start_action("retract")
        logger.info("🦾 机械臂收回中...")

    def get_status(self) -> str:
        """
        获取当前操作状态。

        Returns: 'idle' | 'active' | 'succeeded' | 'failed'
        """
        if self._status == "active" and self._sim_delay:
            if time.time() - self._start_time >= self._sim_duration:
                self._status = "succeeded"
                logger.info(f"✅ 动作 '{self._action}' 完成")
        return self._status

    def fail(self):
        """手动触发失败（测试异常路径用）"""
        self._status = "failed"
        logger.error(f"❌ 动作 '{self._action}' 失败")

    # ── 内部方法 ──────────────────────────────────────

    def _start_action(self, name: str):
        self._action = name
        self._start_time = time.time()
        if self._sim_delay:
            self._status = "active"
        else:
            # 无延迟模式：立即完成
            self._status = "succeeded"
