#!/usr/bin/env python3
"""
机械臂控制器

与 6 轴机械臂 Arduino 固件通信（/dev/ttyACM0, 115200）

协议:
  发 "1\\n"  → 取书 #1，机械臂自动完成: 展开 → 定位 → 夹取
  发 "2\\n"  → 取书 #2
  收到 "9"  → 机械臂已收到指令 (ACK)
  收到 "0"  → 夹取完成
  发 "3\\n"  → 放书
  收到 "6"  → 放书完成

用法:
  arm = ArmController(mode="serial", port="/dev/ttyACM0")
  arm.extend()
  arm.locate_book("Minimalist Forms")   # 查找 book_number → 发串口
  arm.grasp()                          # 等待 "0"
  arm.place()                          # 发 "3", 等待 "6"
"""

import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
GRASP_TIMEOUT = 180.0   # 夹取超时（秒）
PLACE_TIMEOUT = 90.0   # 放书超时（秒）


class ArmController:
    """机械臂操作接口"""

    def __init__(self, mode: str = "mock", port: str = DEFAULT_PORT,
                 baud: int = DEFAULT_BAUD, sim_duration: float = 1.5):
        """
        Args:
            mode:         "mock" | "serial"
            port:         串口设备路径
            baud:         波特率
            sim_duration: mock 模式动作耗时
        """
        self._mode = mode
        self._sim_duration = sim_duration

        self._status = "idle"       # idle | active | succeeded | failed
        self._action = ""
        self._start_time = 0.0
        self._book_number: Optional[int] = None
        self._ack_received = False  # 是否收到机械臂 ACK ("9")

        # 串口
        self._ser = None
        if mode == "serial":
            import serial
            try:
                self._ser = serial.Serial(port, baud, timeout=0.1)
                logger.info(f"机械臂串口已连接: {port} @ {baud}")
            except Exception as e:
                logger.error(f"无法打开机械臂串口 {port}: {e}")
                self._ser = None

        # BookMapper 引用（由 task_manager_node 注入）
        self.book_mapper = None

    # ── 公共接口 ──────────────────────────────────────

    def extend(self):
        """展开机械臂（串口模式下瞬时完成，固件在收到 book_number 后自动展开）"""
        logger.info("🦾 机械臂展开...")
        if self._mode == "mock":
            self._start_action("extend")
        else:
            # serial 模式: 瞬时完成，固件内部处理
            self._status = "succeeded"

    def locate_book(self, book_title: str = ""):
        """
        发送取书指令并等待机械臂完成（展开+定位+夹取）。

        串口模式: 发送 "1\n"，等待 "0" 返回。
        """
        self._start_action("locate")
        self._book_number = 1  # 暂时固定取书 #1

        logger.info(f"📖《{book_title or '?'}》→ 取书 #{self._book_number}")

        if self._mode == "serial" and self._ser:
            self._ser.write(b"1\n")
            self._ser.flush()
            logger.info("📤 发送取书指令: 1")

    def grasp(self):
        """
        夹取完成确认（串口模式下由 locate_book 完成全部动作，此处为 no-op）。
        """
        if self._mode == "mock":
            self._start_action("grasp")
            logger.info("✋ 夹爪闭合抓取...")
        else:
            logger.info("✋ 夹取已完成（串口模式）")
            self._status = "succeeded"

    def place(self):
        """
        放书。

        串口模式: 发送 "3"，等待机械臂返回 "6"
        """
        self._start_action("place")
        logger.info("📦 放书...")

        if self._mode == "serial" and self._ser:
            self._ser.write(b"3\n")
            self._ser.flush()
            logger.info("📤 发送放书指令")

    def retract(self):
        """机械臂收回"""
        logger.info("🦾 机械臂收回...")
        if self._mode == "mock":
            self._start_action("retract")
        else:
            self._status = "succeeded"

    def get_status(self) -> str:
        """
        获取当前操作状态。

        mock 模式: 基于时间自动完成
        serial 模式: 检查串口响应中是否有 "0"

        Returns: 'idle' | 'active' | 'succeeded' | 'failed'
        """
        if self._status != "active":
            return self._status

        if self._mode == "mock":
            if time.time() - self._start_time >= self._sim_duration:
                self._status = "succeeded"
                logger.info(f"✅ 动作 '{self._action}' 完成")

        elif self._mode == "serial":
            # 串口模式下持续读取
            self._read_serial()
            # 超时检查
            if self._status == "active":
                timeout = PLACE_TIMEOUT if self._action == "place" else GRASP_TIMEOUT
                if time.time() - self._start_time > timeout:
                    self._status = "failed"
                    logger.error(f"❌ 动作 '{self._action}' 超时 ({timeout}s)")

        return self._status

    def close(self):
        """关闭串口"""
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info("机械臂串口已关闭")

    # ── 内部方法 ──────────────────────────────────────

    def _start_action(self, name: str):
        self._action = name
        self._start_time = time.time()
        self._status = "active"
        self._ack_received = False

    def _read_serial(self):
        """读取串口数据，检查 '9' (ACK)、'0' (夹取完成)、'6' (放书完成)"""
        if not self._ser or not self._ser.is_open:
            return
        try:
            while self._ser.in_waiting > 0:
                data = self._ser.read(self._ser.in_waiting).decode(
                    "utf-8", errors="replace"
                ).strip()
                if data:
                    logger.info(f"📨 机械臂响应: '{data}'")
                    if "9" in data:
                        self._ack_received = True
                        logger.info("📨 机械臂 ACK: 指令已收到")
                    elif "6" in data:
                        self._status = "succeeded"
                        logger.info("✅ 放书完成 (收到 '6')")
                    elif "0" in data:
                        self._status = "succeeded"
                        logger.info("✅ 夹取完成 (收到 '0')")
        except Exception as e:
            logger.warning(f"串口读取异常: {e}")
