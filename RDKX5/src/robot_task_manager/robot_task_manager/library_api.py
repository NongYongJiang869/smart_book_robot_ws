#!/usr/bin/env python3
"""
图书馆服务器 HTTP API 封装

提供:
  - get_tasks()       → 获取待处理任务列表
  - accept_task()     → 接单
  - update_task()     → 更新任务状态
  - heartbeat()       → 心跳上报

所有方法返回 dict 或 None（失败时），不抛异常。
"""

import logging
import time
from typing import Optional

import requests

logger = logging.getLogger(__name__)

# ── 订单状态码（发送给服务器） ──
STATUS_ACCEPTED       = "accepted"
STATUS_SEARCHING      = "searching"
STATUS_ARM_EXTENDING  = "arm_extending"
STATUS_LOCATING       = "locating"
STATUS_GRASPING       = "grasping"
STATUS_DELIVERING     = "delivering"
STATUS_DELIVERED      = "delivered"

# ── 机器人状态码 ──
ROBOT_IDLE     = "idle"
ROBOT_BUSY     = "busy"
ROBOT_CHARGING = "charging"
ROBOT_ERROR    = "error"


class LibraryAPI:
    """图书馆服务器 REST API 客户端"""

    def __init__(self, robot_name: str, server: str = "39.105.113.176",
                 timeout: float = 5.0, max_retries: int = 3):
        """
        Args:
            robot_name: 机器人唯一名称，如 "robot-01"
            server:     服务器地址（不含协议和路径），如 "39.105.113.176"
            timeout:    HTTP 请求超时（秒）
            max_retries: 失败重试次数
        """
        self.robot_name = robot_name
        self.base = f"http://{server}/api/robot"
        self.timeout = timeout
        self.max_retries = max_retries

    # ── 公开方法 ──────────────────────────────────────

    def get_tasks(self) -> Optional[dict]:
        """GET /api/robot/tasks/ — 获取待处理任务列表"""
        return self._get("/tasks/")

    def accept_task(self, order_id: int) -> Optional[dict]:
        """POST /api/robot/tasks/{order_id}/accept/ — 接单"""
        return self._post(
            f"/tasks/{order_id}/accept/",
            {"robot_name": self.robot_name},
        )

    def update_task(self, order_id: int, status: str,
                    position: str = "") -> Optional[dict]:
        """POST /api/robot/tasks/{order_id}/update/ — 更新任务状态"""
        return self._post(
            f"/tasks/{order_id}/update/",
            {
                "robot_name": self.robot_name,
                "status": status,
                "current_position": position,
            },
        )

    def heartbeat(self, status: str, position: str = "",
                  battery: int = 100) -> Optional[dict]:
        """POST /api/robot/heartbeat/ — 心跳上报"""
        return self._post(
            "/heartbeat/",
            {
                "robot_name": self.robot_name,
                "status": status,
                "current_position": position,
                "battery": battery,
            },
        )

    # ── 内部方法 ──────────────────────────────────────

    def _post(self, path: str, data: dict) -> Optional[dict]:
        """带重试的 POST 请求"""
        url = f"{self.base}{path}"
        for attempt in range(1, self.max_retries + 1):
            try:
                r = requests.post(url, json=data, timeout=self.timeout)
                r.raise_for_status()
                return r.json()
            except requests.exceptions.ConnectionError:
                logger.error(f"无法连接服务器: {url}")
                if attempt == self.max_retries:
                    return None
            except requests.exceptions.Timeout:
                logger.warning(f"请求超时 ({attempt}/{self.max_retries}): {url}")
                if attempt == self.max_retries:
                    return None
            except requests.exceptions.HTTPError as e:
                logger.error(
                    f"HTTP {e.response.status_code}: "
                    f"{e.response.text[:200]}"
                )
                return None  # HTTP 错误不重试
            time.sleep(1.0)
        return None

    def _get(self, path: str) -> Optional[dict]:
        """带重试的 GET 请求"""
        url = f"{self.base}{path}"
        for attempt in range(1, self.max_retries + 1):
            try:
                r = requests.get(url, timeout=self.timeout)
                r.raise_for_status()
                return r.json()
            except requests.exceptions.ConnectionError:
                logger.error(f"无法连接服务器: {url}")
                if attempt == self.max_retries:
                    return None
            except requests.exceptions.Timeout:
                logger.warning(f"请求超时 ({attempt}/{self.max_retries}): {url}")
                if attempt == self.max_retries:
                    return None
            except requests.exceptions.HTTPError as e:
                logger.error(f"HTTP {e.response.status_code}")
                return None
            time.sleep(1.0)
        return None
