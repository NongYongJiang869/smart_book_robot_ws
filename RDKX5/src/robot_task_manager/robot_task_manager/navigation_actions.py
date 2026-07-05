#!/usr/bin/env python3
"""
Nav2 导航动作客户端封装

封装 nav2_msgs/action/NavigateToPose，提供同步式状态查询接口。
"""

import logging
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

logger = logging.getLogger(__name__)


class NavigationController:
    """
    Nav2 导航控制器。

    用法:
        nav = NavigationController(node)
        nav.navigate_to(1.0, 2.0, 0.0)       # 发送目标
        while nav.get_status() == "active":    # 轮询等待
            rclpy.spin_once(node, timeout_sec=0.1)
        if nav.get_status() == "succeeded":
            print("到达!")
    """

    def __init__(self, node: Node, timeout: float = 120.0):
        """
        Args:
            node:    ROS2 节点
            timeout: 单次导航超时（秒），超时后自动标记为 failed
        """
        self._node = node
        self._timeout = timeout

        self._action_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

        self._goal_handle = None
        self._result_future: Optional[Future] = None
        self._status = "idle"        # idle | active | succeeded | failed
        self._start_time = 0.0
        self._target: Optional[dict] = None

        # 等待 action server 就绪
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            logger.warning(
                "NavigateToPose action server 未就绪，"
                "导航将在 Nav2 启动后可用"
            )
        else:
            logger.info("NavigationController: NavigateToPose server 已连接")

    # ── 公共接口 ──────────────────────────────────────

    def navigate_to(self, x: float, y: float, z: float = 0.0,
                    yaw: float = 0.0, frame: str = "map"):
        """
        发送导航目标（非阻塞）。

        Args:
            x, y:  地图坐标（米）
            z:     楼层相关高度（如果使用 3D 坐标，0 表示忽略 z）
            yaw:   目标朝向（弧度），0=正东
            frame: 坐标系，默认 "map"
        """
        # 先取消之前的导航
        if self._status == "active":
            self.cancel()

        self._target = {"x": x, "y": y, "z": z, "yaw": yaw}

        # 构建 PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0   # 2D 导航忽略 z

        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal_pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # 发送
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

        self._status = "active"
        self._start_time = time.time()
        logger.info(
            f"导航目标已发送: ({x:.2f}, {y:.2f}, z={z:.1f}) yaw={yaw:.2f}"
        )

    def get_status(self) -> str:
        """
        获取当前导航状态。

        Returns: 'idle' | 'active' | 'succeeded' | 'failed'
        """
        # 超时检查
        if self._status == "active":
            if time.time() - self._start_time > self._timeout:
                logger.error(f"导航超时 ({self._timeout}s)")
                self.cancel()
                self._status = "failed"
        return self._status

    def cancel(self):
        """取消当前导航"""
        if self._goal_handle is not None:
            logger.info("取消导航")
            future = self._goal_handle.cancel_goal_async()
            # 自旋等待取消完成
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
        self._status = "idle"
        self._goal_handle = None
        self._result_future = None

    @property
    def target(self) -> Optional[dict]:
        """当前导航目标（用于调试/日志）"""
        return self._target

    # ── 内部回调 ──────────────────────────────────────

    def _goal_response_callback(self, future: Future):
        """目标已被 server 接受或拒绝"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.error("导航目标被 Nav2 拒绝")
            self._status = "failed"
            return

        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future):
        """导航完成回调"""
        result = future.result()
        if result.status == 4:  # SUCCEEDED (Nav2 action result)
            self._status = "succeeded"
            logger.info("导航成功到达目标")
        else:
            self._status = "failed"
            logger.warning(f"导航失败, Nav2 status={result.status}")
