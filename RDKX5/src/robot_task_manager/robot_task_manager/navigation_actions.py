#!/usr/bin/env python3
"""
Nav2 导航控制器 — 通过 /goal_pose topic 发送目标（同 RViz 2D Goal Pose）
"""

import logging
import math
import time
from typing import Optional

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

logger = logging.getLogger(__name__)


class NavigationController:
    """
    导航控制器 —— 完全模仿 RViz "2D Goal Pose" 的工作方式。

    不直接调 Action Client，而是发布到 /goal_pose topic，
    bt_navigator 自动收取并执行。通过 /amcl_pose 判断是否到达。
    """

    def __init__(self, node: Node, timeout: float = 120.0,
                 goal_tolerance: float = 0.2):
        self._node = node
        self._timeout = timeout
        self._goal_tolerance = goal_tolerance

        # 发布 /goal_pose（同 RViz 2D Goal Pose）
        self._goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)

        # 订阅 AMCL 位姿
        self._current_x = 0.0
        self._current_y = 0.0
        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            10,
        )

        self._status = "idle"
        self._target: Optional[dict] = None
        self._start_time = 0.0
        self._last_log_time = 0.0      # 上次打印距离日志的时间
        self._pose_received = False    # 是否收到过 AMCL 位姿
        self._best_dist = float('inf') # 历史最近距离
        self._best_dist_time = 0.0     # 达到最近距离的时间

        # 途经点队列
        self._waypoints: list = []     # 途经点坐标列表
        self._wp_index = 0            # 当前途经点下标

        logger.info("NavigationController: 使用 /goal_pose topic（同 RViz）")

    # ── 公共接口 ──────────────────────────────────────

    def navigate_to(self, x: float, y: float, z: float = 0.0,
                    yaw: float = 0.0, frame: str = "map"):
        """发送导航目标到 /goal_pose（非阻塞）"""
        self._target = {"x": x, "y": y}
        self._waypoints = []   # 清除途经点（直接导航模式）
        self._wp_index = 0

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0

        half_yaw = yaw / 2.0
        goal.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw),
        )

        self._goal_pub.publish(goal)

        self._status = "active"
        self._start_time = time.time()
        self._best_dist = float('inf')
        self._best_dist_time = time.time()
        logger.info(f"导航目标已发布: ({x:.2f}, {y:.2f}) yaw={yaw:.2f}")

    def navigate_waypoints(self, waypoints: list):
        """
        发送途经点序列，逐个导航。

        Args:
            waypoints: [{"x":..., "y":..., "z":..., "yaw":...}, ...]
                       至少包含一个点，最后一个点是最终目标
        """
        if not waypoints:
            logger.warning("navigate_waypoints: 途经点列表为空")
            return

        self._waypoints = list(waypoints)
        self._wp_index = 0
        logger.info(f"途经点导航: 共 {len(self._waypoints)} 个点")
        self._send_current_waypoint()

    def _send_current_waypoint(self):
        """发送当前途经点"""
        wp = self._waypoints[self._wp_index]
        total = len(self._waypoints)
        logger.info(f"途经点 [{self._wp_index + 1}/{total}]: ({wp['x']:.2f}, {wp['y']:.2f})")
        self.navigate_to(
            wp["x"], wp["y"],
            wp.get("z", 0.0),
            wp.get("yaw", 0.0),
        )

    def get_status(self) -> str:
        """
        Returns: 'idle' | 'active' | 'succeeded' | 'failed'
        """
        if self._status != "active":
            return self._status

        # 超时
        if time.time() - self._start_time > self._timeout:
            logger.error(f"导航超时 ({self._timeout}s)")
            self._status = "failed"
            return self._status

        # 距离判断
        if self._target:
            dx = self._current_x - self._target["x"]
            dy = self._current_y - self._target["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            now = time.time()

            # 追踪最佳距离
            if dist < self._best_dist:
                self._best_dist = dist
                self._best_dist_time = now

            # 距离不再缩小超过 3 秒 → Nav2 已尽力，接受当前距离
            settling = now - self._best_dist_time > 3.0
            close_enough = dist < 0.30  # 30cm 以内就算到了

            # 每 2 秒打印一次距离
            if now - self._last_log_time > 2.0:
                self._last_log_time = now
                logger.info(
                    f"📍 当前位置 ({self._current_x:.2f}, {self._current_y:.2f}) "
                    f"→ 目标 ({self._target['x']:.2f}, {self._target['y']:.2f}) "
                    f"距离 {dist:.2f}m 最佳 {self._best_dist:.2f}m"
                )

            if dist < self._goal_tolerance:
                self._on_waypoint_reached(dist, now)
            elif settling and close_enough:
                self._on_waypoint_reached(dist, now)

        return self._status

    def _on_waypoint_reached(self, dist: float, now: float):
        """当前途经点到达后的处理：推进到下一个或标记成功"""
        if self._waypoints and self._wp_index + 1 < len(self._waypoints):
            # 还有更多途经点 → 推进
            self._wp_index += 1
            logger.info(
                f"✅ 途经点到达 (距离 {dist:.3f}m), "
                f"推进到 [{self._wp_index + 1}/{len(self._waypoints)}]"
            )
            self._send_current_waypoint()
            # _send_current_waypoint → navigate_to 已将 status 重置为 "active"
        else:
            self._status = "succeeded"
            logger.info(f"✅ 到达最终目标 (距离 {dist:.3f}m)")

    def cancel(self):
        """取消导航"""
        self._status = "idle"
        self._target = None
        self._waypoints = []
        self._wp_index = 0

    @property
    def target(self) -> Optional[dict]:
        return self._target

    # ── 内部 ──────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self._pose_received:
            self._pose_received = True
            logger.info(f"📍 首次收到 AMCL 位姿: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
