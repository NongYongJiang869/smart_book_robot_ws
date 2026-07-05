#!/usr/bin/env python3
"""
机器人任务管理器 ROS2 节点

串联 LibraryAPI → LocationMapper → RobotStateMachine → NavigationController → ArmController
通过定时器驱动状态机，周期性发送心跳和轮询任务。

用法:
    ros2 launch robot_task_manager task_manager.launch.py
    ros2 run robot_task_manager task_manager_node --ros-args -p robot_name:=robot-01
"""

import os
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from .library_api import LibraryAPI, ROBOT_IDLE
from .location_mapper import LocationMapper
from .robot_state_machine import RobotStateMachine, RobotState
from .navigation_actions import NavigationController
from .arm_controller import ArmController


class TaskManagerNode(Node):
    """机器人任务管理器主节点"""

    def __init__(self):
        super().__init__('task_manager_node')

        # ── 参数声明 ──
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', 'robot-01'),
                ('server_url', '39.105.113.176'),
                ('heartbeat_interval', 5.0),
                ('poll_interval', 3.0),
                ('state_tick_rate', 5.0),
                ('nav_timeout', 120.0),
                ('nav_retries', 2),
                ('docking_station', '1F-充电站'),
                ('locations_file', 'locations.json'),
            ],
        )

        robot_name = self.get_parameter('robot_name').value
        server_url = self.get_parameter('server_url').value

        self.get_logger().info(f'🤖 机器人: {robot_name}')
        self.get_logger().info(f'🌐 服务器: {server_url}')

        # ── 1. HTTP API ──
        self.api = LibraryAPI(robot_name, server_url)

        # ── 2. 位置映射 ──
        locations_file = self.get_parameter('locations_file').value
        if not os.path.isabs(locations_file):
            # 相对路径 → 在包的 share 目录下查找
            try:
                pkg_share = get_package_share_directory('robot_task_manager')
                locations_path = os.path.join(pkg_share, 'config', locations_file)
            except Exception:
                # ament index 不可用时（开发环境），尝试当前目录
                locations_path = os.path.join(
                    os.path.dirname(__file__), '..', 'config', locations_file
                )
        else:
            locations_path = locations_file

        self.get_logger().info(f'📍 位置映射: {locations_path}')
        self.mapper = LocationMapper(locations_path)

        # ── 3. 导航控制器 ──
        nav_timeout = self.get_parameter('nav_timeout').value
        self.nav = NavigationController(self, timeout=nav_timeout)

        # ── 4. 机械臂控制器 (模拟) ──
        self.arm = ArmController(sim_delay=True, sim_duration=1.5)

        # ── 5. 状态机 ──
        self.sm = RobotStateMachine(self.api, self.mapper, self.nav, self.arm)
        self.sm.set_docking(self.get_parameter('docking_station').value)
        self.sm.on_state_changed = self._on_state_changed

        # ── 6. 定时器 ──
        hb_interval = self.get_parameter('heartbeat_interval').value
        self.hb_timer = self.create_timer(hb_interval, self._heartbeat_loop)

        poll_interval = self.get_parameter('poll_interval').value
        self.poll_timer = self.create_timer(poll_interval, self._poll_loop)

        tick_rate = self.get_parameter('state_tick_rate').value
        self.tick_timer = self.create_timer(1.0 / tick_rate, self._state_tick)

        # ── 启动 ──
        self.get_logger().info('✅ TaskManager 节点已启动')
        self._send_heartbeat()  # 立即发送一次心跳注册上线

    # ── 定时器回调 ────────────────────────────────────

    def _heartbeat_loop(self):
        """周期性心跳"""
        self._send_heartbeat()

    def _poll_loop(self):
        """空闲时轮询任务"""
        if self.sm.state != RobotState.IDLE:
            return

        resp = self.api.get_tasks()
        if resp is None:
            return

        tasks = resp.get("tasks", [])
        pending = [t for t in tasks if t.get("status") == "pending"]

        if pending:
            task = pending[0]  # 取第一个待处理任务
            self.get_logger().info(
                f'🔔 发现新任务 #{task["id"]}: 《{task["book_title"]}》'
            )
            self.sm.assign_task(task)

    def _state_tick(self):
        """驱动状态机"""
        self.sm.tick()

    # ── 状态变化回调 ──────────────────────────────────

    def _on_state_changed(self, old: RobotState, new: RobotState):
        """状态转移时的回调（用于日志和其他副作用）"""
        self.get_logger().info(f'状态: {old.name} → {new.name}')
        # 状态变化后立即发一次心跳，让服务器知道最新状态
        self._send_heartbeat()

    # ── 内部方法 ──────────────────────────────────────

    def _send_heartbeat(self):
        """发送心跳"""
        robot_status = self.sm.get_robot_status()

        # 构建位置描述
        dock = self.sm.docking_station
        if self.sm.state == RobotState.IDLE:
            position = f"空闲 {dock}"
        elif self.sm.state == RobotState.RETURNING:
            position = f"返回{dock}途中"
        elif self.sm.state == RobotState.CHARGING:
            position = f"{dock} 充电中"
        elif self.sm.state == RobotState.ERROR:
            position = f"故障: {self.sm.error_reason}"
        elif self.sm.current_task:
            task = self.sm.current_task
            stage = self.sm.state_name
            position = f"[{stage}] {task.get('book_location', '')}"
        else:
            position = "未知"

        resp = self.api.heartbeat(
            status=robot_status,
            position=position,
            battery=self.sm.battery,
        )

        if resp:
            # 检查服务器返回的活跃任务（断线重连恢复用）
            active = resp.get("active_task")
            if active and self.sm.state == RobotState.IDLE:
                self.get_logger().warn(
                    f'🔄 服务器上有活跃任务 #{active["id"]}，恢复上下文'
                )
                # TODO: 恢复任务上下文


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
