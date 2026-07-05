#!/usr/bin/env python3
"""
到位后原地旋转对齐 — 确认 Nav2 彻底停止后才接管 cmd_vel，纯原地旋转到目标方向。
避免与 Nav2 的 cmd_vel 发生竞态。
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped


class RotateToGoal(Node):
    def __init__(self):
        super().__init__('rotate_to_goal')

        # 最后收到的目标
        self.goal = None          # (x, y, yaw)
        self.goal_time = self.get_clock().now()

        # 当前位置
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0

        # nav2 的 cmd_vel — 用于判断 nav2 是否还在活动
        self.nav2_cmd_active = False
        self.last_nav2_cmd_time = self.get_clock().now()  # 上次收到 nav2 cmd_vel 的时间

        # 状态机: IDLE → WAIT_NAV2_DONE → ROTATING → DONE
        self.state = 'IDLE'

        # ── 参数 ──
        self.declare_parameter('xy_tolerance', 0.25)       # 认为"到位"的位置容差
        self.declare_parameter('yaw_tolerance', 0.087)     # 方向容差 rad (~5°)
        self.declare_parameter('angular_speed', 0.5)       # 旋转角速度 rad/s
        self.declare_parameter('goal_timeout', 30.0)       # 目标超时秒
        self.declare_parameter('nav2_idle_time', 1.5)      # Nav2 cmd_vel 静默多久算结束

        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.ang_speed = self.get_parameter('angular_speed').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.nav2_idle = self.get_parameter('nav2_idle_time').value

        # ── 订阅 ──
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        # 监听 nav2 发布的 cmd_vel（topic_tools 不可靠，直接用时间戳判断）
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # ── 发布 cmd_vel ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 定时器 ──
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RotateToGoal 节点已启动')

    def cmd_vel_cb(self, msg: Twist):
        """订阅所有 cmd_vel，记录 nav2 活动时间（跳过自己发的）"""
        if self.state == 'ROTATING':
            return  # 自己正在旋转，不更新 nav2 时间戳
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.last_nav2_cmd_time = self.get_clock().now()

    def goal_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.goal_time = self.get_clock().now()
        self.state = 'IDLE'
        self.get_logger().info(f'收到目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) yaw={math.degrees(yaw):.1f}°')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.curr_x = p.position.x
        self.curr_y = p.position.y
        q = p.orientation
        self.curr_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _nav2_is_idle(self) -> bool:
        """Nav2 的 cmd_vel 已经静默超过 nav2_idle 秒"""
        dt = (self.get_clock().now() - self.last_nav2_cmd_time).nanoseconds * 1e-9
        return dt > self.nav2_idle

    def control_loop(self):
        if self.goal is None:
            return

        gx, gy, gyaw = self.goal

        # 目标超时
        elapsed = (self.get_clock().now() - self.goal_time).nanoseconds * 1e-9
        if elapsed > self.goal_timeout:
            if self.state == 'WAIT_NAV2_DONE':
                self.get_logger().warn('等待 Nav2 结束超时，强制接管')
                self.state = 'ROTATING'

        # 位置误差 & 角度误差
        dist = math.sqrt((self.curr_x - gx)**2 + (self.curr_y - gy)**2)
        yaw_err = gyaw - self.curr_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        twist = Twist()

        # ── 状态机 ──
        if self.state == 'IDLE':
            # 等待 nav2 把车开到目标位置
            if dist < self.xy_tol and self._nav2_is_idle():
                self.state = 'WAIT_NAV2_DONE'
                # 额外等 1 秒确保 nav2 真的结束了
                self.nav2_done_time = self.get_clock().now()

        elif self.state == 'WAIT_NAV2_DONE':
            # 确认 nav2 已经结束
            if self._nav2_is_idle():
                wait = (self.get_clock().now() - self.nav2_done_time).nanoseconds * 1e-9
                if wait > 1.0:  # 额外等 1 秒安全确认
                    if abs(yaw_err) > self.yaw_tol:
                        self.get_logger().info(f'Nav2 结束，开始原地旋转: 偏差={math.degrees(yaw_err):.1f}°')
                        self.state = 'ROTATING'
                    else:
                        self.get_logger().info(f'方向已对齐 ✓ (偏差={math.degrees(yaw_err):.1f}°)')
                        self.goal = None
                        self.state = 'IDLE'

        elif self.state == 'ROTATING':
            # 纯原地旋转
            if dist > self.xy_tol * 2:  # 位置跑偏太多，放弃
                self.get_logger().warn(f'位置偏移过大 ({dist:.2f}m)，停止旋转')
                self.cmd_pub.publish(twist)
                self.goal = None
                self.state = 'IDLE'
                return

            if abs(yaw_err) > self.yaw_tol:
                speed = self.ang_speed if yaw_err > 0 else -self.ang_speed
                # 接近目标时减速
                slow_thresh = self.ang_speed * 0.3
                if abs(yaw_err) < slow_thresh:
                    speed = speed * abs(yaw_err) / slow_thresh
                # 最小角速度 0.3
                if abs(speed) < 0.3 and abs(speed) > 0.01:
                    speed = 0.3 if speed > 0 else -0.3

                twist.angular.z = speed
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f'方向对齐完成 ✓ (偏差={math.degrees(yaw_err):.1f}°)')
                self.cmd_pub.publish(twist)  # 停
                self.goal = None
                self.state = 'IDLE'


def main():
    rclpy.init()
    node = RotateToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
