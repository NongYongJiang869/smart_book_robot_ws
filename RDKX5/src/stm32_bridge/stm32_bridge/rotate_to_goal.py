#!/usr/bin/env python3
"""
到位后原地旋转对齐 — Nav2 到位后接管 cmd_vel 纯原地旋转，完成后取消 Nav2 任务。
用法: 和 navigation 一起启动即可。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


class RotateToGoal(Node):
    def __init__(self):
        super().__init__('rotate_to_goal')

        self.goal = None
        self.goal_time = self.get_clock().now()
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0

        # 卡住检测 — 记录最近一次位置变化的时间和坐标
        self.best_x = 0.0
        self.best_y = 0.0
        self.best_dist_time = self.get_clock().now()

        self.state = 'IDLE'       # IDLE -> WAIT -> ROTATING -> DONE
        self.state_enter_time = self.get_clock().now()
        self._last_debug = 0

        # params
        self.declare_parameter('xy_tolerance', 0.25)
        self.declare_parameter('yaw_tolerance', 0.087)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('stuck_timeout', 5.0)        # 卡住不动多久放弃
        self.declare_parameter('goal_timeout', 300.0)       # 硬上限兜底

        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.ang_speed = self.get_parameter('angular_speed').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.goal_timeout = self.get_parameter('goal_timeout').value

        # Nav2 action client — 用来发送目标（不直接用，只为了连上 action server）
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # CancelGoal 服务 — 取消所有 goal（不管谁发的）
        from action_msgs.srv import CancelGoal
        self.cancel_srv = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        # subs
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RotateToGoal started')

    def goal_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z), 1.0 - 2.0*(q.z*q.z))
        self.goal = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.goal_time = self.get_clock().now()
        self.best_x = self.curr_x
        self.best_y = self.curr_y
        self.best_dist_time = self.get_clock().now()
        self._set_state('IDLE')
        self.get_logger().info(
            f'Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) yaw={math.degrees(yaw):.1f}')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.curr_x = p.position.x
        self.curr_y = p.position.y
        q = p.orientation
        self.curr_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _set_state(self, s):
        if self.state != s:
            self.state = s
            self.state_enter_time = self.get_clock().now()

    def _in_state(self, sec):
        dt = (self.get_clock().now() - self.state_enter_time).nanoseconds * 1e-9
        return dt > sec

    def _cancel_nav2(self):
        """取消 Nav2 的 NavigateToPose action 上所有 goal"""
        from action_msgs.srv import CancelGoal
        if not self.cancel_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('CancelGoal service not available')
            return
        try:
            req = CancelGoal.Request()  # goal_info=[] means cancel ALL
            self.get_logger().info('Cancelling all Nav2 goals')
            self.cancel_srv.call_async(req)
        except Exception as e:
            self.get_logger().error(f'Cancel Nav2 failed: {e}')

    def control_loop(self):
        if self.goal is None:
            return

        gx, gy, gyaw = self.goal
        dist = math.sqrt((self.curr_x - gx)**2 + (self.curr_y - gy)**2)
        yaw_err = gyaw - self.curr_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        # 卡住检测 — 只在接近目标时检查（远处交给 Nav2 处理）
        if dist < 2.0:
            moved = math.sqrt((self.curr_x - self.best_x)**2 + (self.curr_y - self.best_y)**2)
            if moved > 0.05:   # 位置变了 ≥ 5cm → 还在动
                self.best_x = self.curr_x
                self.best_y = self.curr_y
                self.best_dist_time = self.get_clock().now()
            stuck_dt = (self.get_clock().now() - self.best_dist_time).nanoseconds * 1e-9
            if stuck_dt > self.stuck_timeout:
                self.get_logger().warn(f'Not moving for {stuck_dt:.0f}s near goal, giving up')
                self.goal = None
                self._set_state('IDLE')
                return
        else:
            # 远处：只要有目标就一直等 Nav2
            self.best_x = self.curr_x
            self.best_y = self.curr_y
            self.best_dist_time = self.get_clock().now()
        # 硬超时兜底
        elapsed = (self.get_clock().now() - self.goal_time).nanoseconds * 1e-9
        if elapsed > self.goal_timeout:
            self.get_logger().warn('Hard timeout')
            self.goal = None
            self._set_state('IDLE')
            return

        # debug: every 5s log current state
        now_sec = int(self.get_clock().now().nanoseconds * 1e-9)
        if now_sec % 5 == 0 and not getattr(self, '_last_debug', 0) == now_sec:
            self._last_debug = now_sec
            self.get_logger().info(
                f'[{self.state}] dist={dist*100:.0f}cm err={math.degrees(yaw_err):.1f}deg')

        twist = Twist()

        if self.state == 'IDLE':
            if dist < self.xy_tol:
                # 到位后立刻取消 Nav2，防止它进 recovery 循环
                self.get_logger().info(f'Position reached (err={dist*100:.0f}cm), cancel Nav2')
                self._cancel_nav2()
                self._set_state('WAIT')

        elif self.state == 'WAIT':
            # 等 0.5 秒让 Nav2 彻底停
            if self._in_state(0.5):
                if abs(yaw_err) > self.yaw_tol:
                    self.get_logger().info(f'Start rotating: err={math.degrees(yaw_err):.1f}')
                    self._set_state('ROTATING')
                else:
                    self.get_logger().info(f'Already aligned (err={math.degrees(yaw_err):.1f})')
                    self.goal = None
                    self._set_state('IDLE')

        elif self.state == 'ROTATING':
            if abs(yaw_err) > self.yaw_tol:
                speed = self.ang_speed if yaw_err > 0 else -self.ang_speed
                slow = self.ang_speed * 0.3
                if abs(yaw_err) < slow:
                    speed = speed * abs(yaw_err) / slow
                if 0.01 < abs(speed) < 0.3:
                    speed = 0.3 if speed > 0 else -0.3
                twist.angular.z = speed
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f'Rotation done (err={math.degrees(yaw_err):.1f})')
                self.cmd_pub.publish(Twist())
                self.goal = None
                self._set_state('IDLE')


def main():
    rclpy.init()
    node = RotateToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # 防止 rcl_shutdown 双重调用崩溃
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
