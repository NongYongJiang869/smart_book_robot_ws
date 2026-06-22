#!/usr/bin/env python3
"""
STM32 串口桥接节点

功能:
  - 通过 /dev/ttyS1 (115200 8N1) 与 STM32 底盘通信
  - 发布 /odom (nav_msgs/Odometry), /imu (sensor_msgs/Imu)
  - 发布 /chassis_status (custom_interfaces/ChassisStatus)
  - 发布 /tf (odom → base_link)
  - 订阅 /cmd_vel (geometry_msgs/Twist), 编码后发送到 STM32

超时保护: 200ms 未收到新 /cmd_vel → 自动发送零速指令
"""

import math
import time
import struct
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from custom_interfaces.msg import ChassisStatus

from .serial_protocol import SerialProtocol
from .odometry import OdometryComputer, CalibrationMonitor, quaternion_from_yaw
from custom_interfaces.srv import CalibrateWheelBase


# ── QoS 策略 (同设计文档 04) ──
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

QOS_STATUS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class STM32BridgeNode(Node):
    """STM32 底盘串口桥接 ROS2 节点"""

    def __init__(self):
        super().__init__('stm32_bridge')

        # ── 参数 ──
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyS1'),
                ('baud_rate', 115200),
                ('wheel_circumference', 0.241),
                ('wheel_base', 0.35),
                ('counts_per_rev', 1560),
                ('max_linear_vel', 0.5),
                ('max_angular_vel', 2.0),
                ('cmd_timeout_ms', 200),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('publish_rate', 50.0),
                ('cmd_rate', 100.0),
            ]
        )

        # ── 串口 ──
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.get_logger().info(f'打开串口: {port} @ {baud}')
        try:
            self.ser = serial.Serial(port, baud, timeout=0.001)
        except Exception as e:
            self.get_logger().error(f'无法打开串口 {port}: {e}')
            raise

        # ── 协议 & 里程计 ──
        self.proto = SerialProtocol()
        wc = self.get_parameter('wheel_circumference').value
        wb = self.get_parameter('wheel_base').value
        cpr = self.get_parameter('counts_per_rev').value
        self.odom_comp = OdometryComputer(wc, wb, cpr)
        self.calib = CalibrationMonitor(wb)

        # ── 发布者 ──
        self.odom_pub = self.create_publisher(Odometry, '/odom', QOS_SENSOR)
        self.imu_pub  = self.create_publisher(Imu, '/imu', QOS_SENSOR)
        self.status_pub = self.create_publisher(
            ChassisStatus, '/chassis_status', QOS_STATUS)

        # ── 订阅者 ──
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_callback, 10)

        # ── 服务 ──
        self.calib_srv = self.create_service(
            CalibrateWheelBase, '/calibrate_wheel_base', self._calib_callback)

        # ── TF 广播 ──
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 速度指令缓存 ──
        self._last_cmd_time = self.get_clock().now()
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._cmd_timeout = rclpy.duration.Duration(
            seconds=self.get_parameter('cmd_timeout_ms').value / 1000.0)

        # ── 定时器 ──
        period = 1.0 / self.get_parameter('publish_rate').value
        self.read_timer = self.create_timer(period, self._read_loop)

        cmd_period = 1.0 / self.get_parameter('cmd_rate').value
        self.cmd_timer = self.create_timer(cmd_period, self._send_vel_loop)

        # ── 状态跟踪 ──
        self._seq = 0
        self._lost_frames = 0
        self._total_frames = 0
        self._last_status = None

        self.get_logger().info('STM32 Bridge 节点已启动')

    # ============================================================
    # 订阅回调
    # ============================================================

    def _cmd_callback(self, msg: Twist):
        """接收 /cmd_vel 速度指令"""
        max_lv = self.get_parameter('max_linear_vel').value
        max_av = self.get_parameter('max_angular_vel').value

        self._linear_x = max(-max_lv, min(max_lv, msg.linear.x))
        self._angular_z = max(-max_av, min(max_av, msg.angular.z))
        self._last_cmd_time = self.get_clock().now()

    # ============================================================
    # 标定服务
    # ============================================================

    def _calib_callback(self, request, response):
        """处理 /calibrate_wheel_base 服务请求"""
        if request.start:
            self.calib.start()
            self.get_logger().info(
                '轮距标定已开始 — 请遥控小车原地旋转 (建议正反转各几圈, 累计 ≥360°)')
            response.success = True
            response.message = (
                f'标定已开始 (当前 wheel_base={self.calib.wheel_base_nominal:.3f}m), '
                '请原地旋转小车, 完成后再次调用此服务 (start=false)')
            response.calibrated_wheel_base = 0.0
            response.correction_factor = 0.0
            response.sample_count = 0
            response.encoder_yaw_deg = 0.0
            response.gyro_yaw_deg = 0.0
        else:
            ok, msg = self.calib.stop()
            if ok:
                self.get_logger().info(
                    f'轮距标定完成: wheel_base={self.calib.result_wheel_base:.4f}m '
                    f'(修正系数 {self.calib.result_factor:.4f}, '
                    f'原值 {self.calib.wheel_base_nominal:.3f}m)')
            else:
                self.get_logger().warn(f'轮距标定失败: {msg}')
            response.success = ok
            response.message = msg
            response.calibrated_wheel_base = self.calib.result_wheel_base or 0.0
            response.correction_factor = self.calib.result_factor or 0.0
            response.sample_count = self.calib.sample_count
            response.encoder_yaw_deg = math.degrees(self.calib.encoder_yaw_total)
            response.gyro_yaw_deg = math.degrees(self.calib.gyro_yaw_total)
        return response

    # ============================================================
    # 发送循环 (100Hz)
    # ============================================================

    def _send_vel_loop(self):
        """周期性发送速度指令, 超时则发送零速, 含电机补偿"""
        now = self.get_clock().now()
        if now - self._last_cmd_time > self._cmd_timeout:
            self._linear_x = 0.0
            self._angular_z = 0.0

        linear  = self._linear_x
        angular = self._angular_z

        frame = self.proto.encode_vel_cmd(linear, angular)
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'串口写入失败: {e}')

    # ============================================================
    # 读取循环 (50Hz)
    # ============================================================

    def _read_loop(self):
        """读取串口数据, 解析帧, 发布话题

        注意: 不使用 in_waiting + read(n) 模式, 因为两者之间存在竞态条件,
        会导致 "reports readiness but returned no data" 错误.
        直接用 read() 读取, 空返回说明本周期无数据, 不报错.
        """
        try:
            data = self.ser.read(256)
            if data:
                self._process_data(data)
        except serial.SerialException as e:
            self.get_logger().error(f'串口读取失败: {e}')

    def _process_data(self, data: bytes):
        """处理接收到的原始字节流"""
        # 循环解析帧, 直到数据耗尽
        offset = 0
        while offset < len(data):
            result = self.proto.decode_frame(data[offset:])
            if result is None:
                # 跳到下一个可能的帧头位置
                idx = data.find(self.proto.HEADER, offset + 1)
                if idx < 0:
                    break
                offset = idx
                continue

            frame_type, seq, payload = result
            self._total_frames += 1
            frame_len = 7 + len(payload)  # 头(2)+长度(1)+类型(1)+序号(1)+负载+CRC(2)
            offset += frame_len

            # 分发处理
            if frame_type == self.proto.TYPE_ODOM_DATA:
                self._handle_odom(payload, seq)
            elif frame_type == self.proto.TYPE_STATUS:
                self._handle_status(payload, seq)
            elif frame_type == self.proto.TYPE_ERROR:
                self._handle_error(payload, seq)

    # ============================================================
    # 帧处理
    # ============================================================

    def _handle_odom(self, payload: bytes, seq: int):
        """处理 ODOM_DATA 帧 → 发布 /odom, /imu, /tf"""
        d = self.proto.decode_odom_data(payload)
        if d is None:
            return

        now = self.get_clock().now().to_msg()

        # ── 里程计 ──
        dt = 1.0 / self.get_parameter('publish_rate').value
        vx, vth, dyaw = self.odom_comp.update(
            d['left_enc'], d['right_enc'], dt,
            gyro_z_dps=d['gyro_z_dps'])  # 陀螺角速度 → 免疫轮子打滑

        # ── 轮距标定数据采集 ──
        if self.calib.active:
            self.calib.feed(
                dyaw,                              # 编码器 yaw 增量 (rad)
                d['gyro_z_dps'],                   # 陀螺 Z (°/s)
                dt,
                self.odom_comp.last_d_left_m,      # 左轮位移 (m)
                self.odom_comp.last_d_right_m)      # 右轮位移 (m)

        q = quaternion_from_yaw(self.odom_comp.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.get_parameter('odom_frame').value
        odom_msg.child_frame_id = self.get_parameter('base_frame').value
        odom_msg.pose.pose.position.x = self.odom_comp.x
        odom_msg.pose.pose.position.y = self.odom_comp.y
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth
        # 协方差 (编码器里程计典型值)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.05  # yaw
        self.odom_pub.publish(odom_msg)

        # ── TF (odom → base_link) ──
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.get_parameter('odom_frame').value
        tf_msg.child_frame_id = self.get_parameter('base_frame').value
        tf_msg.transform.translation.x = self.odom_comp.x
        tf_msg.transform.translation.y = self.odom_comp.y
        tf_msg.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        # ── IMU ──
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = d['gyro_z_dps']
        imu_msg.linear_acceleration.x = d['accel_x']
        imu_msg.linear_acceleration.y = d['accel_y']
        imu_msg.orientation_covariance[0] = -1.0  # 无姿态估计
        self.imu_pub.publish(imu_msg)

    def _handle_status(self, payload: bytes, seq: int):
        """处理 STATUS 帧 → 发布 /chassis_status"""
        d = self.proto.decode_status(payload)
        if d is None:
            return

        self._last_status = d

        msg = ChassisStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.motor_enabled = d['motor_state'] & 0x0F
        msg.emergency_stop = d['emergency_stop']
        msg.collision_front = d['collision_front']
        msg.collision_rear = d['collision_rear']
        msg.error_code = d['error_code']
        msg.lost_frames = self._lost_frames
        self.status_pub.publish(msg)

    def _handle_error(self, payload: bytes, seq: int):
        """处理 ERROR 帧"""
        code = self.proto.decode_error(payload)
        err_names = []
        if code & self.proto.ERR_ESTOP:
            err_names.append('ESTOP')
        if code & self.proto.ERR_FRONT_COLLISION:
            err_names.append('FRONT_COLLISION')
        if code & self.proto.ERR_REAR_COLLISION:
            err_names.append('REAR_COLLISION')
        if code & self.proto.ERR_COMM_TIMEOUT:
            err_names.append('COMM_TIMEOUT')
        if code & self.proto.ERR_IMU_FAULT:
            err_names.append('IMU_FAULT')
        self.get_logger().error(f'底盘错误: 0x{code:04X} ({", ".join(err_names)})')


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
