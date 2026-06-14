"""
里程计计算 — 从编码器 + IMU 数据推算位姿增量

差速驱动运动学 (设计文档 02_hardware_architecture.md §5):
  v = (v_left + v_right) / 2      线速度 (m/s)
  ω = (v_right - v_left) / L      角速度 (rad/s)

编码器 → 轮速:
  distance = encoder_delta × wheel_circumference / cpr
  v_wheel = distance / dt

TF 发布:
  odom → base_link (50Hz)
"""

import math
from typing import Tuple


class OdometryComputer:
    """差速驱动机器人里程计"""

    def __init__(self, wheel_circumference: float, wheel_base: float,
                 counts_per_rev: int):
        """
        Args:
            wheel_circumference: 轮子周长 (m)
            wheel_base: 左右轮间距 (m)
            counts_per_rev: 编码器每圈脉冲数 (4倍频后)
        """
        self.wheel_circumference = wheel_circumference
        self.wheel_base = wheel_base
        self.counts_per_rev = counts_per_rev

        # 位姿累计
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 上一帧编码器值
        self._last_left_enc = 0
        self._last_right_enc = 0
        self._initialized = False

    @property
    def pose(self) -> Tuple[float, float, float]:
        """返回 (x, y, yaw) 累计位姿"""
        return (self.x, self.y, self.yaw)

    def reset(self):
        """重置里程计"""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._initialized = False

    def update(self, left_enc: int, right_enc: int,
               dt: float) -> Tuple[float, float, float]:
        """
        更新里程计

        Args:
            left_enc:  左轮编码器累计值 (脉冲数)
            right_enc: 右轮编码器累计值
            dt:        采样间隔 (秒)

        Returns:
            (vx, vth, delta_yaw) — 线速度, 角速度, 角度增量
        """
        if not self._initialized:
            self._last_left_enc = left_enc
            self._last_right_enc = right_enc
            self._initialized = True
            return (0.0, 0.0, 0.0)

        # 编码器增量
        d_left = left_enc - self._last_left_enc
        d_right = right_enc - self._last_right_enc
        self._last_left_enc = left_enc
        self._last_right_enc = right_enc

        if dt <= 0.0:
            return (0.0, 0.0, 0.0)

        # 脉冲 → 距离
        dist_per_pulse = self.wheel_circumference / self.counts_per_rev
        d_left_m = d_left * dist_per_pulse
        d_right_m = d_right * dist_per_pulse

        # 差速运动学
        d_center = (d_left_m + d_right_m) / 2.0
        d_yaw = (d_right_m - d_left_m) / self.wheel_base

        # 更新位姿
        self.x += d_center * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_center * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw += d_yaw

        # 瞬时速度
        vx = d_center / dt
        vth = d_yaw / dt

        return (vx, vth, d_yaw)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """偏航角 → 四元数 (仅绕Z轴旋转)"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
