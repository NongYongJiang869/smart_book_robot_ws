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
               dt: float, gyro_z_dps: float = None) -> Tuple[float, float, float]:
        """
        更新里程计

        Args:
            left_enc:    左轮编码器累计值 (脉冲数)
            right_enc:   右轮编码器累计值
            dt:          采样间隔 (秒)
            gyro_z_dps:  陀螺 Z 角速度 (°/s), 如果提供则用于角速度,
                         避免 skid-steering 侧滑造成的角度误差

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

        # 线速度: 始终由编码器推算 (受滑移影响但程度较轻)
        d_center = (d_left_m + d_right_m) / 2.0

        # 角速度: 优先使用陀螺仪 (不受轮子打滑影响)
        if gyro_z_dps is not None:
            d_yaw = math.radians(gyro_z_dps) * dt
        else:
            d_yaw = (d_right_m - d_left_m) / self.wheel_base

        # 更新位姿
        self.x += d_center * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_center * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw += d_yaw

        # 瞬时速度
        vx = d_center / dt
        vth = d_yaw / dt

        # 保存轮位移 (供 CalibrationMonitor 使用)
        self.last_d_left_m = d_left_m
        self.last_d_right_m = d_right_m

        return (vx, vth, d_yaw)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """偏航角 → 四元数 (仅绕Z轴旋转)"""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CalibrationMonitor:
    """轮距自动标定 — 对比编码器偏航与陀螺仪偏航来修正 wheel_base

    嵌入在 bridge_node 中使用, 通过 /calibrate_wheel_base 服务控制。
    标定期间不影响正常的 /odom 发布和 /cmd_vel 接收。

    原理:
      编码器偏航 = (d_right - d_left) / wheel_base_nominal
      陀螺仪偏航 = ∫ gyro_z dt  (更接近真实偏航)
      wheel_base_real = wheel_base_nominal × |编码器偏航累计| / |陀螺仪偏航累计|
    """

    MIN_SAMPLES  = 30          # 最少有效样本数
    MIN_GYRO_YAW = 180.0       # 最少陀螺偏航 (°)

    def __init__(self, wheel_base_nominal: float):
        self.wheel_base_nominal = wheel_base_nominal
        self.active = False
        self._reset()

    def _reset(self):
        self.encoder_yaw_total = 0.0
        self.gyro_yaw_total = 0.0
        self.sample_count = 0
        self.last_used = False
        self.result_wheel_base = None  # type: Optional[float]
        self.result_factor = None      # type: Optional[float]

    def start(self):
        self.active = True
        self._reset()

    def stop(self) -> Tuple[bool, str]:
        """结束标定, 返回 (success, message)"""
        self.active = False
        gyro_deg = abs(math.degrees(self.gyro_yaw_total))
        if self.sample_count < self.MIN_SAMPLES:
            return (False,
                    f"样本不足 ({self.sample_count}/{self.MIN_SAMPLES}), 请重新标定")
        if gyro_deg < self.MIN_GYRO_YAW:
            return (False,
                    f"旋转角度不足 ({gyro_deg:.0f}°/{self.MIN_GYRO_YAW}°), 请多转几圈")
        if abs(self.gyro_yaw_total) < 0.001:
            return (False, "陀螺仪未检测到旋转")
        ratio = abs(self.encoder_yaw_total / self.gyro_yaw_total)
        self.result_factor = ratio
        self.result_wheel_base = self.wheel_base_nominal * ratio
        return (True,
                f"标定完成: wheel_base = {self.result_wheel_base:.4f}m "
                f"(修正系数 {ratio:.4f}, 样本 {self.sample_count})")

    def feed(self, d_yaw_enc: float, gyro_z_dps: float, dt: float,
             d_left_m: float, d_right_m: float) -> bool:
        """输入一帧数据, 自动判断是否为有效旋转并累计"""
        if not self.active:
            self.last_used = False
            return False

        dl, dr = abs(d_left_m), abs(d_right_m)
        total = dl + dr
        if total < 0.0001:
            self.last_used = False
            return False

        # 中心位移 < 30% 总位移 → 纯旋转
        center_ratio = abs(d_left_m + d_right_m) / total
        if center_ratio > 0.3:
            self.last_used = False
            return False

        self.encoder_yaw_total += d_yaw_enc
        self.gyro_yaw_total += math.radians(gyro_z_dps) * dt
        self.sample_count += 1
        self.last_used = True
        return True
