#!/usr/bin/env python3
"""
底盘里程计测试工具 — 接收 STM32 串口数据, 实时计算并显示轮式里程计

用途:
  1. 验证编码器符号是否正确 (前进时两侧 delta 是否都为正)
  2. 观察里程计漂移情况 (静止时 x/y/yaw 是否稳定)
  3. 对比编码器里程 vs 实际位移, 标定轮子周长/轴距

用法:
  python3 tools/odometry_test.py                     # 自动检测串口
  python3 tools/odometry_test.py -p /dev/ttyS1       # 指定串口
  python3 tools/odometry_test.py --show-raw          # 同时显示原始编码器值
  python3 tools/odometry_test.py --csv log.csv       # 记录到 CSV

操作:
  推动小车前进/后退/转弯, 观察 odom 输出是否符合预期。
  前进时左侧 d_left 和右侧 d_right 都应为正值。

键盘快捷键:
  r     重置里程计 (x/y/yaw 归零)
  c     轮距标定 (开始/结束原地旋转标定)
  q     退出程序
  Space 暂停/恢复显示刷新
"""

import argparse
import csv
import math
import os
import select
import signal
import struct
import sys
import termios
import time
from datetime import datetime
from typing import Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 需要 pyserial, 请执行: pip3 install pyserial")
    sys.exit(1)

# ============================================================
# 协议常量 (与 chassis/src/protocol.h 一致)
# ============================================================

HEADER = b'\x5A\xA5'
HEADER0, HEADER1 = 0x5A, 0xA5

# ── 运动学参数 (与 stm32_params.yaml 一致) ──
WHEEL_CIRCUMFERENCE = 0.241    # 轮子周长 (m) — 走80cm标定
WHEEL_BASE          = 0.269     # 左右轮间距 (m) — 已标定
COUNTS_PER_REV      = 1560     # 编码器每圈脉冲数 (4倍频后)

DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV

# 速度显示限幅
MAX_DISPLAY_VX  = 0.6
MAX_DISPLAY_VTH = 1.2

# 缓冲区上限
MAX_BUF_SIZE = 4096

# ANSI 转义 (用 \r\n 确保终端正确处理换行)
CSI = '\033['

def sgr(*codes) -> str:            # Select Graphic Rendition (颜色/样式)
    return f"{CSI}{';'.join(map(str, codes))}m"

# 样式
RST   = sgr(0)
BOLD  = sgr(1)
DIM   = sgr(2)
GREEN = sgr(32)
YELLOW= sgr(33)
RED   = sgr(31)
CYAN  = sgr(36)
MAG   = sgr(35)
WHITE = sgr(37)

# 光标控制
SAVE_CUR    = f"{CSI}s"      # 保存光标位置
RESTORE_CUR = f"{CSI}u"      # 恢复光标位置
HIDE_CUR    = f"{CSI}?25l"
SHOW_CUR    = f"{CSI}?25h"
CLEAR_LINE  = f"{CSI}2K"
CLEAR_BELOW = f"{CSI}0J"     # 清除光标到屏幕末尾

# 行尾统一用 \r\n (兼容 raw/cooked 终端)
NL = '\r\n'


# ============================================================
# CRC16
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021)"""
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ============================================================
# 帧解析
# ============================================================

class FrameParser:
    """增量式帧解析器"""

    def __init__(self):
        self.buf = bytearray()
        self.total_odom  = 0
        self.bad_crc     = 0
        self.bad_len     = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        if len(self.buf) > MAX_BUF_SIZE:
            self.buf = self.buf[-MAX_BUF_SIZE//2:]

    def next_frame(self) -> Optional[Tuple[int, int, bytes, int]]:
        """返回 (ftype, seq, payload, total_len) 或 None"""
        while True:
            # 搜索帧头
            idx = -1
            for i in range(len(self.buf) - 1):
                if self.buf[i] == HEADER0 and self.buf[i+1] == HEADER1:
                    idx = i
                    break
            if idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return None
            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 7:
                return None

            length = self.buf[2]
            total_len = 2 + 1 + length

            if length < 4:     # type(1)+seq(1)+crc(2) 最低 4
                del self.buf[:2]
                self.bad_len += 1
                continue

            if len(self.buf) < total_len:
                return None

            # 帧体 = [len][type][seq][payload...][crcL][crcH]
            frame_body = self.buf[2 : 2+1+length]
            crc_pos = 3 + (length - 4)
            check_range = frame_body[1:crc_pos]
            crc_rx = frame_body[crc_pos] | (frame_body[crc_pos+1] << 8)
            crc_calc = crc16_ccitt(check_range)

            ftype = frame_body[1]
            seq   = frame_body[2]
            payload = bytes(frame_body[3:crc_pos])

            del self.buf[:total_len]

            if crc_rx != crc_calc:
                self.bad_crc += 1
                continue

            if ftype == 0x01:
                self.total_odom += 1

            return (ftype, seq, payload, total_len)


def parse_odom(payload: bytes) -> Optional[dict]:
    """解析 ODOM_DATA (0x01), 陀螺/加速度为原始LSB"""
    if len(payload) < 24:
        return None
    le, re, lv, rv, gyro, ax, ay, ts = struct.unpack('<iiffhhhH', payload)
    return {
        'left_enc':  le,
        'right_enc': re,
        'left_v':    lv,
        'right_v':   rv,
        'gyro_z':    gyro / 131.0,             # LSB → °/s
        'accel_x':   ax / 16384.0 * 9.807,     # LSB → m/s²
        'accel_y':   ay / 16384.0 * 9.807,
        'ts':        ts,
    }


# ============================================================
# 里程计计算
# ============================================================

class Odometry:
    """差速驱动里程计 + IMU 独立显示 (不融合)"""

    def __init__(self):
        # ── 轮式里程计位姿 ──
        self.x  = 0.0
        self.y  = 0.0
        self.yaw = 0.0
        self._le = 0
        self._re = 0
        self._init = False

        self.raw_le  = 0
        self.raw_re  = 0
        self.dl       = 0
        self.dr       = 0
        self.dl_m     = 0.0
        self.dr_m     = 0.0
        self.dc       = 0.0
        self.dyaw     = 0.0
        self.vx       = 0.0
        self.vth      = 0.0
        self.ts       = 0

        # ── IMU 原始值 (不参与融合, 仅显示) ──
        self.gyro_z   = 0.0    # Z轴角速度 (°/s)
        self.accel_x  = 0.0    # X轴加速度 (g)
        self.accel_y  = 0.0    # Y轴加速度 (g)

        # ── 陀螺仪独立积分 (与编码器 yaw 对比用) ──
        self.gyro_yaw = 0.0    # 陀螺仪累计偏航 (°)
        self._gyro_ts = 0

    def update(self, d: dict) -> None:
        le = d['left_enc']
        re = d['right_enc']
        ts = d['ts']

        # ── 存储 IMU 原始值 ──
        self.gyro_z  = d['gyro_z']
        self.accel_x = d['accel_x']
        self.accel_y = d['accel_y']

        # ── 陀螺仪独立积分 ──
        if self._init:
            dt_gyro = ((ts - self._gyro_ts) & 0xFFFF) / 1000.0
            if 0 < dt_gyro < 0.5:   # 合理范围内才积分
                self.gyro_yaw += self.gyro_z * dt_gyro
        self._gyro_ts = ts

        # ── 轮式里程计 ──
        if not self._init:
            self._le = le
            self._re = re
            self.ts  = ts
            self._init = True
            return

        self.dl = le - self._le
        self.dr = re - self._re
        self._le = le
        self._re = re

        self.raw_le = le
        self.raw_re = re

        self.dl_m = self.dl * DIST_PER_PULSE
        self.dr_m = self.dr * DIST_PER_PULSE

        self.dc   = (self.dl_m + self.dr_m) / 2.0
        self.dyaw = (self.dr_m - self.dl_m) / WHEEL_BASE

        hdy = self.dyaw / 2.0
        self.x   += self.dc * math.cos(self.yaw + hdy)
        self.y   += self.dc * math.sin(self.yaw + hdy)
        self.yaw += self.dyaw
        self.yaw  = (self.yaw + math.pi) % (2*math.pi) - math.pi

        dt = ((ts - self.ts) & 0xFFFF) / 1000.0
        if dt <= 0:
            dt = 0.02
        self.vx  = self.dc / dt
        self.vth = self.dyaw / dt
        self.ts  = ts

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.gyro_yaw = 0.0
        self._init = False


# ============================================================
# 轮距自动标定
# ============================================================

class CalibrationMonitor:
    """轮距自动标定 — 对比编码器偏航与陀螺仪偏航来修正 wheel_base

    使用方法:
      1. 将小车放在实际运行地面上
      2. 按 'c' 开始标定
      3. 遥控小车原地旋转 (建议正反转各几圈, 累计 ≥360°)
      4. 按 'c' 结束标定, 查看建议的 wheel_base 值
      5. 将结果更新到 stm32_params.yaml 的 wheel_base 参数

    原理:
      编码器偏航 = (d_right - d_left) / wheel_base_nominal
      陀螺仪偏航 = ∫ gyro_z dt  (更接近真实偏航)
      wheel_base_real = wheel_base_nominal × |编码器偏航累计| / |陀螺仪偏航累计|

    侧滑越严重, 标定后的 wheel_base 会越大于几何轮距。
    """

    MIN_SAMPLES   = 30          # 最少有效样本数
    MIN_GYRO_YAW  = 180.0       # 最少陀螺偏航 (°) — 至少转半圈才算有效

    def __init__(self, wheel_base_nominal: float):
        self.wheel_base_nominal = wheel_base_nominal
        self.active = False
        self.encoder_yaw_total = 0.0   # 编码器偏航累计 (rad)
        self.gyro_yaw_total = 0.0      # 陀螺仪偏航累计 (rad)
        self.sample_count = 0
        self.last_used = False         # 上一帧是否被采集
        self.result_wheel_base = None  # type: Optional[float]
        self.result_factor = None      # type: Optional[float]

    def start(self):
        self.active = True
        self.encoder_yaw_total = 0.0
        self.gyro_yaw_total = 0.0
        self.sample_count = 0
        self.last_used = False
        self.result_wheel_base = None
        self.result_factor = None

    def stop(self):
        self.active = False
        gyro_deg = abs(math.degrees(self.gyro_yaw_total))
        if self.sample_count >= self.MIN_SAMPLES and gyro_deg >= self.MIN_GYRO_YAW:
            if abs(self.gyro_yaw_total) > 0.001:
                ratio = abs(self.encoder_yaw_total / self.gyro_yaw_total)
            else:
                ratio = 1.0
            self.result_factor = ratio
            self.result_wheel_base = self.wheel_base_nominal * ratio

    def feed(self, d_yaw_enc: float, gyro_z_dps: float, dt: float,
             d_left_m: float, d_right_m: float) -> bool:
        """输入一帧数据, 自动判断是否为有效旋转并累计

        Args:
            d_yaw_enc:   编码器偏航增量 (rad, 已用 nominal wheel_base 计算)
            gyro_z_dps:  陀螺 Z 轴角速度 (°/s)
            dt:          采样间隔 (s)
            d_left_m:    左轮位移 (m)
            d_right_m:   右轮位移 (m)

        Returns:
            True 如果本帧被采集
        """
        if not self.active:
            self.last_used = False
            return False

        # 判断是否为"纯旋转": 左右轮反向且线速度分量小
        dl, dr = abs(d_left_m), abs(d_right_m)
        total = dl + dr
        if total < 0.0001:
            self.last_used = False
            return False

        # 中心位移占比 < 30% → 认为在旋转
        center_ratio = abs(d_left_m + d_right_m) / total
        if center_ratio > 0.3:
            self.last_used = False
            return False

        self.encoder_yaw_total += d_yaw_enc
        self.gyro_yaw_total += math.radians(gyro_z_dps) * dt
        self.sample_count += 1
        self.last_used = True
        return True

    def progress_deg(self) -> float:
        """陀螺仪累计偏航 (°) — 用于显示进度"""
        return math.degrees(self.gyro_yaw_total)

    def progress_ratio(self) -> float:
        """已完成比例 (0~1), 基于最少角度要求"""
        return min(abs(self.progress_deg()) / self.MIN_GYRO_YAW, 1.0)


# ============================================================
# 显示
# ============================================================

def _bar(val: float, width: int, max_abs: float) -> str:
    """水平进度条"""
    if max_abs <= 0:
        max_abs = 1.0
    n = min(int(abs(val) / max_abs * width), width)
    if n == 0:
        return f"{DIM}{'·' * width}{RST}"
    color = GREEN if val > 0 else RED
    return f"{color}{'█'*n}{'░'*(width-n)}{RST}"


def _deg(rad: float) -> str:
    return f"{math.degrees(rad):+7.1f}°"


def _clamp_width(text: str, width: int) -> str:
    """裁剪字符串到显示宽度 (ANSI码不计入宽度)"""
    # 简单处理: 去掉 ANSI 序列后计算长度, 空格填充
    import re
    visible = re.sub(r'\033\[[0-9;]*m', '', text)
    if len(visible) > width:
        return text[:width]  # rough cut
    return text


W = 78  # 终端宽度假设 (避免折行)


def draw_header() -> None:
    """打印不可滚动的表头"""
    sys.stdout.write(NL)
    sys.stdout.write(f"{BOLD}{CYAN}{'═'*W}{RST}{NL}")
    sys.stdout.write(f"{BOLD}{CYAN}  里程计测试工具{' '*(W-10)}{RST}{NL}")
    sys.stdout.write(f"{BOLD}{CYAN}{'═'*W}{RST}{NL}")
    sys.stdout.write(f"  轮周长={WHEEL_CIRCUMFERENCE:.3f}m  轴距={WHEEL_BASE:.2f}m  "
                     f"CPR={COUNTS_PER_REV}  每脉冲={DIST_PER_PULSE*1000:.2f}mm{NL}")
    sys.stdout.write(f"{CYAN}{'─'*W}{RST}{NL}")
    sys.stdout.write(f"  推动小车 → 观察里程计。前进时 ΔL/ΔR 都应为正。{NL}")
    sys.stdout.write(f"  {BOLD}r{RST}=重置  {BOLD}c{RST}=标定  {BOLD}q{RST}=退出  {BOLD}Space{RST}=暂停/恢复{NL}")
    sys.stdout.write(f"{CYAN}{'─'*W}{RST}{NL}")
    sys.stdout.write(NL)  # 数据区从下一行开始
    sys.stdout.write(SAVE_CUR)  # 记住数据区起始位置
    sys.stdout.flush()


def draw_frame(odom: Odometry, show_raw: bool, paused: bool,
               calib: 'Optional[CalibrationMonitor]' = None) -> None:
    """重绘数据区 (从 SAVE_CUR 位置开始, 清除到末尾)"""
    out = []
    out.append(RESTORE_CUR + CLEAR_BELOW)

    # ── 编码器行 ──
    if show_raw:
        out.append(f"  {DIM}ENC_L{RST} {odom.raw_le:+8d}   "
                   f"{DIM}ENC_R{RST} {odom.raw_re:+8d}   "
                   f"{DIM}TS{RST} {odom.ts:5d}ms{NL}")
    else:
        out.append(NL)

    # ── Δ 编码器 ──
    cl = GREEN if odom.dl >= 0 else RED
    cr = GREEN if odom.dr >= 0 else RED
    out.append(f"  {DIM}ΔL{RST} {cl}{odom.dl:+8d}{RST}   "
               f"{DIM}ΔR{RST} {cr}{odom.dr:+8d}{RST}   "
               f"{DIM}(正向 → 两者 +){RST}{NL}")

    # ── 轮位移 mm ──
    out.append(f"  {DIM}左轮{RST} {odom.dl_m*1000:+8.2f}mm   "
               f"{DIM}右轮{RST} {odom.dr_m*1000:+8.2f}mm{NL}")

    # ── 条形图 ──
    bl = _bar(odom.dl_m, 25, DIST_PER_PULSE*200)
    br = _bar(odom.dr_m, 25, DIST_PER_PULSE*200)
    out.append(f"  [{bl}] L{NL}")
    out.append(f"  [{br}] R{NL}")

    # ── Δ 中心/yaw ──
    out.append(f"  {DIM}Δ中心{RST} {odom.dc*1000:+8.2f}mm   "
               f"{DIM}Δ偏航{RST} {_deg(odom.dyaw)}  "
               f"{DIM}({math.degrees(odom.dyaw):+.2f}°){RST}{NL}")

    out.append(NL)

    # ── 累计位姿 (轮式编码器) ──
    out.append(f"  {BOLD}{CYAN}位姿累计 (编码器){RST}{NL}")
    out.append(f"    X {GREEN}{odom.x:+8.3f}m{RST}   "
               f"Y {GREEN}{odom.y:+8.3f}m{RST}   "
               f"θ {MAG}{_deg(odom.yaw)}{RST}   "
               f"距离 {math.hypot(odom.x, odom.y):.3f}m{NL}")

    # ── 速度 ──
    bx = _bar(odom.vx,  20, MAX_DISPLAY_VX)
    bt = _bar(odom.vth, 20, MAX_DISPLAY_VTH)
    out.append(f"  {DIM}线速度{RST} {odom.vx:+6.3f} m/s  {bx}{NL}")
    out.append(f"  {DIM}角速度{RST} {odom.vth:+6.3f} rad/s {bt}{NL}")

    out.append(NL)

    # ══════════════════════════════════════════════════════════════
    # IMU 传感器 (独立显示, 不参与位姿融合)
    #
    # 单位说明:
    #   Gyro Z  = Z轴角速度 (°/s), 正值=左转, 直行/静止时应 ≈0
    #   Gyro Yaw = ∫gyro_z·dt 累计偏航 (°), 与编码器 yaw 对比看漂移
    #   Accel    = 加速度 (m/s²), 1g≈9.8m/s², 静止水平时应 X≈0,Y≈0
    # ══════════════════════════════════════════════════════════════
    out.append(f"  {BOLD}{MAG}─── IMU 传感器 (独立, 未融合) ───{RST}{NL}")

    # 陀螺仪 Z轴角速度
    # 正常直行/静止时 gyro_z ≈ 0; 偏差大说明陀螺零偏未校准好
    gz_bar = _bar(odom.gyro_z, 25, 60.0)  # ±60°/s
    gz_warn = f" {YELLOW}⚠ 零偏大{RST}" if abs(odom.gyro_z) > 5 else ""
    out.append(f"  {DIM}Gyro Z (角速度){RST} {odom.gyro_z:+8.1f}°/s  {gz_bar}{gz_warn}{NL}")

    # 陀螺仪积分偏航 vs 编码器偏航
    # 差值小 = 陀螺/编码一致; 差值大 = 陀螺漂移或轮子打滑
    gyaw_diff = odom.gyro_yaw - math.degrees(odom.yaw)
    if abs(gyaw_diff) < 5:
        diff_color, diff_note = GREEN, ""
    elif abs(gyaw_diff) < 15:
        diff_color, diff_note = YELLOW, f" {YELLOW}⚠{RST}"
    else:
        diff_color, diff_note = RED, f" {RED}✗ 严重漂移{RST}"
    out.append(f"  {DIM}Gyro Yaw (积分){RST} {odom.gyro_yaw:+8.1f}°  "
               f"vs 编码器 {math.degrees(odom.yaw):+8.1f}°  "
               f"Δ={diff_color}{gyaw_diff:+5.1f}°{RST}{diff_note}{NL}")

    # 加速度计 (单位 m/s², 非 g!)
    # 静止水平: X≈0, Y≈0, (Z≈9.8 但未采集)
    out.append(f"  {DIM}Accel (m/s²){RST} X {odom.accel_x:+7.3f}  "
               f"Y {odom.accel_y:+7.3f}  "
               f"{DIM}(静止时应≈0){RST}{NL}")

    out.append(NL)

    # ══════════════════════════════════════════════════════════════
    # 轮距标定 (按 'c' 开始/结束)
    # ══════════════════════════════════════════════════════════════
    if calib is not None:
        if calib.result_wheel_base is not None:
            # 标定完成 — 显示结果
            out.append(f"  {BOLD}{GREEN}═══ 轮距标定结果 ═══{RST}{NL}")
            out.append(f"  {DIM}编码器偏航累计:{RST} {math.degrees(calib.encoder_yaw_total):+8.1f}°   "
                       f"{DIM}陀螺仪偏航累计:{RST} {math.degrees(calib.gyro_yaw_total):+8.1f}°   "
                       f"{DIM}样本:{RST} {calib.sample_count}{NL}")
            out.append(f"  {DIM}修正系数:{RST} {BOLD}{GREEN}{calib.result_factor:.4f}{RST}   "
                       f"{DIM}(编码器/陀螺仪 比值){NL}")
            out.append(NL)
            out.append(f"  {BOLD}{YELLOW}建议 wheel_base: {calib.result_wheel_base:.4f} m{RST}{NL}")
            out.append(f"  {DIM}(原值 {calib.wheel_base_nominal:.3f} m, "
                       f"请更新到 stm32_params.yaml){NL}")
            out.append(NL)
        elif calib.active:
            # 采集中 — 显示实时进度
            prog = calib.progress_ratio()
            bar_w = 30
            n = min(int(prog * bar_w), bar_w)
            if calib.last_used:
                bar = f"{GREEN}{'█'*n}{DIM}{'░'*(bar_w-n)}{RST}"
                status = f"{GREEN}采集中... 正在旋转 ✓{RST}"
            else:
                bar = f"{DIM}{'░'*bar_w}{RST}"
                status = f"{YELLOW}采集中... 等待旋转{RST}  (左右轮反向时自动采集)"
            out.append(f"  {BOLD}{MAG}─── 轮距标定 (按 c 结束) ───{RST}{NL}")
            out.append(f"  [{bar}] {prog*100:3.0f}%{NL}")
            out.append(f"  {status}{NL}")
            out.append(f"  {DIM}编码器偏航:{RST} {math.degrees(calib.encoder_yaw_total):+8.1f}°   "
                       f"{DIM}陀螺仪偏航:{RST} {math.degrees(calib.gyro_yaw_total):+8.1f}°   "
                       f"{DIM}样本:{RST} {calib.sample_count}{NL}")
            gyro_deg = abs(calib.progress_deg())
            if gyro_deg < calib.MIN_GYRO_YAW:
                out.append(f"  {DIM}(至少需要转 {calib.MIN_GYRO_YAW}° 才有效, 当前 {gyro_deg:.0f}°){NL}")
            out.append(NL)

    # ── 运动方向 ──
    if odom.dl > 0 and odom.dr > 0:
        out.append(f"  {GREEN}↑ 前进{RST}{NL}")
    elif odom.dl < 0 and odom.dr < 0:
        out.append(f"  {YELLOW}↓ 后退{RST}{NL}")
    elif odom.dl == 0 and odom.dr == 0:
        out.append(f"  {DIM}■ 静止{RST}{NL}")
    else:
        out.append(f"  {MAG}↻ 旋转{RST}{NL}")

    # ── 状态栏 ──
    pause_tag = f" {BOLD}{YELLOW}⏸ PAUSED{RST} " if paused else ""
    out.append(f"  {DIM}{'─'*W}{RST}{NL}")
    out.append(f"  {pause_tag}{DIM}q=退出  r=重置  c=标定  空格=暂停/恢复{RST}{NL}")

    sys.stdout.write(''.join(out))
    sys.stdout.flush()


# ============================================================
# 非阻塞键盘 (只改输入, 不改输出)
# ============================================================

class KeyboardReader:
    """仅修改终端输入属性: 关闭 canonical 和 echo, 保留输出处理"""

    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._saved = None
        self._active = False

    def start(self) -> None:
        if not os.isatty(self._fd):
            return
        self._saved = termios.tcgetattr(self._fd)
        new = termios.tcgetattr(self._fd)
        # 只改输入相关标志, 保留 [1] (OPOST/ONLCR 等输出标志)
        new[3] &= ~(termios.ICANON | termios.ECHO)   # 关闭行缓冲和回显
        new[6][termios.VMIN]  = 0                     # 非阻塞
        new[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, new)
        self._active = True

    def stop(self) -> None:
        if self._saved is not None and os.isatty(self._fd):
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None
        self._active = False

    def read(self) -> Optional[str]:
        """非阻塞读取一个按键, 无输入返回 None"""
        if not self._active:
            return None
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return os.read(self._fd, 1).decode('utf-8', errors='replace')
        except (OSError, IOError):
            pass
        return None


# ============================================================
# 串口
# ============================================================

def list_ports() -> list:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if os.path.exists(dev) and dev not in ports:
            ports.append(dev)
    return sorted(ports)


def auto_detect(baud: int) -> Optional[str]:
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if not os.path.exists(dev):
            continue
        try:
            ser = serial.Serial(dev, baud, timeout=0.3)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.5:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    if HEADER in data:
                        ser.close()
                        return dev
                time.sleep(0.05)
            ser.close()
        except Exception:
            pass
    return None


# ============================================================
# CSV
# ============================================================

def csv_header(w):
    w.writerow(['time','ts_ms','left_enc','right_enc','d_left','d_right',
                'd_left_mm','d_right_mm','d_center_mm','d_yaw_deg',
                'x','y','yaw_deg','vx','vth',
                'gyro_z_dps','gyro_yaw_deg','accel_x_g','accel_y_g'])

def csv_row(w, odom: Odometry):
    w.writerow([datetime.now().isoformat(), odom.ts,
                odom.raw_le, odom.raw_re, odom.dl, odom.dr,
                odom.dl_m*1000, odom.dr_m*1000, odom.dc*1000,
                math.degrees(odom.dyaw),
                odom.x, odom.y, math.degrees(odom.yaw),
                odom.vx, odom.vth,
                odom.gyro_z, odom.gyro_yaw, odom.accel_x, odom.accel_y])


# ============================================================
# main
# ============================================================

def main():
    ap = argparse.ArgumentParser(description="STM32 底盘里程计测试工具")
    ap.add_argument('-p','--port', default=None, help='串口设备路径')
    ap.add_argument('-b','--baud', type=int, default=115200)
    ap.add_argument('--show-raw', action='store_true', help='显示原始编码器值')
    ap.add_argument('-c','--csv', default=None, help='CSV 输出文件')
    ap.add_argument('-l','--list', action='store_true', help='列出串口')
    args = ap.parse_args()

    if args.list:
        for p in list_ports():
            print(f"  {p}")
        return

    # ── 串口 ──
    port = args.port
    if not port:
        print("自动检测 STM32 串口...", end=' ', flush=True)
        port = auto_detect(args.baud)
        if port:
            print(f"找到 {port}")
        else:
            port = '/dev/ttyS1'
            print(f"未检测到, 默认 {port}")

    # ── CSV ──
    csv_file = None
    csv_w = None
    if args.csv:
        csv_file = open(args.csv, 'w', newline='')
        csv_w = csv.writer(csv_file)
        csv_header(csv_w)

    # ── 串口打开 ──
    print(f"打开 {port} @ {args.baud} ...", end=' ', flush=True)
    try:
        ser = serial.Serial(port, args.baud, timeout=0)
    except Exception as e:
        print(f"失败: {e}")
        if csv_file: csv_file.close()
        sys.exit(1)
    print("OK")

    # ── 组件 ──
    odom    = Odometry()
    calib   = CalibrationMonitor(WHEEL_BASE)
    parser  = FrameParser()
    kb      = KeyboardReader()

    # 隐藏光标, 打印表头, 保存数据区起点
    sys.stdout.write(HIDE_CUR)
    sys.stdout.flush()
    draw_header()

    kb.start()

    paused       = False
    dirty        = False
    need_reset   = False
    last_draw    = 0.0
    DRAW_IVAL    = 1.0 / 30
    running      = True

    def cleanup():
        kb.stop()
        sys.stdout.write(RESTORE_CUR + CLEAR_BELOW + SHOW_CUR + NL)
        sys.stdout.flush()
        try:    ser.close()
        except: pass
        if csv_file: csv_file.close()

    def on_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while running:
            # 1) 读串口
            try:
                n = ser.in_waiting
                if n > 0:
                    data = ser.read(n)
                    if data:
                        parser.feed(data)
            except (serial.SerialException, OSError) as e:
                sys.stdout.write(f"{RESTORE_CUR}{CLEAR_BELOW}{RED}串口错误: {e}{RST}{NL}")
                sys.stdout.flush()
                time.sleep(0.5)
                try:
                    ser.close()
                    ser = serial.Serial(port, args.baud, timeout=0)
                except Exception:
                    pass
                continue

            # 2) 解析帧
            while True:
                r = parser.next_frame()
                if r is None:
                    break
                ftype, seq, payload, _ = r
                if ftype == 0x01:
                    d = parse_odom(payload)
                    if d:
                        prev_ts = odom.ts
                        odom.update(d)
                        # 标定数据采集 (odom.update 之后, dyaw/dl_m/dr_m 已更新)
                        if calib.active:
                            dt_calib = ((d['ts'] - prev_ts) & 0xFFFF) / 1000.0
                            if dt_calib <= 0:
                                dt_calib = 0.02
                            calib.feed(odom.dyaw, odom.gyro_z, dt_calib,
                                       odom.dl_m, odom.dr_m)
                        dirty = True

            # 3) 键盘
            key = kb.read()
            while key is not None:
                if key in ('q', 'Q', '\x03'):
                    running = False
                    break
                elif key in ('r', 'R'):
                    need_reset = True
                elif key in ('c', 'C'):
                    if calib.active:
                        calib.stop()
                    else:
                        calib.start()
                    dirty = True
                elif key == ' ':
                    paused = not paused
                    dirty = True
                key = kb.read()

            if need_reset:
                odom.reset()
                calib = CalibrationMonitor(WHEEL_BASE)
                need_reset = False
                dirty = True

            # 4) 刷新
            now = time.monotonic()
            if dirty and not paused and (now - last_draw >= DRAW_IVAL):
                draw_frame(odom, args.show_raw, paused=False, calib=calib)
                if csv_w:
                    csv_row(csv_w, odom)
                last_draw = now
                dirty = False
            elif paused and dirty:
                # 暂停时只刷一次 (显示暂停标签)
                draw_frame(odom, args.show_raw, paused=True, calib=calib)
                dirty = False

            # 5) 等待
            try:
                fds = [ser.fileno()]
                if kb._active:
                    fds.append(sys.stdin.fileno())
                select.select(fds, [], [], 0.02)
            except (OSError, ValueError):
                time.sleep(0.02)

    finally:
        cleanup()

    # 统计
    print(f"  共处理 {parser.total_odom} 帧 ODOM_DATA", end='')
    if parser.bad_crc or parser.bad_len:
        print(f"  (丢弃: {parser.bad_crc} CRC错, {parser.bad_len} 长度非法)")
    else:
        print()
    print(f"  编码器位姿: x={odom.x:.3f}m  y={odom.y:.3f}m  "
          f"yaw={math.degrees(odom.yaw):.1f}°")
    print(f"  陀螺仪积分: yaw={odom.gyro_yaw:.1f}°  "
          f"(差值 {odom.gyro_yaw - math.degrees(odom.yaw):.1f}°)")
    if args.csv:
        print(f"  CSV → {args.csv}")
    # 标定结果
    if calib.result_wheel_base is not None:
        print()
        print(f"  ═══ 轮距标定结果 ═══")
        print(f"  编码器偏航累计: {math.degrees(calib.encoder_yaw_total):.1f}°")
        print(f"  陀螺仪偏航累计: {math.degrees(calib.gyro_yaw_total):.1f}°")
        print(f"  有效样本数:     {calib.sample_count}")
        print(f"  修正系数:       {calib.result_factor:.4f}")
        print(f"  ★ 建议 wheel_base: {calib.result_wheel_base:.4f} m"
              f"  (原值 {WHEEL_BASE:.3f} m)")
        print(f"    请将 wheel_base 更新到:")
        print(f"      RDKX5/src/stm32_bridge/config/stm32_params.yaml")
        print(f"      tools/odometry_test.py (WHEEL_BASE 常量)")
    elif calib.active:
        print()
        print(f"  ⚠ 标定未完成 (按 c 可继续, 或按 r 重置)")
        gyro_deg = abs(calib.progress_deg())
        if gyro_deg < calib.MIN_GYRO_YAW:
            print(f"    还需旋转至少 {calib.MIN_GYRO_YAW - gyro_deg:.0f}°")


if __name__ == '__main__':
    main()
