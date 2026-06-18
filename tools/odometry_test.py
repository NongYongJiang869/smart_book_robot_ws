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
"""

import argparse
import csv
import math
import os
import signal
import struct
import sys
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

# ── 运动学参数 (与 stm32_params.yaml 一致) ──
WHEEL_CIRCUMFERENCE = 0.478    # 轮子周长 (m)
WHEEL_BASE          = 0.35     # 左右轮间距 (m)
COUNTS_PER_REV      = 1560     # 编码器每圈脉冲数 (4倍频后)

DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / COUNTS_PER_REV  # 每脉冲对应距离 (m)

# 速度显示限幅
MAX_DISPLAY_VX = 0.6    # m/s
MAX_DISPLAY_VTH = 1.2   # rad/s

# ANSI 颜色
C = {
    'R': '\033[0m',     # Reset
    'B': '\033[1m',     # Bold
    'D': '\033[2m',     # Dim
    'G': '\033[32m',    # Green
    'Y': '\033[33m',    # Yellow
    'R': '\033[31m',    # Red
    'C': '\033[36m',    # Cyan
    'M': '\033[35m',    # Magenta
}

# ============================================================
# CRC16
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


# ============================================================
# 帧解析
# ============================================================

def parse_frame(data: bytes) -> Optional[Tuple[int, int, bytes, int]]:
    """
    尝试从 data 中解析一帧。
    返回 (frame_type, seq, payload, frame_length) 或 None。
    """
    idx = data.find(HEADER)
    if idx < 0:
        return None
    if idx > 0:
        data = data[idx:]

    if len(data) < 7:
        return None

    length = data[2]
    total_len = 2 + 1 + length  # header(2) + len_byte(1) + content(length)
    if len(data) < total_len:
        return None

    frame = data[2:2 + 1 + length]
    crc_pos = 3 + (length - 4)
    check_data = frame[1:crc_pos]  # type + seq + payload
    crc_r = struct.unpack_from('<H', frame, crc_pos)[0]
    crc_c = crc16_ccitt(check_data)
    if crc_r != crc_c:
        return None

    ftype = frame[1]
    seq = frame[2]
    payload = frame[3:crc_pos]
    return (ftype, seq, payload, total_len)


def parse_odom(payload: bytes) -> Optional[dict]:
    """解析 ODOM_DATA (0x01) 负载"""
    if len(payload) < 24:
        return None
    le, re, lv, rv, gyro, ax, ay, ts = \
        struct.unpack('<iiffhhhH', payload)
    return {
        'left_enc': le,
        'right_enc': re,
        'left_v': lv,
        'right_v': rv,
        'gyro_z': gyro / 1000.0,
        'accel_x': ax / 1000.0,
        'accel_y': ay / 1000.0,
        'ts': ts,
    }


# ============================================================
# 里程计计算
# ============================================================

class Odometry:
    """差速驱动里程计 (与 RDKX5/.../odometry.py 算法一致)"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_le = 0
        self._last_re = 0
        self._initialized = False

        # 最新帧的原始值和中间结果 (供显示)
        self.raw_left_enc = 0
        self.raw_right_enc = 0
        self.d_left = 0
        self.d_right = 0
        self.d_left_m = 0.0
        self.d_right_m = 0.0
        self.d_center = 0.0
        self.d_yaw = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_ts = 0

    def update(self, data: dict) -> None:
        """输入一帧 ODOM_DATA, 更新里程计"""
        le = data['left_enc']
        re = data['right_enc']
        ts = data['ts']

        if not self._initialized:
            self._last_le = le
            self._last_re = re
            self._initialized = True
            self.last_ts = ts
            return

        # 编码器增量
        self.d_left = le - self._last_le
        self.d_right = re - self._last_re
        self._last_le = le
        self._last_re = re

        self.raw_left_enc = le
        self.raw_right_enc = re

        # 脉冲 → 距离
        self.d_left_m = self.d_left * DIST_PER_PULSE
        self.d_right_m = self.d_right * DIST_PER_PULSE

        # 差速运动学
        self.d_center = (self.d_left_m + self.d_right_m) / 2.0
        self.d_yaw = (self.d_right_m - self.d_left_m) / WHEEL_BASE

        # 累积位姿
        self.x += self.d_center * math.cos(self.yaw + self.d_yaw / 2.0)
        self.y += self.d_center * math.sin(self.yaw + self.d_yaw / 2.0)
        self.yaw += self.d_yaw

        # 瞬时速度 (基于帧间隔)
        dt = ((ts - self.last_ts) & 0xFFFF) / 1000.0  # 考虑回绕
        if dt <= 0:
            dt = 0.02
        self.vx = self.d_center / dt
        self.vth = self.d_yaw / dt
        self.last_ts = ts

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._initialized = False


# ============================================================
# 显示
# ============================================================

def bar(val: float, width: int = 20, max_abs: float = 1.0) -> str:
    """数值 → 彩色进度条"""
    n = int(abs(val) / max_abs * width)
    if n > width:
        n = width
    if n == 0:
        return C['D'] + '·' * width + C['R']
    bar_chars = '█' * n + '░' * (width - n)
    color = C['G'] if val > 0 else C['R']
    return color + bar_chars + C['R']


def deg(rad: float) -> str:
    """弧度 → 带符号角度字符串"""
    return f"{math.degrees(rad):+7.1f}°"


def draw_header():
    """绘制表头"""
    title = "里程计测试工具"
    print(f"\n{C['B']}{C['C']}{'═' * 72}{C['R']}")
    print(f"{C['B']}{C['C']}   {title}  {C['R']}")
    print(f"{C['B']}{C['C']}{'═' * 72}{C['R']}")
    print(f"  参数: 轮周长={WHEEL_CIRCUMFERENCE}m  轴距={WHEEL_BASE}m  编码器CPR={COUNTS_PER_REV}")
    print(f"  每脉冲={DIST_PER_PULSE*1000:.3f}mm  帧率≈50Hz")
    print(f"{C['C']}{'─' * 72}{C['R']}")
    print(f"  推动小车, 观察里程计是否与运动一致。")
    print(f"  前进时 {C['G']}d_left 和 d_right 都应为正{C['R']} | "
          f"后退都应为负 | 左转 d_left > d_right")
    print(f"{C['C']}{'─' * 72}{C['R']}")


def draw_odom(odom: Odometry, show_raw: bool):
    """绘制一帧里程计数据"""
    # 清除上一帧
    print("\033[14A", end="")  # 向上移14行覆盖

    # ── 第1行: 编码器原始值 ──
    if show_raw:
        print(f"  {C['D']}ENC_L:{C['R']} {odom.raw_left_enc:>+10d}  "
              f"{C['D']}ENC_R:{C['R']} {odom.raw_right_enc:>+10d}  "
              f"{C['D']}TS:{C['R']} {odom.last_ts:>5d}ms")
    else:
        print()

    # ── 第2行: 编码器 delta ──
    dl_color = C['G'] if odom.d_left >= 0 else C['R']
    dr_color = C['G'] if odom.d_right >= 0 else C['R']
    print(f"  {C['D']}ΔL:{C['R']} {dl_color}{odom.d_left:+8d}{C['R']}  "
          f"{C['D']}ΔR:{C['R']} {dr_color}{odom.d_right:+8d}{C['R']}  "
          f"{C['D']}(正向运动时两者都应 > 0){C['R']}")

    # ── 第3行: 左右轮位移 (mm) ──
    dlm = odom.d_left_m * 1000
    drm = odom.d_right_m * 1000
    print(f"  {C['D']}左轮位移:{C['R']} {dlm:+8.2f}mm  "
          f"{C['D']}右轮位移:{C['R']} {drm:+8.2f}mm")

    # ── 第4行: 轮速条形图 ──
    l_bar = bar(odom.d_left_m, 25, DIST_PER_PULSE * 200)
    r_bar = bar(odom.d_right_m, 25, DIST_PER_PULSE * 200)
    print(f"  [{l_bar}] L")
    print(f"  [{r_bar}] R")

    # ── 第5行: 中心位移 & 角度增量 ──
    print(f"  {C['D']}Δ中心:{C['R']} {odom.d_center*1000:+8.2f}mm  "
          f"{C['D']}Δ偏航:{C['R']} {deg(odom.d_yaw)}  "
          f"{C['D']}({math.degrees(odom.d_yaw):+.2f}rad)")

    # ── 第6行: 空行 ──
    print()

    # ── 第7行: 累计位姿 ──
    print(f"  {C['B']}{C['C']}位姿累计:{C['R']}")
    print(f"    X:{C['G']}{odom.x:+8.3f}m{C['R']}  "
          f"Y:{C['G']}{odom.y:+8.3f}m{C['R']}  "
          f"偏航:{C['M']}{deg(odom.yaw)}{C['R']}  "
          f"距离原点:{math.hypot(odom.x, odom.y):.3f}m")

    # ── 第8行: 速度 ──
    vx_bar = bar(odom.vx, 20, MAX_DISPLAY_VX)
    vth_bar = bar(odom.vth, 20, MAX_DISPLAY_VTH)
    print(f"  {C['D']}线速度:{C['R']} {odom.vx:+6.3f} m/s  {vx_bar}")
    print(f"  {C['D']}角速度:{C['R']} {odom.vth:+6.3f} rad/s {vth_bar}")

    # ── 第9-13行: 提示 ──
    print()
    if odom.d_left > 0 and odom.d_right > 0:
        print(f"  {C['G']}✓ 两侧编码器同号 (正/正) → 直线运动 (前进){C['R']}")
    elif odom.d_left < 0 and odom.d_right < 0:
        print(f"  {C['Y']}← 两侧编码器同号 (负/负) → 直线运动 (后退){C['R']}")
    else:
        print(f"  {C['M']}↻ 两侧编码器异号 → 旋转运动{C['R']}")
    print(f"  {C['D']}{'─' * 70}{C['R']}")
    print(f"  {C['D']}Ctrl+C 退出  |  r 重置里程计  |  "
          f"前进时 ΔL/ΔR 应都为正, 否则编码器符号需要修正{C['R']}")


def draw_welcome():
    """初始空白帧"""
    print('\n' * 14)


# ============================================================
# CSV 记录
# ============================================================

def csv_header(writer):
    writer.writerow([
        'time', 'ts_ms',
        'left_enc', 'right_enc', 'd_left', 'd_right',
        'd_left_mm', 'd_right_mm', 'd_center_mm', 'd_yaw_deg',
        'x', 'y', 'yaw_deg',
        'vx', 'vth',
    ])


def csv_row(writer, odom: Odometry):
    writer.writerow([
        datetime.now().isoformat(), odom.last_ts,
        odom.raw_left_enc, odom.raw_right_enc,
        odom.d_left, odom.d_right,
        odom.d_left_m * 1000, odom.d_right_m * 1000,
        odom.d_center * 1000, math.degrees(odom.d_yaw),
        odom.x, odom.y, math.degrees(odom.yaw),
        odom.vx, odom.vth,
    ])


# ============================================================
# 串口工具
# ============================================================

def list_ports():
    """列出所有可用串口"""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
    # 补充板载串口
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if os.path.exists(dev) and dev not in ports:
            ports.append(dev)
    return ports


def auto_detect(baud=115200) -> Optional[str]:
    """自动检测 STM32 串口"""
    for dev in (f"/dev/ttyS{i}" for i in range(8)):
        if not os.path.exists(dev):
            continue
        try:
            ser = serial.Serial(dev, baud, timeout=0.3)
            t0 = time.time()
            while time.time() - t0 < 1.5:
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
# 主函数
# ============================================================

def main():
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    parser = argparse.ArgumentParser(
        description="STM32 底盘里程计测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                        自动检测串口
  %(prog)s -p /dev/ttyS1          指定串口
  %(prog)s -p /dev/ttyUSB0        通过 USB 串口
  %(prog)s --show-raw             显示原始编码器值
  %(prog)s --csv log.csv          记录到 CSV
  %(prog)s --list                 列出串口
        """)
    parser.add_argument('--port', '-p', default=None,
                        help='串口设备路径 (默认自动检测)')
    parser.add_argument('--baud', '-b', type=int, default=115200)
    parser.add_argument('--show-raw', action='store_true',
                        help='显示原始编码器值')
    parser.add_argument('--csv', '-c', default=None,
                        help='CSV 输出文件路径')
    parser.add_argument('--list', '-l', action='store_true',
                        help='列出可用串口')
    args = parser.parse_args()

    if args.list:
        for p in list_ports():
            print(f"  {p}")
        return

    # 确定串口
    port = args.port
    if not port:
        print("自动检测 STM32 串口...", end=' ', flush=True)
        port = auto_detect(args.baud)
        if port:
            print(f"找到 {port}")
        else:
            port = '/dev/ttyS1'
            print(f"未检测到, 默认使用 {port}")

    # CSV 输出
    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_header(csv_writer)

    # 打开串口
    print(f"打开 {port} @ {args.baud} ...", end=' ', flush=True)
    try:
        ser = serial.Serial(port, args.baud, timeout=0.01)
        print("OK")
    except Exception as e:
        print(f"失败: {e}")
        sys.exit(1)

    # 初始化
    odom = Odometry()
    buf = b''
    frame_count = 0
    last_print = time.time()

    draw_header()
    draw_welcome()

    # ── 主循环 ──
    try:
        while True:
            # 读取串口
            try:
                if ser.in_waiting:
                    buf += ser.read(ser.in_waiting)
            except serial.SerialException:
                break

            # 解析帧
            while True:
                result = parse_frame(buf)
                if result is None:
                    if len(buf) > 2048:
                        buf = buf[-1024:]  # 防内存泄漏
                    break

                ftype, seq, payload, flen = result
                buf = buf[flen:]

                if ftype == 0x01:  # ODOM_DATA
                    d = parse_odom(payload)
                    if d:
                        odom.update(d)
                        frame_count += 1

            # 每秒刷新显示 (50帧刷新一次)
            now = time.time()
            if frame_count > 0 and now - last_print >= 0.02:
                draw_odom(odom, args.show_raw)
                if csv_writer:
                    csv_row(csv_writer, odom)
                last_print = now
                frame_count = 0

            # 检查键盘输入 (非阻塞)
            # 用简单的 time.sleep 替代, 不在终端输入处理上纠结
            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n\n  共处理 {frame_count} 帧 ODOM_DATA")
        print(f"  最终位姿: x={odom.x:.3f}  y={odom.y:.3f}  yaw={math.degrees(odom.yaw):.1f}°")
        ser.close()
        if csv_file:
            csv_file.close()
            print(f"  CSV: {args.csv}")


if __name__ == '__main__':
    main()
