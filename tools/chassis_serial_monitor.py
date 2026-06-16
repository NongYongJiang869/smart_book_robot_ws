#!/usr/bin/env python3
"""
底盘串口监控程序 — 解析 STM32 二进制帧协议, 实时显示编码器/IMU/状态

自动适配:
  - 二进制模式: 解析 0x5A 0xA5 帧协议 (当前固件)
  - 文本模式:   解析旧版 printf 输出 (兼容旧固件)

用法:
  python3 tools/chassis_serial_monitor.py                 # 自动检测
  python3 tools/chassis_serial_monitor.py -p /dev/ttyS1   # 指定串口
  python3 tools/chassis_serial_monitor.py --csv log.csv   # CSV 记录
  python3 tools/chassis_serial_monitor.py --list          # 列出串口
"""

import argparse
import csv
import os
import re
import signal
import struct
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 需要 pyserial, 请执行: pip3 install pyserial")
    sys.exit(1)

# ============================================================
# 二进制协议 (与 chassis/src/protocol.c 和 RDKX5 serial_protocol.py 一致)
# ============================================================

HEADER = b'\x5A\xA5'

FRAME_NAMES = {
    0x01: "ODOM", 0x02: "STATUS", 0x04: "ERROR",
    0x05: "ACK",  0x06: "HEARTBEAT",
    0x81: "VEL_CMD", 0x82: "LED", 0x83: "BUZZER",
    0x84: "RST_ODOM", 0x85: "BRAKE",
}

ERROR_NAMES = {
    0x0001: "急停", 0x0002: "前碰撞", 0x0004: "后碰撞",
    0x0100: "通信超时", 0x0400: "IMU故障",
}

# ANSI 颜色
C_RESET = "\033[0m"; C_BOLD = "\033[1m"; C_DIM = "\033[2m"
C_RED = "\033[31m"; C_GREEN = "\033[32m"; C_YELLOW = "\033[33m"
C_BLUE = "\033[34m"; C_CYAN = "\033[36m"; C_WHITE = "\033[37m"

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
# 二进制帧解析
# ============================================================

class FrameParser:
    def __init__(self):
        self.buf = b''

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0: break
            if idx > 0:
                self.buf = self.buf[idx:]  # 丢弃帧头前垃圾
            if len(self.buf) < 7: break
            L = self.buf[2]
            total = 2 + 1 + L
            if len(self.buf) < total: break
            frame = self.buf[:total]
            self.buf = self.buf[total:]
            # 校验 CRC
            ftype = frame[3]; plen = L - 4
            if plen < 0: continue
            crc_r = struct.unpack_from('<H', frame, 5 + plen)[0]
            crc_c = crc16_ccitt(frame[3:5 + plen])
            if crc_r != crc_c: continue
            payload = frame[5:5 + plen]
            frames.append((ftype, frame[4], payload))
        return frames

def parse_odom(payload: bytes):
    if len(payload) < 24: return None
    le, re = struct.unpack_from('<ii', payload, 0)
    lv, rv = struct.unpack_from('<ff', payload, 8)
    gyro_z, acc_x, acc_y = struct.unpack_from('<hhh', payload, 16)
    ts = struct.unpack_from('<H', payload, 22)[0]
    return {
        'left_enc': le, 'right_enc': re,
        'left_v': lv, 'right_v': rv,
        'gyro_z': gyro_z, 'accel_x': acc_x, 'accel_y': acc_y,
        'ts': ts,
    }

def parse_status(payload: bytes):
    if len(payload) < 6: return None
    motor, sensor, temp, err = struct.unpack_from('<BBhH', payload, 0)
    return {
        'motor_state': motor,
        'emergency': bool(sensor & 0x01),
        'front_coll': bool(sensor & 0x02),
        'rear_coll': bool(sensor & 0x04),
        'mcu_temp': temp / 100.0,
        'error_code': err,
    }

def parse_vel_cmd(payload: bytes):
    if len(payload) < 8: return None
    lx, az = struct.unpack_from('<ff', payload, 0)
    return {'linear_x': lx, 'angular_z': az}

# ============================================================
# 文本协议解析 (旧版兼容)
# ============================================================

LINE_RE = re.compile(
    r"^(FWD|BWD|LFT|RGT|STP)\s+L:([+-]?\d+)\s+R:([+-]?\d+)\s+\|\s+T:([+-]?\d+)\s+([+-]?\d+)")

def parse_text(line: str):
    m = LINE_RE.match(line.strip())
    if not m: return None
    return {
        "state": m.group(1), "speed_l": int(m.group(2)),
        "speed_r": int(m.group(3)), "total_l": int(m.group(4)),
        "total_r": int(m.group(5)),
    }

# ============================================================
# 显示
# ============================================================

class Monitor:
    def __init__(self, csv_writer=None):
        self.csv = csv_writer
        self.counts = {}        # 帧类型计数
        self.last_odom = None   # 上次 ODOM_DATA
        self.last_status = None # 上次 STATUS
        self.start_time = time.time()
        self.line = 0           # 显示行数

    def _hdr(self, text):
        print(f"\n{C_BOLD}{C_CYAN}── {text} {C_RESET}{'─'*55}")

    def _val(self, label, val, unit="", color=""):
        return f"{C_DIM}{label}:{C_RESET} {color}{val}{unit}{C_RESET}"

    def _bar(self, val, width=15, max_v=1000):
        """速度条"""
        n = min(abs(val) // (max_v // width), width)
        if n == 0: return C_DIM + "·" * width + C_RESET
        if val > 0:
            return f"{C_GREEN}{'█' * n}{C_DIM}{'░' * (width - n)}{C_RESET}"
        else:
            return f"{C_DIM}{'░' * (width - n)}{C_RED}{'█' * n}{C_RESET}"

    def show_frame(self, ftype: int, payload: bytes):
        name = FRAME_NAMES.get(ftype, f"0x{ftype:02X}")
        self.counts[name] = self.counts.get(name, 0) + 1

        if ftype == 0x01:       # ODOM_DATA
            self._show_odom(payload)
        elif ftype == 0x02:     # STATUS
            self._show_status(payload)
        elif ftype == 0x04:     # ERROR
            self._show_error(payload)
        elif ftype == 0x06:     # HEARTBEAT
            pass  # 静默
        elif ftype == 0x81:     # VEL_CMD
            self._show_vel_cmd(payload)

    def _show_odom(self, payload):
        d = parse_odom(payload)
        if not d: return
        self.last_odom = d
        # 只在新的一秒或状态变化时打印
        if self.line % 10 == 0:
            self._hdr("编码器 & IMU")
            print(f"  {'时间':>8s} {'左编码':>8s} {'右编码':>8s} {'左速度':>8s} {'右速度':>8s} | "
                  f"{'陀螺Z':>7s} {'加X':>7s} {'加Y':>7s}")

        gyro_color = C_YELLOW if abs(d['gyro_z']) > 500 else ""   # >0.5°/s 高亮
        acc_color  = C_YELLOW if abs(d['accel_x']) > 500 or abs(d['accel_y']) > 500 else ""

        lv_bar = self._bar(int(d['left_v']))
        rv_bar = self._bar(int(d['right_v']))
        print(f"  {d['ts']:>5d}ms {d['left_enc']:>+8d} {d['right_enc']:>+8d} "
              f"{d['left_v']:>+8.0f} {d['right_v']:>+8.0f} | "
              f"{gyro_color}{d['gyro_z']:>+7d}{C_RESET} "
              f"{acc_color}{d['accel_x']:>+7d} {d['accel_y']:>+7d}{C_RESET}")

        if self.csv:
            self.csv.writerow([datetime.now().isoformat(), "ODOM",
                d['left_enc'], d['right_enc'], d['left_v'], d['right_v'],
                d['gyro_z'], d['accel_x'], d['accel_y']])

    def _show_status(self, payload):
        d = parse_status(payload)
        if not d: return
        changed = (self.last_status is None or
                   d['error_code'] != self.last_status.get('error_code') or
                   d['motor_state'] != self.last_status.get('motor_state'))
        if not changed:
            return
        self.last_status = d

        # 错误码解析
        errs = []
        code = d['error_code']
        for mask, name in sorted(ERROR_NAMES.items(), reverse=True):
            if code & mask:
                errs.append(name)
                code &= ~mask
        err_str = ",".join(errs) if errs else f"{C_GREEN}正常{C_RESET}"

        # 传感器
        sensor_parts = []
        if d['emergency']: sensor_parts.append(f"{C_RED}急停{C_RESET}")
        if d['front_coll']: sensor_parts.append(f"{C_RED}前碰{C_RESET}")
        if d['rear_coll']: sensor_parts.append(f"{C_RED}后碰{C_RESET}")
        sensor_str = ",".join(sensor_parts) if sensor_parts else f"{C_GREEN}无{C_RESET}"

        motor = "使能" if d['motor_state'] else "停止"

        self._hdr("底盘状态")
        print(f"  电机: {motor} | 传感器: {sensor_str} | "
              f"MCU温度: {d['mcu_temp']:.1f}°C | 错误: {err_str}")

    def _show_error(self, payload):
        if len(payload) < 2: return
        code = struct.unpack_from('<H', payload, 0)[0]
        name = ERROR_NAMES.get(code, f"0x{code:04X}")
        print(f"\n{C_RED}{C_BOLD}  ⚠ 底盘错误: {name} (0x{code:04X}){C_RESET}")

    def _show_vel_cmd(self, payload):
        d = parse_vel_cmd(payload)
        if not d: return
        print(f"  {C_BLUE}RDK→STM:{C_RESET} V={d['linear_x']:.2f}m/s ω={d['angular_z']:.2f}rad/s")

    def show_rate(self):
        """每秒打印帧率统计"""
        elapsed = time.time() - self.start_time
        if elapsed < 1.0: return
        parts = []
        for name in ["ODOM", "STATUS", "HEARTBEAT", "VEL_CMD"]:
            n = self.counts.get(name, 0)
            if n > 0:
                parts.append(f"{name}:{n/elapsed:.0f}Hz")
        if parts:
            print(f"\n{C_DIM}  {' | '.join(parts)}  (总帧数: {sum(self.counts.values())}){C_RESET}")
        self.counts.clear()
        self.start_time = time.time()

    def show_text(self, data: dict):
        """显示旧版文本协议数据"""
        self.counts["TEXT"] = self.counts.get("TEXT", 0) + 1
        if self.line % 15 == 0:
            self._hdr("编码器 (文本模式)")
        bar_l = self._bar(data['speed_l'], 15, 1500)
        bar_r = self._bar(data['speed_r'], 15, 1500)
        print(f"  {data['state']:4s}  左 {data['speed_l']:+6d} [{bar_l}]  "
              f"右 {data['speed_r']:+6d} [{bar_r}]  "
              f"累计:({data['total_l']:+6d},{data['total_r']:+6d})")
        self.line += 1


# ============================================================
# 串口工具
# ============================================================

def list_serial_ports():
    print("可用的串口设备:")
    print(f"  {'设备路径':<20} {'描述'}")
    print(f"  {'─'*20} {'─'*35}")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device:<20} {port.description}")
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if os.path.exists(dev) and not any(p.device == dev for p in serial.tools.list_ports.comports()):
            print(f"  {dev:<20} (板载串口)")

def auto_detect(baud=115200):
    """自动检测 STM32 (查找 0x5A 0xA5 帧头)"""
    for dev in (f"/dev/ttyS{i}" for i in range(8)):
        if not os.path.exists(dev): continue
        try:
            ser = serial.Serial(dev, baud, timeout=0.3)
            print(f"  尝试 {dev} ... ", end="", flush=True)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                n = ser.in_waiting
                if n > 0:
                    data = ser.read(n)
                    if HEADER in data:
                        print(f"{C_GREEN}✓ 检测到 STM32 数据{C_RESET}")
                        ser.close()
                        return dev
                time.sleep(0.05)
            ser.close()
            print("无数据")
        except Exception:
            pass
    return None


# ============================================================
# 主循环
# ============================================================

def main():
    sys.stdout.reconfigure(line_buffering=True)  # 解决管道缓冲问题
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    parser = argparse.ArgumentParser(
        description="STM32 底盘串口监控 (二进制帧 + 文本兼容)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                           自动检测
  %(prog)s -p /dev/ttyS1             指定串口
  %(prog)s --csv debug.csv           保存 CSV
  %(prog)s --bin-hex                 16进制显示帧数据
  %(prog)s --list                    列出可用串口
        """)
    parser.add_argument("--port", "-p", default=None)
    parser.add_argument("--baud", "-b", type=int, default=115200)
    parser.add_argument("--csv", "-c", default=None)
    parser.add_argument("--bin-hex", action="store_true",
                        help="每帧打印 16 进制原始数据")
    parser.add_argument("--no-color", "-n", action="store_true",
                        help="禁用 ANSI 颜色")
    parser.add_argument("--all", "-a", action="store_true",
                        help="打印每一帧 (默认 ODOM 每 250ms 打印一行)")
    parser.add_argument("--list", "-l", action="store_true")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    port = args.port or auto_detect(args.baud) or "/dev/ttyS1"
    if not args.port:
        print()

    print(f"打开 {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.5)

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["time", "type", "left_enc", "right_enc",
                             "left_v", "right_v", "gyro_z", "accel_x", "accel_y"])

    # 颜色控制
    global C_RESET, C_BOLD, C_DIM, C_RED, C_GREEN, C_YELLOW, C_BLUE, C_CYAN, C_WHITE
    if args.no_color:
        C_RESET = C_BOLD = C_DIM = C_RED = C_GREEN = C_YELLOW = C_BLUE = C_CYAN = C_WHITE = ""

    monitor = Monitor(csv_writer)
    parser_ = FrameParser()
    text_buf = ""
    mode = None
    last_rate = time.time()
    last_odom_print = 0  # ODOM 打印节流

    print("监控中 ... Ctrl+C 退出\n")

    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)

                # 自动检测模式
                if mode is None:
                    if HEADER in data:
                        mode = 'binary'
                        monitor._hdr("检测到二进制协议, 开始解析")
                    elif b'\n' in data or b'\r' in data:
                        mode = 'text'
                        monitor._hdr("检测到文本协议, 开始解析")
                    else:
                        continue  # 等更多数据

                if mode == 'binary':
                    frames = parser_.feed(data)
                    for ftype, seq, payload in frames:
                        # ODOM_DATA 节流 (默认 250ms 打印一次, --all 打印每帧)
                        if ftype == 0x01 and not args.all:
                            now = time.time()
                            if now - last_odom_print < 0.25:
                                monitor.counts["ODOM"] = monitor.counts.get("ODOM", 0) + 1
                                monitor.line += 1
                                continue
                            last_odom_print = now
                        monitor.show_frame(ftype, payload)
                        if args.bin_hex:
                            print(f"  {C_DIM}[{FRAME_NAMES.get(ftype, f'0x{ftype:02X}')} seq={seq}] "
                                  f"{payload.hex()}{C_RESET}")
                        monitor.line += 1

                else:  # text mode
                    text = data.decode("utf-8", errors="replace")
                    text_buf += text
                    while "\n" in text_buf:
                        line, text_buf = text_buf.split("\n", 1)
                        d = parse_text(line)
                        if d:
                            monitor.show_text(d)
                        elif line.strip():
                            kw = ["===", "---", "Encoder", "STM32"]
                            if any(k in line for k in kw):
                                print(f"  {C_CYAN}{line.strip('-').strip('=').strip()}{C_RESET}")

            else:
                time.sleep(0.01)

            # 每秒打印帧率
            if time.time() - last_rate > 2.0:
                monitor.show_rate()
                last_rate = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        total = sum(monitor.counts.values())
        print(f"\n共 {total} 帧. 串口已关闭.")
        ser.close()
        if csv_file:
            csv_file.close()
            print(f"CSV: {args.csv}")


if __name__ == "__main__":
    main()
