#!/usr/bin/env python3
"""
底盘串口监控程序 — 接收 STM32 编码器转速数据并实时显示

STM32 输出格式 (115200 8N1, 每100ms一行):
    FWD L:+1234 R:+1250 | T:1234 1250
    BWD L:-1200 R:-1210 | T:34 40
    LFT L:-1300 R:+1280 | T:-1266 1320
    RGT L:+1280 R:-1300 | T:-1266 1320
    STP L:0 R:0 | T:-1266 1320

用法:
    python3 chassis_serial_monitor.py                          # 自动检测串口
    python3 chassis_serial_monitor.py --port /dev/ttyS1        # 指定串口
    python3 chassis_serial_monitor.py --baud 115200 --csv log.csv  # 记录到CSV
    python3 chassis_serial_monitor.py --list                   # 列出可用串口
"""

import argparse
import csv
import os
import re
import signal
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 需要 pyserial 库, 请执行: pip3 install pyserial")
    sys.exit(1)

# ============================================================
# 配置
# ============================================================

DEFAULT_BAUD = 115200
DEFAULT_PORT = "/dev/ttyS1"       # RDK X5 ↔ STM32 底盘通信串口 (已确认)
LINE_PATTERN = re.compile(
    r"^(FWD|BWD|LFT|RGT|STP)\s+"
    r"L:([+-]?\d+)\s+"
    r"R:([+-]?\d+)\s+\|\s+"
    r"T:([+-]?\d+)\s+"
    r"([+-]?\d+)"
)

# 状态显示映射
STATE_DISPLAY = {
    "FWD": ("前进", "→"),
    "BWD": ("后退", "←"),
    "LFT": ("左转", "↺"),
    "RGT": ("右转", "↻"),
    "STP": ("停止", "■"),
}

# ============================================================
# 串口检测
# ============================================================

def list_serial_ports():
    """列出所有可用串口"""
    print("可用的串口设备:")
    print(f"{'设备路径':<20} {'描述'}")
    print("-" * 60)
    for port in serial.tools.list_ports.comports():
        print(f"{port.device:<20} {port.description}")
    # 同时列出 /dev/ttyS* (Linux 板载串口可能不在 list_ports 中)
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if os.path.exists(dev):
            found = any(p.device == dev for p in serial.tools.list_ports.comports())
            if not found:
                print(f"{dev:<20} (板载串口)")

def auto_detect_port():
    """自动检测 STM32 串口 (尝试打开每个 ttyS*)"""
    for i in range(8):
        dev = f"/dev/ttyS{i}"
        if not os.path.exists(dev):
            continue
        try:
            ser = serial.Serial(dev, DEFAULT_BAUD, timeout=0.5)
            print(f"尝试 {dev} ... ", end="", flush=True)
            time.sleep(0.3)
            # 等待 STM32 输出数据
            t0 = time.time()
            got_data = False
            while time.time() - t0 < 1.5:
                line = ser.readline()
                if b"Encoder" in line or b"L:" in line:
                    got_data = True
                    break
            ser.close()
            if got_data:
                print(f"✓ 检测到 STM32 数据")
                return dev
            else:
                print("无数据")
        except Exception:
            pass
    return None


# ============================================================
# 数据解析
# ============================================================

def parse_line(line: str):
    """解析一行 STM32 编码器数据, 返回 dict 或 None"""
    m = LINE_PATTERN.match(line.strip())
    if not m:
        return None
    return {
        "state": m.group(1),
        "speed_l": int(m.group(2)),
        "speed_r": int(m.group(3)),
        "total_l": int(m.group(4)),
        "total_r": int(m.group(5)),
        "timestamp": datetime.now(),
    }


# ============================================================
# 终端显示
# ============================================================

class ConsoleDisplay:
    """实时终端显示 (ANSI 控制)"""

    def __init__(self, csv_writer=None):
        self.csv = csv_writer
        self.line_count = 0
        self.last_state = None

    def show(self, data: dict):
        state = data["state"]
        state_cn, symbol = STATE_DISPLAY.get(state, (state, "?"))

        # 速度条 (每 100 pulse/s = 1 格, 最大 20 格)
        def bar(val, width=20):
            abs_v = abs(val)
            n = min(abs_v // 100, width)
            if n == 0:
                return "." * width
            if val > 0:
                return "█" * n + "░" * (width - n)
            else:
                return "░" * (width - n) + "█" * n

        bar_l = bar(data["speed_l"])
        bar_r = bar(data["speed_r"])

        # 只在状态变化时打印标题行
        if state != self.last_state:
            print(f"\n{'─'*65}")
            print(f"  {symbol} {state_cn} ({state})")
            print(f"{'─'*65}")
            self.last_state = state

        # 速度行
        now_str = data["timestamp"].strftime("%H:%M:%S")
        print(f"  {now_str}  左 {data['speed_l']:+6d} [{bar_l}] "
              f"右 {data['speed_r']:+6d} [{bar_r}]  "
              f"累计:({data['total_l']:+6d}, {data['total_r']:+6d})")

        self.line_count += 1

        # CSV 记录
        if self.csv:
            self.csv.writerow([
                data["timestamp"].isoformat(),
                data["state"],
                data["speed_l"],
                data["speed_r"],
                data["total_l"],
                data["total_r"],
            ])

    def show_raw(self, line: str):
        """显示未识别的原始行 (调试用)"""
        print(f"  [RAW] {line.strip()}")


# ============================================================
# 主循环
# ============================================================

def signal_handler(sig, frame):
    print("\n\n程序已终止.")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(
        description="STM32 底盘编码器串口监控",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                         自动检测并监控
  %(prog)s --port /dev/ttyS1       指定串口
  %(prog)s --baud 921600           高速模式
  %(prog)s --csv speed_log.csv     记录到CSV文件
  %(prog)s --raw                   显示原始数据(不解析)
  %(prog)s --list                  列出可用串口
        """,
    )
    parser.add_argument("--port", "-p", default=None,
                        help=f"串口设备路径 (默认自动检测, 回退到 {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD,
                        help=f"波特率 (默认 {DEFAULT_BAUD})")
    parser.add_argument("--csv", "-c", default=None,
                        help="CSV 日志文件路径")
    parser.add_argument("--raw", "-r", action="store_true",
                        help="原始模式: 直接打印所有接收数据, 不解析")
    parser.add_argument("--list", "-l", action="store_true",
                        help="列出可用串口并退出")
    parser.add_argument("--timeout", "-t", type=float, default=1.0,
                        help="串口读取超时秒数 (默认 1.0)")

    args = parser.parse_args()

    # --list 模式
    if args.list:
        list_serial_ports()
        return

    # 确定串口
    port = args.port
    if port is None:
        print("正在自动检测 STM32 串口 ...")
        port = auto_detect_port()
        if port is None:
            print(f"自动检测失败, 使用默认串口 {DEFAULT_PORT}")
            port = DEFAULT_PORT
        print()

    # 打开串口
    print(f"打开串口: {port} @ {args.baud} baud")
    try:
        ser = serial.Serial(port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        sys.exit(1)

    # CSV 文件
    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "state", "speed_l", "speed_r", "total_l", "total_r"])
        print(f"CSV 日志: {args.csv}")

    display = ConsoleDisplay(csv_writer)

    print("开始监控 ... 按 Ctrl+C 退出\n")

    buf = ""
    try:
        while True:
            # 读取可用数据
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
                buf += chunk

                # 按行处理
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)

                    if args.raw:
                        # 原始模式: 直接打印
                        print(line)
                        continue

                    data = parse_line(line)
                    if data:
                        display.show(data)
                    elif line.strip():
                        # 非空但无法解析 → 作为信息行打印
                        stripped = line.strip()
                        if any(kw in stripped for kw in
                               ["===", "---", "Encoder"]):
                            print(f"\n{'═'*65}")
                            print(f"  {stripped.strip('-').strip('=').strip()}")
                            print(f"{'═'*65}")
                        else:
                            display.show_raw(line)

            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n共接收 {display.line_count} 行数据")
        ser.close()
        if csv_file:
            csv_file.close()
            print(f"CSV 已保存到: {args.csv}")


if __name__ == "__main__":
    main()
