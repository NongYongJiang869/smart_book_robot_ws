#!/usr/bin/env python3
"""
OpenMV 串口通信工具 — 向 OpenMV 发送指令并接收响应

OpenMV 作为中间桥接:
  RDK X5 ──USB(/dev/ttyACM1)──→ OpenMV ──UART3──→ 机械臂 Arduino
  RDK X5 ←──USB(/dev/ttyACM1)── OpenMV ←──UART3── 机械臂 Arduino

支持的指令类型:
  ┌──────────────────────────────────┬──────────────────────┐
  │ OpenMV 本地指令                  │ 说明                 │
  ├──────────────────────────────────┼──────────────────────┤
  │  1                               │ 触发取书 #1 (AprilTag 0) │
  │  2                               │ 触发取书 #2 (AprilTag 1) │
  │  3                               │ 触发放书动作         │
  │  PX:<kp>,<ki>,<kd>              │ 设置 X 轴 PID         │
  │  PZ:<kp>,<ki>,<kd>              │ 设置 Z 轴 PID         │
  │  DBG                             │ 查询 PID 状态         │
  ├──────────────────────────────────┼──────────────────────┤
  │ 机械臂透传指令 (经 OpenMV → 机械臂) │                      │
  ├──────────────────────────────────┼──────────────────────┤
  │  $KMS:x,y,z,time!               │ IK 笛卡尔移动         │
  │  #xxxPyyyyTzzzz!                │ 单舵机 PWM 控制       │
  │  #255PyyyyTzzzz!                │ 全舵机 PWM 控制       │
  │  $DST!  /  $DST:N!              │ 停止 / 停止指定舵机    │
  │  $RST!                          │ 软复位                │
  │  $QSTAT!                        │ 查询状态              │
  │  $QPWM!                         │ 查询所有舵机 PWM       │
  │  $DGT:start-end,times!          │ 执行动作组            │
  │  {Gxxxx#...!...}                │ 动作组直接执行         │
  │  <Gxxxx#...!...>                │ 动作组下载            │
  └──────────────────────────────────┴──────────────────────┘

用法:
  python3 tools/openmv_serial.py                    交互模式 (默认)
  python3 tools/openmv_serial.py -s "1"             发送单条指令
  python3 tools/openmv_serial.py -s "DBG"           查询 PID
  python3 tools/openmv_serial.py -s '$KMS:100,200,50,1000!'  发送机械臂指令
  python3 tools/openmv_serial.py -s '#000P1500T1000!'        发送舵机指令
  python3 tools/openmv_serial.py -c 1               发送取书 #1 (简写)
  python3 tools/openmv_serial.py --monitor          只监听串口输出
"""

import argparse
import sys
import time
import threading

try:
    import serial
except ImportError:
    print("错误: 需要 pyserial, 请执行: pip3 install pyserial")
    sys.exit(1)

# ============================================================
# 固定串口配置 — OpenMV 通过 USB 虚拟串口连接
# 使用 udev 规则固定设备名（推荐）:
#   SUBSYSTEM=="tty", ATTRS{idVendor}=="37c5", ATTRS{idProduct}=="5605",
#   SYMLINK+="openmv"
# 然后改为: PORT = "/dev/openmv"
# ============================================================
PORT = "/dev/ttyACM1"
BAUDRATE = 115200
TIMEOUT = 0.1          # 读取超时 (秒)
RESPONSE_WAIT = 0.5    # 发送指令后等待响应的时间 (秒)
EXIT_KEYWORDS = ("q", "quit", "exit", "\\q")


def open_port():
    """打开串口"""
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        return ser
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {PORT}: {e}")
        sys.exit(1)


def read_all(ser) -> str:
    """读取串口缓冲区中的所有数据"""
    data = b""
    while ser.in_waiting:
        chunk = ser.read(ser.in_waiting)
        if chunk:
            data += chunk
        time.sleep(0.01)
    return data.decode(errors="replace").strip()


def send_command(ser, cmd: str) -> str:
    """发送指令并等待响应"""
    # 发送
    payload = (cmd + "\n").encode() if not cmd.endswith("\n") else cmd.encode()
    ser.write(payload)
    ser.flush()

    # 等待响应
    time.sleep(RESPONSE_WAIT)

    # 读取
    resp = read_all(ser)
    return resp


def send_command_verbose(ser, cmd: str):
    """发送指令并打印响应"""
    print(f"→ {cmd}")
    resp = send_command(ser, cmd)
    if resp:
        for line in resp.split("\n"):
            print(f"← {line}")
    else:
        print("← (无响应)")
    print()


def monitor_mode(ser):
    """只监听串口输出，不发送任何数据"""
    print(f"监听 {PORT} ... (按 Ctrl+C 退出)")
    print("━" * 50)
    try:
        while True:
            if ser.in_waiting:
                data = read_all(ser)
                if data:
                    for line in data.split("\n"):
                        print(f"← {line}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def interactive_mode(ser):
    """交互模式：逐条发送指令并查看响应"""
    print("╔══════════════════════════════════════════════════════╗")
    print("║         OpenMV 串口通信工具 — 交互模式               ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  OpenMV 本地:  1 | 2 | 3 | PX:k,i,d | PZ:k,i,d | DBG║")
    print("║  机械臂透传:  $KMS:x,y,z,t! | #xxxPyyyyTzzzz!       ║")
    print("║               $DST! | $RST! | $QSTAT! | $DGT:...    ║")
    print("║  快捷命令:     /monitor 只监听 | /help | q 退出       ║")
    print("╚══════════════════════════════════════════════════════╝")
    print(f"串口: {PORT} @ {BAUDRATE}")
    print()

    try:
        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not cmd:
                continue

            if cmd.lower() in EXIT_KEYWORDS:
                break

            # 快捷命令
            if cmd == "/help":
                print("OpenMV 本地: 1 | 2 | 3 | PX:k,i,d | PZ:k,i,d | DBG")
                print("机械臂: $KMS:... | #xxxP... | $DST! | $RST! | $QSTAT!")
                print("动作组: {Gxxxx#...!} | <Gxxxx#...!> | $DGT:s-e,times!")
                print("快捷:  /monitor | /help | q")
                print()
                continue

            if cmd == "/monitor":
                print("进入监听模式 (按 Ctrl+C 返回交互模式)...")
                try:
                    while True:
                        if ser.in_waiting:
                            data = read_all(ser)
                            if data:
                                print(f"← {data}")
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    print("\n返回交互模式")
                    print()
                continue

            send_command_verbose(ser, cmd)

    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        print("串口已关闭")


def main():
    parser = argparse.ArgumentParser(
        description="OpenMV 串口通信工具 — 向 OpenMV 发送指令并接收响应",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                               交互模式
  %(prog)s -s "1"                        发送取书 #1 指令
  %(prog)s -s "DBG"                      查询 PID 参数
  %(prog)s -s '$KMS:100,200,50,1000!'   发送机械臂 IK 指令
  %(prog)s -c 2                          取书 #2 (简写)
  %(prog)s --monitor                     只监听串口
        """,
    )
    parser.add_argument(
        "--send", "-s",
        default="",
        help="发送单条指令后退出",
    )
    parser.add_argument(
        "--cmd", "-c",
        type=int, choices=[1, 2, 3],
        help="快捷取书/放书 (1/2=取书, 3=放书)",
    )
    parser.add_argument(
        "--monitor", "-m",
        action="store_true",
        help="只监听串口输出",
    )
    args = parser.parse_args()

    # ---- 一次性指令模式 ----
    if args.cmd:
        ser = open_port()
        try:
            send_command_verbose(ser, str(args.cmd))
        finally:
            ser.close()
        return

    if args.send:
        ser = open_port()
        try:
            send_command_verbose(ser, args.send)
        finally:
            ser.close()
        return

    # ---- 监听模式 ----
    if args.monitor:
        ser = open_port()
        try:
            monitor_mode(ser)
        finally:
            ser.close()
        return

    # ---- 交互模式 (默认) ----
    ser = open_port()
    interactive_mode(ser)


if __name__ == "__main__":
    main()
