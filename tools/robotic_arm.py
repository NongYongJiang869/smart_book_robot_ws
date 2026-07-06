#!/usr/bin/env python3
"""
机械臂串口测试工具 — 发 1 取书 #1, 发 2 取书 #2, 收到 0 表示夹取完成

用法:
  python3 tools/robotic_arm.py                     # 默认 /dev/ttyACM0
  python3 tools/robotic_arm.py -p /dev/ttyACM0     # 指定串口
"""

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 需要 pyserial, 请执行: pip3 install pyserial")
    sys.exit(1)

DEFAULT_PORT = "/dev/ttyACM0"
BAUDRATE = 115200


def main():
    parser = argparse.ArgumentParser(description="机械臂取书测试工具")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"串口设备路径 (默认: {DEFAULT_PORT})")
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUDRATE, timeout=0.5)
    print(f"已连接 {args.port} @ {BAUDRATE}")
    print("输入 1 取书#1, 2 取书#2, 3 放书, q 退出\n")

    try:
        while True:
            cmd = input("> ").strip()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd not in ("1", "2", "3"):
                print("请输入 1, 2 或 3")
                continue

            if cmd == "3":
                print("放书 ...")
                ser.write(b"3\n")
                t0 = time.time()
                while time.time() - t0 < 90:
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting).decode(errors="replace").strip()
                        if data:
                            print(f"  ← {data}")
                            if "6" in data:
                                print("✅ 放书完成!")
                                break
                    time.sleep(0.1)
                else:
                    print("⚠ 超时: 未收到放书完成信号")
            else:
                book_id = int(cmd)
                print(f"取书 #{book_id} ...")
                ser.write(f"{book_id}\n".encode())

                # 等待响应, 收到 0 表示夹取完成
                t0 = time.time()
                while time.time() - t0 < 90:
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting).decode(errors="replace").strip()
                        if data:
                            print(f"  ← {data}")
                            if "0" in data:
                                print(f"✅ 书籍 #{book_id} 夹取完成!")
                                break
                    time.sleep(0.1)
                else:
                    print(f"⚠ 超时: 未收到夹取完成信号")

    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
