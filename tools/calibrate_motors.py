#!/usr/bin/env python3
"""
电机补偿曲线标定 — 测多个速度点的左右轮速比

用法:
  1. 关掉 stm32_bridge
  2. 架起小车或放到地面
  3. python3 tools/calibrate_motors.py
"""

import struct, time, sys, numpy as np, serial

PORT, BAUD = "/dev/ttyS1", 115200

def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8): c = (c << 1) ^ 0x1021 if c & 0x8000 else c << 1
        c &= 0xFFFF
    return c

def vel_frame(lx, az, seq=0):
    p = struct.pack('<ff', lx, az)
    ts = struct.pack('<BB', 0x81, seq)
    L = 4 + len(p)
    H = b'\x5A\xA5'
    return H + struct.pack('<B', L) + ts + p + struct.pack('<H', crc16(ts + p))

class Parser:
    def __init__(s): s.b = b''
    def feed(s, d):
        s.b += d; fs = []
        while 1:
            i = s.b.find(b'\x5A\xA5')
            if i < 0: break
            if i: s.b = s.b[i:]
            if len(s.b) < 7: break
            L = s.b[2]; t = 2+1+L
            if len(s.b) < t: break
            f = s.b[:t]; s.b = s.b[t:]
            if crc16(f[3:3+L-2]) == struct.unpack_from('<H', f, t-2)[0]:
                fs.append((f[3], f[4], f[5:3+L-2]))
        return fs

def read_speeds(ser, parser, timeout=1.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ser.in_waiting:
            for tp, sq, pl in parser.feed(ser.read(ser.in_waiting)):
                if tp == 0x01 and len(pl) >= 24:
                    lv, rv = struct.unpack_from('<ff', pl, 8)
                    return abs(lv), abs(rv)
        time.sleep(0.02)
    return None, None

print("=" * 55)
print("电机速比标定 — 测多个速度点")
print("=" * 55)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
parser = Parser()
time.sleep(0.2)
ser.write(vel_frame(0, 0))
time.sleep(0.2)

# 测前进 10 个速度点
speeds = [0.15, 0.18, 0.22, 0.26, 0.30, 0.34, 0.38, 0.42, 0.46, 0.50]
ratios = []

for v in speeds:
    ser.write(vel_frame(v, 0.0))
    time.sleep(1.0)
    lv, rv = read_speeds(ser, parser)
    if lv and rv and lv > 100 and rv > 100:
        ratio = rv / lv  # <1:右慢, >1:右快
        comp = 1.0 / ratio if ratio > 0 else 1.0
        ratios.append((v, ratio, comp))
        print(f"  {v:.2f}m/s  左={lv:5.0f}  右={rv:5.0f}  右/左={ratio:.3f}  补偿={comp:.3f}")
    else:
        print(f"  {v:.2f}m/s  电机未转 (PWM太低)")

ser.write(vel_frame(0, 0))
time.sleep(0.3)
ser.close()

if len(ratios) < 3:
    print("\n数据不足!")
    sys.exit(1)

print(f"\n{'='*55}")
print("结果: 不同速度下右/左转速比")
print(f"{'='*55}")
for v, r, c in ratios:
    bar = "█" * int(abs(1-r) * 200) if r else ""
    dir = "→右快" if r > 1 else "→左快"
    print(f"  {v:.2f}m/s  比={r:.3f}  补偿={c:.3f}  {dir} {bar}")

print(f"\n建议 stm32_params.yaml:")
print(f"  # 速度-补偿分段:")
print(f"  # 低速(<0.25): 不补偿 (死区, 两边都不可靠)")
for i, (v, r, c) in enumerate(ratios):
    if v >= 0.25:
        print(f"  # {v:.2f}m/s: 补偿={c:.3f}")
