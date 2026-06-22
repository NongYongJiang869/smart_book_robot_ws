"""
STM32 串口帧协议编解码器

帧格式 (设计文档 03_communication_protocols.md §1):
  ┌────────┬────────┬────────┬────────┬──────────────┬────────┐
  │ 0x5A   │ 长度   │ 类型   │ 序号   │ 数据负载     │ CRC16  │
  │ 0xA5   │ 1B     │ 1B     │ 1B     │ 0~255B       │ 2B     │
  └────────┴────────┴────────┴────────┴──────────────┴────────┘

下行 (RDK X5 → STM32): VEL_CMD(0x81,100Hz), LED_CTRL(0x82), BUZZER(0x83),
                       RESET_ODOM(0x84), MOTOR_BRAKE(0x85)
上行 (STM32 → RDK X5): ODOM_DATA(0x01,50Hz), STATUS(0x02,10Hz),
                       ERROR(0x04), ACK(0x05)

所有多字节字段为 Little-Endian.
"""

import struct
from typing import Optional, Tuple, Dict


class SerialProtocol:
    """STM32 二进制帧协议"""

    HEADER = b'\x5A\xA5'

    # ── 帧类型码 ──
    # 上行 (STM32 → RDK X5)
    TYPE_ODOM_DATA  = 0x01
    TYPE_STATUS     = 0x02
    TYPE_ERROR      = 0x04
    TYPE_ACK        = 0x05
    TYPE_HEARTBEAT  = 0x06

    # 下行 (RDK X5 → STM32)
    TYPE_VEL_CMD    = 0x81
    TYPE_LED_CTRL   = 0x82
    TYPE_BUZZER     = 0x83
    TYPE_RESET_ODOM = 0x84
    TYPE_MOTOR_BRAKE= 0x85

    # ── 错误码 ──
    ERR_ESTOP          = 0x0001
    ERR_FRONT_COLLISION = 0x0002
    ERR_REAR_COLLISION  = 0x0004
    ERR_COMM_TIMEOUT    = 0x0100
    ERR_IMU_FAULT       = 0x0400

    def __init__(self):
        self._tx_seq = 0  # 发送序号 0~255

    # ============================================================
    # CRC16
    # ============================================================

    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        """CRC-16-CCITT (多项式 0x1021, 初始值 0x0000)"""
        crc = 0x0000
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    # ============================================================
    # 编码 (构建帧)
    # ============================================================

    def _build_frame(self, frame_type: int, payload: bytes) -> bytes:
        """构建完整帧: 帧头 + 长度 + 类型 + 序号 + 负载 + CRC"""
        length = len(payload) + 4  # 类型(1) + 序号(1) + 负载(n) + CRC(2)
        header_and_type = struct.pack('<BB', frame_type, self._tx_seq) + payload

        crc = self.crc16_ccitt(header_and_type)

        self._tx_seq = (self._tx_seq + 1) & 0xFF
        return self.HEADER + struct.pack('<B', length) + header_and_type + struct.pack('<H', crc)

    def encode_vel_cmd(self, linear_x: float, angular_z: float) -> bytes:
        """
        编码速度指令 (TYPE_VEL_CMD, 0x81)

        数据负载 (8B, LE):
          [0:4] float linear_x  目标线速度 m/s, 范围 [-0.5, 0.5]
          [4:8] float angular_z 目标角速度 rad/s, 范围 [-1.0, 1.0]
        """
        payload = struct.pack('<ff', linear_x, angular_z)
        return self._build_frame(self.TYPE_VEL_CMD, payload)

    def encode_led_ctrl(self, led_state: int) -> bytes:
        """编码灯光控制 (0x82), 负载 2B LE uint16"""
        return self._build_frame(self.TYPE_LED_CTRL, struct.pack('<H', led_state))

    def encode_buzzer(self, duration_ms: int) -> bytes:
        """编码蜂鸣器 (0x83), 负载 1B"""
        return self._build_frame(self.TYPE_BUZZER, struct.pack('<B', min(duration_ms, 255)))

    def encode_reset_odom(self) -> bytes:
        """编码重置里程计 (0x84), 无负载"""
        return self._build_frame(self.TYPE_RESET_ODOM, b'')

    def encode_motor_brake(self) -> bytes:
        """编码紧急刹车 (0x85), 无负载"""
        return self._build_frame(self.TYPE_MOTOR_BRAKE, b'')

    # ============================================================
    # 解码 (解析帧)
    # ============================================================

    def decode_frame(self, data: bytes) -> Optional[Tuple[int, int, bytes]]:
        """
        解析原始字节, 返回 (frame_type, seq, payload) 或 None

        内部维护状态机, 可处理不完整的流式数据。
        作为简易版本, 这里要求调用方传入的 data 已经过帧同步处理。
        """
        if len(data) < 6:  # 最小帧: 头(2) + 长度(1) + 类型(1) + 序号(1) + CRC(2) = 7B
            return None

        # 查找帧头
        idx = data.find(self.HEADER)
        if idx < 0:
            return None
        if idx > 0:
            data = data[idx:]  # 跳过帧头前的垃圾数据

        if len(data) < 7:
            return None

        length = data[2]
        total_len = 2 + 1 + length  # 头(2) + 长度字节(1) + 实际内容

        if len(data) < total_len:
            return None  # 帧不完整

        frame = data[2:2 + 1 + length]  # 长度字节 + 内容
        payload_start = 3  # 跳过长度(1) + 类型(1) + 序号(1)
        crc_pos = payload_start + (length - 4)  # 减去 类型+序号+CRC

        frame_type = frame[1]
        seq = frame[2]
        check_data = frame[1:crc_pos]  # 类型 → 负载末尾
        crc_received = struct.unpack_from('<H', frame, crc_pos)[0]
        crc_calculated = self.crc16_ccitt(check_data)

        if crc_received != crc_calculated:
            return None  # CRC 校验失败

        payload = frame[payload_start:crc_pos]
        return (frame_type, seq, payload)

    # ============================================================
    # 上行帧解析
    # ============================================================

    def decode_odom_data(self, payload: bytes) -> Optional[Dict]:
        """
        解析 ODOM_DATA (0x01) 负载
        格式 (24B LE):
          [0:4]  int32  left_enc       左轮编码器累计值
          [4:8]  int32  right_enc      右轮编码器累计值
          [8:12] float  left_wheel_v   左轮速度 m/s
          [12:16]float  right_wheel_v  右轮速度 m/s
          [16:18]int16  gyro_z         陀螺Z (度/秒 ×1000)
          [18:20]int16  accel_x        加速度X (m/s² ×1000)
          [20:22]int16  accel_y        加速度Y (m/s² ×1000)
          [22:24]uint16 timestamp_ms   毫秒时间戳
        """
        if len(payload) < 24:
            return None
        left_enc, right_enc, lv, rv, gyro, acc_x, acc_y, ts = \
            struct.unpack('<iiffhhhH', payload)
        return {
            'left_enc': left_enc,
            'right_enc': right_enc,
            'left_wheel_v': lv,
            'right_wheel_v': rv,
            'gyro_z_dps': gyro / 131.0,       # LSB→°/s  (±250°/s量程)
            'accel_x': acc_x / 16384.0 * 9.807,  # LSB→m/s² (±2g量程)
            'accel_y': acc_y / 16384.0 * 9.807,
            'timestamp_ms': ts,
        }

    def decode_status(self, payload: bytes) -> Optional[Dict]:
        """
        解析 STATUS (0x02) 负载
        格式 (6B):
          [0] uint8  motor_state   位0~3: 4路电机使能
          [1] uint8  sensor_state  位0:急停 位1:前碰 位2:后碰
          [2:4] int16 mcu_temp     温度×100 (°C)
          [4:6] uint16 error_code
        """
        if len(payload) < 6:
            return None
        motor_state, sensor_state, mcu_temp, error_code = \
            struct.unpack('<BBhH', payload)
        return {
            'motor_state': motor_state,
            'emergency_stop': bool(sensor_state & 0x01),
            'collision_front': bool(sensor_state & 0x02),
            'collision_rear': bool(sensor_state & 0x04),
            'mcu_temp': mcu_temp / 100.0,
            'error_code': error_code,
        }

    def decode_error(self, payload: bytes) -> int:
        """解析 ERROR (0x04), 返回 error_code"""
        if len(payload) < 2:
            return 0
        return struct.unpack('<H', payload)[0]
