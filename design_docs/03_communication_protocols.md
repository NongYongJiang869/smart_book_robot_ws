# 智能寻书机器人 — 通信协议设计

## 1. STM32 ↔ RDK X5 串口帧协议

### 1.1 帧格式（统一格式，双向共用）

```
┌─────────┬─────────┬─────────┬─────────┬──────────────┬──────────┐
│ 帧头    │ 长度    │ 帧类型  │ 帧序号  │ 数据负载     │ CRC16    │
│ 2 Bytes │ 1 Byte  │ 1 Byte  │ 1 Byte  │ 0~255 Bytes  │ 2 Bytes  │
├─────────┼─────────┼─────────┼─────────┼──────────────┼──────────┤
│ 0x5A    │ N+7     │ 见下表  │ 0~255   │              │ CRC-16   │
│ 0xA5    │ (含CRC) │         │ 循环    │              │ CCITT    │
└─────────┴─────────┴─────────┴─────────┴──────────────┴──────────┘
```

- **帧头**：固定 `0x5A 0xA5`，用于帧同步
- **长度**：从 帧类型 到 CRC 的总字节数 = 数据负载长度 + 4
- **帧类型**：标识该帧的用途，见类型表
- **帧序号**：每个方向的帧独立递增（0~255 循环），用于检测丢帧
- **数据负载**：具体内容由帧类型决定
- **CRC16**：从 帧类型 到 数据负载末尾 的 CRC-16-CCITT 校验值（多项式 0x1021）

### 1.2 帧类型定义

#### STM32 → RDK X5（上行帧）

| 类型码 | 名称 | 发送频率 | 负载长度 | 说明 |
|--------|------|----------|----------|------|
| 0x01 | ODOM_DATA | 50Hz | 24 Bytes | 里程计+IMU数据 |
| 0x02 | STATUS | 10Hz | 6 Bytes | 底盘状态 |
| 0x04 | ERROR | 事件触发 | 2 Bytes | 错误/告警 |
| 0x05 | ACK | 事件触发 | 2 Bytes | 响应确认 |
| 0x06 | HEARTBEAT | 1Hz | 0 Bytes | 心跳（仅帧头+长度+类型+序号+CRC）|

#### RDK X5 → STM32（下行帧）

| 类型码 | 名称 | 发送频率 | 负载长度 | 说明 |
|--------|------|----------|----------|------|
| 0x81 | VEL_CMD | 100Hz | 8 Bytes | 速度指令 |
| 0x82 | LED_CTRL | 事件触发 | 2 Bytes | 灯光控制 |
| 0x83 | BUZZER | 事件触发 | 1 Byte | 蜂鸣器 |
| 0x84 | RESET_ODOM | 事件触发 | 0 Bytes | 重置里程计 |
| 0x85 | MOTOR_BRAKE | 事件触发 | 0 Bytes | 紧急刹车 |
| 0x86 | HEARTBEAT | 1Hz | 0 Bytes | 心跳 |

### 1.3 数据负载详细格式

#### 0x01 — ODOM_DATA（上行，50Hz，24 Bytes）

```
偏移  长度   类型      字段          说明
──────────────────────────────────────────────
0     4      int32    left_enc       左轮编码器累计值（圈数×CPR）
4     4      int32    right_enc      右轮编码器累计值（圈数×CPR）
8     4      float    left_wheel_v   左轮速度(m/s)
12    4      float    right_wheel_v  右轮速度(m/s)
16    2      int16    gyro_z         陀螺仪Z轴角速度×1000 (°/s)
18    2      int16    accel_x        加速度X轴×1000 (m/s²)
20    2      int16    accel_y        加速度Y轴×1000 (m/s²)
22    2      uint16   timestamp_ms   毫秒时间戳（0~65535循环）
──────────────────────────────────────────────
总长度: 24 Bytes
```

**注意**：多字节数值均采用**小端字节序**（Little-Endian）。

#### 0x02 — STATUS（上行，10Hz，8 Bytes）

```
偏移  长度   类型      字段          说明
──────────────────────────────────────────────
0     1      uint8    motor_state    位0~3: 4个电机使能状态
1     1      uint8    sensor_state   位0:急停 位1:前碰 位2:后碰
2     2      int16    mcu_temp       MCU温度×100 (°C)
4     2      uint16   error_code     错误码
──────────────────────────────────────────────
总长度: 6 Bytes
```

#### 0x81 — VEL_CMD（下行，100Hz，8 Bytes）

```
偏移  长度   类型      字段          说明
──────────────────────────────────────────────
0     4      float    linear_x       目标线速度(m/s)，范围[-0.5, 0.5]
4     4      float    angular_z      目标角速度(rad/s)，范围[-1.0, 1.0]
──────────────────────────────────────────────
总长度: 8 Bytes
```

#### 0x04 — ERROR（上行，事件触发，2 Bytes）

```
偏移  长度   类型      字段          说明
──────────────────────────────────────────────
0     2      uint16   error_code     错误码
                                     0x0001: 急停触发
                                     0x0002: 前碰撞
                                     0x0004: 后碰撞
                                     0x0100: 通信超时
                                     0x0400: IMU故障
```

### 1.4 CRC16 计算

```python
# CRC-16-CCITT (多项式 0x1021, 初始值 0x0000)
def crc16_ccitt(data: bytes) -> int:
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
```

### 1.5 帧同步与错误处理

**发送端**：
1. 构建帧（从帧类型到数据负载），计算 CRC，添加帧头
2. 若帧内出现 `0x5A` 或 `0xA5`，无需转义（帧长固定，不会误判）

**接收端（状态机）**：
```
状态 ──收到 0x5A──→ 状态1 ──收到 0xA5──→ 状态2（读长度）
  ↑                    │                    │
  └────────────────────┴────────────────────┘（字节≠0xA5则回退）
                                              │
                                         读长度→读类型→读序号→读数据→读CRC
                                              │
                                         验证CRC→处理帧→回到状态0
```

**超时规则**：
- ODOM_DATA 超过 100ms 未收到：标记里程计异常
- VEL_CMD 超过 200ms 未发送新值：STM32 自动减速到 0
- HEARTBEAT 超过 1s 未收到：记录通信告警

---

## 2. RDK X5 ↔ OpenMV 通信协议

### 2.1 协议概述

- 物理层：USB 虚拟串口 (CDC)
- 波特率：115200
- 格式：**文本行协议**，每条消息以 `\n` 结尾
- 编码：UTF-8

### 2.2 消息格式

#### RDK X5 → OpenMV（命令）

```
CMD:<命令名>;<参数1>=<值1>;<参数2>=<值2>\n
```

示例：
```
CMD:search_qr;content=book_042\n       # 搜索特定内容的QR码
CMD:continuous_detect;enable=true;interval=100\n
CMD:set_exposure;value=5000\n
CMD:led;state=on\n
```

#### OpenMV → RDK X5（检测结果）

```
DETECT:<QR内容>;x=<像素X>;y=<像素Y>;w=<宽度>;h=<高度>;confidence=<置信度>\n
```

示例：
```
DETECT:book_042;x=160;y=120;w=45;h=45;confidence=0.95\n
DETECT:none\n
```

> **备注**：OpenMV H7 Plus 的 QR 码检测无法直接返回物理距离与角度（不像 AprilTag 有 PnP 解算）。距离估算通过 `w`（QR码在图像中的像素宽度）与已知物理尺寸的比例关系来计算，在 openmv_bridge 节点中完成。

### 2.3 OpenMV 端工作模式

| 模式 | 说明 |
|------|------|
| **连续检测模式** | OpenMV 持续检测视野内所有 QR 码，每帧上报一次 |
| **搜索模式** | 指定目标 QR 内容（如 `book_042`），只上报匹配的码 |
| **待机模式** | 降低帧率，仅维持心跳 |

### 2.4 QR 码规格

| 参数 | 值 |
|------|-----|
| 编码内容 | `book_<书籍ID>`，如 `book_042` 对应 ID 42 |
| 物理尺寸 | 20mm×20mm（建议） |
| 版本 | QR Code Version 1~3（21×21 至 29×29 模块） |
| 纠错等级 | M（15%） |
| 贴附位置 | 每本书书脊底部 |
| 打印要求 | 白底黑码，边缘留 4 模块宽的白边（静区） |

### 2.5 距离估算方法

QR 码无法像 AprilTag 那样直接输出 6DOF 姿态，采用像素宽度比例法：

```
已知：QR码物理边长 real_size = 20mm
测得：QR码在图像中的像素宽度 pixel_width（取 w 和 h 中较大者，或对角线）

focal_length = 相机标定的焦距（像素单位，OpenMV H7 Plus 约 320px @ QVGA）

distance_mm = (real_size * focal_length) / pixel_width
```

> 标定方式：将已知尺寸的 QR 码放在 200mm、300mm、400mm 处，记录相应像素宽度，反求焦距。多次平均。

OpenMV H7 Plus 参考值（QVGA 320×240，镜头 2.8mm）：
- 20mm QR码在 300mm 处 → 约 25~30px
- 焦距约为 (30 * 300) / 20 = 450px

---

## 3. RDK X5 ↔ 机械臂控制器 通信协议

### 3.1 协议概述

机械臂已有自己的固定串口控制协议，不需要重新设计。RDK X5 侧的工作是写一个**适配层（Adapter）**，将 ROS2 层的标准接口（关节角度、夹爪控制）转译为臂控制器支持的协议指令。

- 物理层：UART 串口（或 USB 虚拟串口）
- 协议：**遵循臂控制器厂商手册**，不做修改
- 适配层位置：`arm_controller/arm_driver.py` 中实现

### 3.2 适配层设计

适配层（`ArmDriver` 类）封装臂协议细节，对上层暴露统一接口：

```python
class ArmDriver(ABC):
    """机械臂驱动抽象基类"""
    
    @abstractmethod
    def connect(self, port: str, baudrate: int) -> bool: ...
    
    @abstractmethod
    def set_joint_angles(self, angles: List[float]) -> bool:
        """设置6个关节角度（弧度），阻塞直到指令发送"""
        ...
    
    @abstractmethod
    def get_joint_angles(self) -> Optional[List[float]]:
        """读取当前关节角度"""
        ...
    
    @abstractmethod
    def set_gripper(self, position: float) -> bool:
        """夹爪控制：0.0=全开，1.0=全闭"""
        ...
    
    @abstractmethod
    def get_status(self) -> int:
        """返回状态码：0=空闲 1=忙 2=错误 3=归零中"""
        ...
    
    @abstractmethod
    def emergency_stop(self) -> bool: ...
```

**具体实现**：根据臂控制器的手册，继承 `ArmDriver`，实现 `XArmDriver` / `BusServoDriver` 等具体类。上层 `PickSequence` 状态机只依赖抽象接口，不感知具体协议。

### 3.3 取书动作序列

取书序列由 `pick_sequence.py` 中的状态机管理，每步调用 `ArmDriver` 的抽象接口：

```
步骤1: 移动到就绪位姿 → set_joint_angles(ready_pose)
步骤2: 移动到预抓取位姿 → set_joint_angles(pre_grasp_pose)
步骤3: 直线前进到抓取位姿 → set_joint_angles(grasp_pose)
步骤4: 闭合夹爪 → set_gripper(1.0)
步骤5: 垂直提书 3cm → set_joint_angles(lift_pose)
步骤6: 后退 10cm → set_joint_angles(retreat_pose)
步骤7: 移动到放置位姿 → set_joint_angles(place_pose)
步骤8: 打开夹爪释放 → set_gripper(0.0)
步骤9: 返回就绪位姿 → set_joint_angles(ready_pose)
```

### 3.4 你需要提供的臂协议信息

在实现适配层之前，请确认臂手册中的以下内容：

1. 串口参数（波特率、数据位、停止位、校验）
2. 设置关节角度的指令格式（字节序列）
3. 读取关节角度的指令格式
4. 夹爪控制指令格式
5. 状态查询指令格式
6. 指令的响应/确认方式
