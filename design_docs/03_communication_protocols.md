# 智能寻书机器人 — 通信协议设计

## 1. STM32 ↔ RDK X5 串口帧协议

### 1.0 物理连接 ✅ 已确认

| 属性 | STM32 侧 | RDK X5 侧 |
|------|----------|-----------|
| 引脚 | PA2 (USART2_TX), PA3 (USART2_RX) | 40pin UART RX/TX |
| 设备路径 | — | **`/dev/ttyS1`** |
| 波特率 | 115200 | 115200 |
| 数据位 | 8 | 8 |
| 停止位 | 1 | 1 |
| 校验位 | 无 | 无 |
| 流控 | 无 | 无 |
| 电平 | 3.3V TTL | 3.3V TTL |

> **选型原因**: USART1 的 PA9/PB6/PB7 被编码器占用，故选用 USART2 (PA2/PA3)。详见 `chassis/README.md`。

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

机械臂已有自己的固定串口控制协议（`source.c`），采用**文本分隔符协议**。RDK X5 侧只需按协议格式发送指令，无需修改臂端固件。

| 属性 | 值 |
|------|-----|
| 接口类型 | UART 串口 |
| 波特率 | **115200** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 格式 | 文本行，分隔符 `$...!` / `#...!` / `{...}` |

### 3.2 机械臂硬件参数

从 `source.c` 中提取：

```c
// setup_kinematics(L0, L1, L2, L3, &kinematics);
setup_kinematics(100, 105, 75, 180, &kinematics);
// 实际值(放大10倍): L0=1000(即100mm), L1=1050(即105mm), L2=750(即75mm), L3=1800(即180mm)
```

| 参数 | 值 | 说明 |
|------|-----|------|
| L0 (底座高度) | 100 mm | 底盘到肩关节 |
| L1 (大臂长度) | 105 mm | 肩到肘 |
| L2 (小臂长度) | 75 mm | 肘到腕 |
| L3 (末端长度) | 180 mm | 腕到夹爪 |
| 舵机数量 | 6 | 引脚 {7, 3, 5, 6, 9, 8} |
| 臂身舵机 | 0~3 号 | 底座旋转 + 肩 + 肘 + 腕（运动学解算） |
| 夹爪舵机 | 4~5 号 | 末端执行器开合 |
| PWM 范围 | 500 ~ 2500 | 标准舵机 |
| 3/4 号舵机 | 反向 (3000-PWM) | 机械结构原因 |

### 3.3 核心指令

#### 3.3.1 运动学移动（最重要）

```
发送: $KMS:x,y,z,time!
```

- `x, y, z` — 末端目标坐标（单位：mm），原点在底座中心
- `time` — 移动时间（ms）
- 臂端自动搜索最佳 Alpha 角（0° ~ -135°）并执行
- **成功**：蜂鸣1声，串口回复 `@KMS_OK,<alpha>!`（alpha 为实际采用的逼近角度）
- **失败**：蜂鸣2声，串口回复 `@KMS_ERR!`，加打 `Can not find best pos!!!`

```
示例: $KMS:150,0,80,1000!
含义: 末端移动到 (150, 0, 80)mm，耗时 1000ms
```

#### 3.3.2 单舵机控制

```
发送: #{id}P{pwm}T{time}!
发送: #255P{pwm}T{time}!    ← 同时控制全部6个舵机
```

- `id` — 舵机编号 0~5
- `pwm` — PWM 值 500~2500
- `time` — 执行时间（ms）

```
示例: #004P2000T500!    ← 4号舵机(夹爪)移到 PWM 2000，耗时500ms
示例: #255P1500T1000!   ← 全部舵机归中
```

#### 3.3.3 停止

```
发送: $DST!         ← 停止所有舵机（保持当前位姿）
发送: $DST:N!       ← 停止指定舵机 N
```

#### 3.3.4 复位

```
发送: $RST!         ← 软复位臂控制器
```

#### 3.3.5 状态查询 `$QSTAT!`（新增）

查询臂当前状态，回复 `@STATUS:` 开头的一行：

```
发送: $QSTAT!
回复: @STATUS:IDLE,x=150,y=0,z=80,alpha=-45!
回复: @STATUS:BUSY,x=150,y=0,z=80,alpha=-45!
回复: @STATUS:ERROR,x=150,y=0,z=80,alpha=-45!
```

| 状态 | 含义 |
|------|------|
| IDLE | 所有舵机停止 + 无动作组在执行 |
| BUSY | 任一舵机正在移动 或 动作组执行中 |
| ERROR | 最近一次 `$KMS` 执行失败 |
| x,y,z | 最近一次 `$KMS` 的目标坐标（mm） |
| alpha | 最近一次 `$KMS` 采用的逼近角（度） |

#### 3.3.6 PWM 查询 `$QPWM!`（新增）

查询所有舵机当前 PWM 值：

```
发送: $QPWM!
回复: @PWM:1500,2200,2500,1800,1500,1500!
// 对应舵机 0,1,2,3,4,5 的当前 PWM
```

#### 3.3.7 动作组（预编程，可选）

```
发送: $DGT:start-end,times!   ← 执行 flash 中预存的动作组
```

> 动作组需提前通过 `<...>` 指令烧录到臂控制器的 W25Q64 Flash 中。正常取书流程不依赖此功能，用 `$KMS` 逐点控制即可。

### 3.4 适配层设计

适配层（`SixAxisArmDriver`）将 ROS2 调用转译为上述文本指令：

```python
class SixAxisArmDriver:
    """基于 source.c 协议的 6 轴机械臂驱动"""
    
    BAUDRATE = 115200
    
    def __init__(self, port: str):
        self.ser = serial.Serial(port, self.BAUDRATE, timeout=0.5)
    
    def move_to(self, x_mm: float, y_mm: float, z_mm: float, time_ms: int) -> bool:
        """运动学移动，发送 $KMS:x,y,z,time!"""
        cmd = f"$KMS:{int(x_mm)},{int(y_mm)},{int(z_mm)},{time_ms}!"
        self.ser.write(cmd.encode())
        # 等待响应（臂端会打印结果）
        ...
    
    def set_gripper(self, pwm: int) -> bool:
        """夹爪控制（舵机4和5）"""
        cmd = f"#255P{pwm}T500!"
        self.ser.write(cmd.encode())
        ...
    
    def open_gripper(self) -> bool:
        """开爪 — 需根据实际机械结构确定 PWM 值"""
        return self.set_gripper(1500)  # 示例值，需示教确认
    
    def close_gripper(self) -> bool:
        """闭爪 — 需根据实际机械结构确定 PWM 值"""
        return self.set_gripper(2000)  # 示例值，需示教确认
    
    def stop(self):
        """紧急停止"""
        self.ser.write(b"$DST!")
    
    def reset(self):
        """软复位"""
        self.ser.write(b"$RST!")
```

### 3.5 取书动作序列（用 $KMS 实现）

```
步骤1: 就绪位姿 → $KMS:ready_x,ready_y,ready_z,1000!
步骤2: 预抓取位姿 → $KMS:pre_x,pre_y,pre_z,1000!
步骤3: 抓取位姿 → $KMS:grasp_x,grasp_y,grasp_z,800!
步骤4: 闭爪 → #004P{close_pwm}T500! + #005P{close_pwm}T500!
步骤5: 上提 → $KMS:grasp_x,grasp_y,grasp_z+30,500!
步骤6: 后退 → $KMS:pre_x,pre_y,pre_z,800!
步骤7: 放置位姿 → $KMS:place_x,place_y,place_z,1000!
步骤8: 开爪 → #004P{open_pwm}T500! + #005P{open_pwm}T500!
步骤9: 就绪位姿 → $KMS:ready_x,ready_y,ready_z,1000!
```

### 3.6 需要示教确定的参数

| 参数 | 说明 | 确定方法 |
|------|------|----------|
| ready 位姿 (x,y,z) | 收拢状态 | 在 `$KMS` 模式下试出安全位姿 |
| pre_grasp 位姿 | 书前方 5cm | 根据书架坐标+书本坐标计算 |
| grasp 位姿 | 抓取点 | 对位完成后计算 |
| place 位姿 | 载书筐上方 | 实测 |
| open_pwm | 开爪 PWM | 示教，夹爪全开时的 PWM |
| close_pwm | 闭爪 PWM | 示教，夹住一本书时的 PWM |
| Alpha 范围 | 有效逼近角 | 当前代码搜索 0°~-135° |
