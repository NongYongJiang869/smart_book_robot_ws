# 智能寻书机器人 — RDK X5 40pin 外设编程指南

> 本文档基于 RDK X5 官方示例程序 (`/app/40pin_samples/`) 编写，覆盖 GPIO、UART、PWM、I2C、SPI 和中断的 Python 编程模式，供 ROS2 节点开发时参考。

## 1. 关键发现：Hobot.GPIO ≠ RPi.GPIO

RDK X5 使用 **`Hobot.GPIO`**（地平线自研），**不是**树莓派的 `RPi.GPIO`。API 类似但命名不同：

| 功能 | Hobot.GPIO (RDK X5) | RPi.GPIO (树莓派) |
|------|---------------------|-------------------|
| 导入 | `import Hobot.GPIO as GPIO` | `import RPi.GPIO as GPIO` |
| 编码模式 | `GPIO.BOARD` / `GPIO.BCM` | `GPIO.BOARD` / `GPIO.BCM` |
| 设置方向 | `GPIO.setup(pin, GPIO.OUT)` | `GPIO.setup(pin, GPIO.OUT)` |
| 输出 | `GPIO.output(pin, GPIO.HIGH)` | `GPIO.output(pin, GPIO.HIGH)` |
| 输入 | `value = GPIO.input(pin)` | `value = GPIO.input(pin)` |
| PWM | `p = GPIO.PWM(pin, freq)` | `p = GPIO.PWM(pin, freq)` |
| 沿检测 | `GPIO.add_event_detect(pin, GPIO.FALLING, callback=fn, bouncetime=10)` | 同 |
| 等待沿 | `GPIO.wait_for_edge(pin, GPIO.FALLING)` | 同 |
| 清理 | `GPIO.cleanup()` | `GPIO.cleanup()` |
| **警告** | `GPIO.setwarnings(False)` | `GPIO.setwarnings(False)` |

**不支持** `GPIO.BCM` 中断 — 非标准 GPIO 库的部分功能可能行为不同。

## 2. 40pin 引脚资源

### 2.1 可用 GPIO 引脚 (28个)

所有可用 GPIO（BOARD 编号）：

```
引脚号:  3   5   7   8  10  11  12  13  15  16  18  19  21  22
        23  24  26  27  28  29  31  32  33  35  36  37  38  40
```

### 2.2 PWM 引脚

根据 `Hobot.GPIO.all_pin_data` 的 `pwm_chip_dir` 字段，以下引脚具有 PWM 能力：

```
18, 27, 28, 29, 31, 32, 33, 37
```

示例代码特别提到 **32 和 33** 最稳定。频率范围：**RDK X5: 0.05Hz ~ 1MHz**（比 X3 的 48KHz~192MHz 更宽）。

### 2.3 专用接口

| 接口 | 设备路径 | 库 |
|------|----------|-----|
| UART | `/dev/ttyS0` ~ `/dev/ttyS7` | `serial` (pyserial) |
| I2C | `/dev/i2c-0`, `/dev/i2c-2` ~ `/dev/i2c-8` | `i2cdev` |
| SPI | `/dev/spidev1.0`, `/dev/spidev1.1` | `spidev` |

> **注意**：UART/I2C/SPI 是 SoC 硬件外设，通过 Linux 设备驱动访问，**不经过** `Hobot.GPIO` 库。40pin 排针上特定的物理引脚对应特定的 UART/I2C/SPI 控制器，具体映射需查阅 RDK X5 40pin 排针定义文档。

## 3. GPIO 编程模式

### 3.1 输出模式

```python
import Hobot.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)  # 初始化为高电平

# 循环控制
GPIO.output(output_pin, GPIO.HIGH)
GPIO.output(output_pin, GPIO.LOW)

GPIO.cleanup()  # 释放所有 GPIO
```

### 3.2 输入模式

```python
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)

value = GPIO.input(input_pin)  # 返回 GPIO.HIGH (1) 或 GPIO.LOW (0)

GPIO.cleanup()
```

### 3.3 输入轮询（检测变化）

```python
prev_value = None
while True:
    value = GPIO.input(input_pin)
    if value != prev_value:
        state = "HIGH" if value == GPIO.HIGH else "LOW"
        print(f"Pin changed to {state}")
        prev_value = value
    time.sleep(0.01)  # 避免占用 CPU
```

### 3.4 PWM 输出

> **只有引脚 32、33 支持 PWM。使用 PWM 时，必须确保该引脚没有被其他功能占用。**

```python
GPIO.setmode(GPIO.BOARD)
p = GPIO.PWM(33, 48000)       # 引脚33, 频率 48KHz
p.start(25)                    # 初始占空比 25%
p.ChangeDutyCycle(50)          # 动态修改占空比
p.stop()                       # 停止 PWM
GPIO.cleanup()
```

### 3.5 阻塞等待沿事件

```python
GPIO.setmode(GPIO.BOARD)
GPIO.setup(but_pin, GPIO.IN)

print("Waiting for edge...")
GPIO.wait_for_edge(but_pin, GPIO.FALLING)  # 阻塞直到下降沿
print("Edge detected!")
```

### 3.6 中断回调（非阻塞）

```python
def my_callback(channel):
    """中断处理函数，尽量简短"""
    print(f"Interrupt on channel {channel}")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(but_pin, GPIO.IN)

# 注册中断回调：下降沿触发，10ms 消抖
GPIO.add_event_detect(but_pin, GPIO.FALLING, callback=my_callback, bouncetime=10)

# 主循环继续运行
while True:
    time.sleep(1)
```

> **注意**：`bouncetime` 单位为毫秒，在该时间内重复触发会被忽略。

## 4. UART 编程模式

RDK X5 的 UART 使用 Python `serial` 库（pyserial），与标准 Linux 串口编程一致。

```python
import serial

# 列举可用串口
# 物理串口: /dev/ttyS0 ~ /dev/ttyS7
# USB串口:  /dev/ttyUSB0, /dev/ttyACM0 等

ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)  # 1秒超时

# 发送
test_data = "AA55"
write_num = ser.write(test_data.encode('UTF-8'))

# 接收（阻塞读取指定长度）
received_data = ser.read(write_num).decode('UTF-8')

# 接收（读取一行）
line = ser.readline()

ser.close()
```

### 4.1 本项目 UART 规划

| 串口 | RDK X5 侧 | 外设 | 波特率 | 协议 |
|------|-----------|------|--------|------|
| UART1 | **`/dev/ttyS1`** ✅ | STM32 (底盘) | 115200 | 二进制帧 + CRC16 |
| UART2 | `/dev/ttyS?` (待确认) | 机械臂控制器 | 115200 | 文本指令 (`$KMS...!` 等) |
| USB1 | `/dev/ttyUSB0` | YDLidar | 由驱动管理 | LiDAR 私有协议 |
| USB2 | `/dev/ttyACM0` | OpenMV | 115200 | 文本行协议 |

> **已确认**: RDK X5 ↔ STM32 底盘通信使用 `/dev/ttyS1`。

### 4.2 串口权限

```bash
# 将当前用户加入 dialout 组（可能需要重启）
sudo usermod -a -G dialout $USER

# 或设置 udev 规则固定设备名和权限
# /etc/udev/rules.d/99-robot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="stm32_bridge", MODE="0666"
```

## 5. I2C 编程模式

```python
from i2cdev import I2C

# 扫描 I2C 总线
# os.system('i2cdetect -y -r <bus_num>')

# 打开 I2C 设备 (地址需要 0x 前缀 eval 转换)
i2c = I2C(0x51, 0)          # 设备地址 0x51, I2C bus 0

# 读写
value = i2c.read(1)          # 读 1 字节
i2c.write([0x01, 0x02])      # 写多个字节

i2c.close()
```

> 本项目可能用 I2C 连接 IMU（如 MPU6050, 地址 0x68）或其他传感器。

## 6. SPI 编程模式

```python
import spidev

spi = spidev.SpiDev()
spi.open(bus, device)                # 如 spi.open(1, 0) → /dev/spidev1.0
spi.max_speed_hz = 12000000          # 12 MHz

# 全双工传输：发送的同时接收
resp = spi.xfer2([0x55, 0xAA])       # 返回接收到的字节列表

spi.close()
```

## 7. 信号处理与安全退出

所有示例都使用了标准的 `signal` 处理 CTRL+C，确保退出时 `GPIO.cleanup()` 被调用：

```python
import signal

def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
```

在 ROS2 节点中，应在节点的 `on_shutdown()` 或 `destroy_node()` 中调用 `GPIO.cleanup()`。

## 8. 本项目应用映射

### 8.1 急停按钮（GPIO 输入 + 中断）

```python
# 急停按钮 → 物理触发电平变化
# 使用中断方式，确保毫秒级响应
EMERGENCY_PIN = 37  # BOARD 编码（示例，需实际接线确认）

def emergency_callback(channel):
    """急停回调：发布零速指令，设置系统 ERROR 状态"""
    # 在 ROS2 回调中发布 /cmd_vel 零速
    # 调用 book_search_master 的紧急停止服务
    pass

GPIO.setup(EMERGENCY_PIN, GPIO.IN)
GPIO.add_event_detect(EMERGENCY_PIN, GPIO.FALLING,
                      callback=emergency_callback, bouncetime=50)
```

### 8.2 碰撞传感器（GPIO 输入）

```python
# 前后碰撞传感器 → 2 个 GPIO 输入
COLLISION_FRONT_PIN = 15  # 示例
COLLISION_REAR_PIN  = 16  # 示例

# 在 stm32_bridge 或独立安全节点中轮询
GPIO.setup(COLLISION_FRONT_PIN, GPIO.IN)
GPIO.setup(COLLISION_REAR_PIN, GPIO.IN)

if GPIO.input(COLLISION_FRONT_PIN) == GPIO.LOW:
    # 触发紧急停止
    pass
```

### 8.3 状态指示灯（GPIO 输出）

```python
STATUS_LED_PIN = 31   # 示例
GPIO.setup(STATUS_LED_PIN, GPIO.OUT)

# IDLE=慢闪, NAVIGATING=快闪, ERROR=常亮, PICKING=双闪
```

### 8.4 蜂鸣器（GPIO 输出）

```python
BUZZER_PIN = 29       # 示例（需确认是普通 GPIO 还是需 PWM）
GPIO.setup(BUZZER_PIN, GPIO.OUT)
# 无源蜂鸣器需要 PWM 驱动不同频率来发出不同音调
# 有源蜂鸣器直接 GPIO.HIGH/GPIO.LOW 即可
```

### 8.5 STM32 串口通信（在 ROS2 节点中）

参考 `03_communication_protocols.md` 中的帧协议定义，在 `stm32_bridge` 节点中使用 `serial` 库实现：

```python
import serial

class STM32Bridge:
    def __init__(self, port='/dev/ttyS1', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        # 实现 0x5A 0xA5 帧协议...
```

### 8.6 机械臂串口通信

机械臂使用文本协议 (`$KMS:`, `#xxxPxxx`, `$QSTAT!` 等)，参考 `03_communication_protocols.md` 第 3 节。

```python
# arm_controller 节点中
self.ser = serial.Serial('/dev/ttyS2', 115200, timeout=0.5)
self.ser.write(b"$KMS:150,0,80,1000!")
```

## 9. 注意事项

1. **引脚复用**：RDK X5 的 40pin 排针上，某些物理引脚同时连接了 GPIO 和 UART/I2C/SPI。启用专用接口时，对应的 GPIO 功能不可用。
2. **PWM 引脚限制**：只有 32、33 是最稳定的 PWM 引脚，其他有 `pwm_chip_dir` 的引脚（18, 27, 28, 29, 31, 37）在不同硬件版本上可能不可用。
3. **ROS2 与 GPIO 共存**：`Hobot.GPIO` 通过 sysfs 操作 GPIO，与 ROS2 节点无冲突。但在 ROS2 回调中做 GPIO 操作时要注意不要阻塞。
4. **中断安全性**：`GPIO.add_event_detect` 的回调在单独的轮询线程中执行，不要在回调中做长时间操作。
5. **电压**：RDK X5 GPIO 为 3.3V 电平。与 5V 设备（如 Arduino 机械臂控制器）连接时需要电平转换。

## 10. 参考

- RDK X5 40pin 示例目录：`/app/40pin_samples/`
- `Hobot.GPIO` 源码：`/usr/lib/python3/dist-packages/Hobot/GPIO/`
- pyserial 文档：https://pyserial.readthedocs.io/
- 本项目通信协议：`03_communication_protocols.md`
- 本项目软件模块：`05_software_modules.md`
