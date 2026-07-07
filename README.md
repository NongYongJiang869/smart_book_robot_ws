# Smart Book-Finding Robot · 智能寻书机器人

基于 ROS2 Humble 的自主寻书机器人系统。机器人能够在图书馆环境中自主导航，通过识别书脊上的二维码定位目标书籍，并使用六轴机械臂完成抓取与投递。

## 系统概览

```
┌──────────────────────────────────────────────────────────┐
│                      RDK X5 (主控)                        │
│  ROS2 Humble · 任务状态机 · 导航规划 · 视觉伺服 · 机械臂控制  │
├──────────────────────────────────────────────────────────┤
│  STM32 底盘 │ YDLidar 2D │ OpenMV H7 │ 6轴机械臂(Arduino)  │
│  UART串口   │ USB接口     │ USB虚拟串口│ UART串口            │
└──────────────────────────────────────────────────────────┘
```

- **RDK X5** — 运行 ROS2 Humble，负责 SLAM 建图定位、路径规划、任务调度、视觉识别和机械臂控制
- **STM32F103C8T6** — 底盘电机驱动与编码器数据采集，通过 UART 与 RDK X5 通信
- **YDLidar** — 2D 激光雷达，用于 SLAM 建图和障碍物检测
- **OpenMV Cam H7 Plus** — 二维码视觉识别，估计书本距离与角度
- **6 轴机械臂 (Arduino)** — 书本抓取与放置，内置逆运动学解算

## 仓库结构

```
smart_book_robot_ws/
├── RDKX5/                          # ROS2 Humble 工作空间
│   └── src/
│       ├── stm32_bridge/           # STM32 串口桥接（协议编解码/里程计/tf）
│       ├── robot_task_manager/     # 任务管理器（状态机/导航/机械臂控制/图书数据库）
│       ├── custom_interfaces/      # 自定义 ROS2 消息与服务定义
│       ├── ydlidar_ros2_driver/    # YDLidar ROS2 驱动
│       └── rf2o_laser_odometry/    # 激光里程计
├── chassis/                        # STM32F103C8T6 底盘固件
│   ├── inc/                        # 头文件（motor/encoder/serial/imu）
│   ├── src/                        # 源文件（main.c/motor.c/encoder.c 等）
│   ├── startup/                    # 启动汇编
│   ├── stdlib/                     # CMSIS + STM32F10x StdPeriph Driver
│   ├── Makefile                    # arm-none-eabi-gcc 构建
│   └── README.md                   # 引脚分配与定时器说明
├── 6zrobotic_arm/                  # 六轴机械臂 Arduino 固件
│   └── source.c                    # 逆运动学/舵机控制/串口协议
├── Lidar/                          # 激光雷达 SDK 与驱动工作空间
│   ├── YDLidar-SDK-master/         # YDLidar C++ SDK
│   └── yahboomcar_ws/              # ROS2 驱动工作空间
├── openmv/                         # OpenMV 二维码识别固件
├── tools/                          # 调试与标定工具
│   ├── chassis_serial_monitor.py   # STM32 串口监控
│   ├── odometry_test.py            # 里程计测试与轮距自动标定
│   ├── calibrate_motors.py         # 电机标定
│   ├── robotic_arm.py              # 机械臂调试工具
│   ├── map_editor_gui.py           # 地图禁区编辑器（GUI）
│   └── edit_map.py                 # 地图禁区编辑器（CLI）
└── CLAUDE.md                       # AI 编程助手指令（Claude Code）
```

## 硬件规格

| 参数 | 值 |
|------|-----|
| 最大线速度 | 0.5 m/s |
| 最大角速度 | 1.0 rad/s |
| 导航精度 (Nav2) | ±5 cm |
| 导航精度 (视觉伺服) | ±1 cm |
| 单次取书时间目标 | ≤3 分钟 |
| 成功率目标 | ≥95% |

## 快速开始

### 环境要求

- **RDK X5**（或兼容的 Linux 开发板）
- **操作系统**：Ubuntu 22.04（已刷入 RDK X5）
- **ROS2**：Humble Hawksbill
- **Python**：≥3.8
- **STM32 工具链**：`arm-none-eabi-gcc`（仅编译底盘固件时需要）

### 1. 编译 ROS2 工作空间

```bash
cd RDKX5
source /opt/ros/humble/setup.bash
colcon build
```

### 2. 编译 STM32 底盘固件

```bash
cd chassis
make            # 生成 build/app.elf / app.hex / app.bin
make flash      # 烧录（OpenOCD），或者 make clean 清理
```

### 3. 启动机器人

```bash
# 终端 1：启动底盘桥接
cd RDKX5 && source install/setup.bash
ros2 launch stm32_bridge stm32_bridge.launch.py

# 终端 2：启动导航 + 键盘遥控
ros2 launch stm32_bridge chassis_bringup.launch.py teleop:=true

# 终端 3：启动任务管理器
ros2 launch robot_task_manager task_manager.launch.py use_rviz:=true robot_name:=robot-01
```

### 4. 常用话题

```bash
ros2 topic echo /odom              # 里程计
ros2 topic echo /chassis_status    # 底盘状态
ros2 topic echo /scan              # 激光雷达扫描
ros2 topic echo /book_detection    # 书本检测结果
ros2 topic echo /robot_status      # 机器人状态
ros2 topic echo /joint_states      # 机械臂关节状态
```

## 通信协议

### STM32 串口协议（二进制帧）

- 格式：`0x5A 0xA5` + length + type + seq + payload + CRC16-CCITT
- 波特率：115200 8N1，RDK X5 侧设备为 `/dev/ttyS1`
- 上行数据类型：ODOM(50Hz)、STATUS(10Hz)、ERROR、ACK、HEARTBEAT
- 下行数据类型：VEL_CMD(100Hz)、LED_CTRL、BUZZER、RESET_ODOM 等
- 安全机制：200ms 无速度指令自动刹车

### 机械臂串口协议（文本协议）

- `$KMS:x,y,z,time!` — 逆运动学笛卡尔坐标移动
- `#xxxPyyyyTzzzz!` — 单舵机控制
- `$QSTAT!` — 查询状态
- `$DST!` — 紧急停止
- 详见 [6zrobotic_arm/source.c](6zrobotic_arm/source.c)

### OpenMV 串口协议（文本行）

- 命令格式：`CMD:<name>;<key>=<value>;\n`
- 检测格式：`DETECT:<qr_content>;x=<px>;y=<px>;w=<px>;h=<px>;confidence=<0-1>\n`
- 二维码规格：20mm，内容格式 `book_<ID>`

## 坐标系与 TF 树

```
map → odom → base_footprint → base_link → laser
                                       → camera_link → camera_optical
                                       → arm_base → ... → arm_ee
```

- `map → odom`：由 slam_toolbox 维护
- `odom → base_footprint`：由 stm32_bridge 发布
- 坐标系约定：x=东，y=北，yaw=0° 朝 +x 方向

## 禁区蒙版（Keepout Mask）

支持在不修改 SLAM 地图的前提下添加导航禁区，AMCL 定位不受影响。

```bash
# 启动禁区蒙版发布器（随 bringup 自动启动）
ros2 run stm32_bridge keepout_mask_publisher

# 编辑禁区
python3 tools/map_editor_gui.py    # GUI 编辑器
python3 tools/edit_map.py --add-keepout-rect x1 y1 x2 y2  # CLI 编辑
```

## 任务流程

```
IDLE → 收到取书请求 → 路径规划 → 导航至书架 → 扫描书脊二维码
     → 视觉伺服精确对准 → 机械臂抓取 → 导航至目标座位 → 投递书本
     → 返回充电站 → IDLE
```

任务状态机由 `robot_task_manager` 包实现，支持中途取消和异常恢复。

## 工具脚本

| 脚本 | 用途 |
|------|------|
| `tools/chassis_serial_monitor.py` | 监视 STM32 串口输出 |
| `tools/odometry_test.py` | 里程计验证与轮距自动标定（按 `c` 键） |
| `tools/calibrate_motors.py` | PWM-速度映射标定 |
| `tools/robotic_arm.py` | 机械臂串口调试 |
| `tools/map_editor_gui.py` | 禁区蒙版 GUI 编辑器 |
| `tools/edit_map.py` | 禁区蒙版 CLI 编辑器 |
| `tools/pwm_velocity_calib.py` | PWM 速度关系标定 |

## 相关资源

- **底盘引脚分配与定时器**：[chassis/README.md](chassis/README.md)
- **AI 编程助手指令**：[CLAUDE.md](CLAUDE.md)

## 许可证

待定
