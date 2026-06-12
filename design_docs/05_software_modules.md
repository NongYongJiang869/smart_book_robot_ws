# 智能寻书机器人 — 软件模块详细设计

## 1. 模块总览

```
smart_book_robot_ws/src/
├── custom_interfaces/        # 消息/服务/动作定义（纯定义，无运行时逻辑）
├── stm32_bridge/             # 串口桥接（Python）
├── openmv_bridge/            # OpenMV 通信（Python）
├── arm_controller/           # 机械臂控制（Python）
├── book_search/              # 任务调度（Python）
└── robot_bringup/            # 启动+配置（Launch + YAML + URDF）
```

---

## 2. stm32_bridge 模块

### 2.1 职责

- 与 STM32 通过串口通信
- 发送速度指令，接收里程计+IMU+状态数据
- 发布 `/odom`、`/imu`、`/tf`(odom→base_link)
- 订阅 `/cmd_vel`

### 2.2 文件结构

```
stm32_bridge/
├── CMakeLists.txt
├── package.xml
├── stm32_bridge/
│   ├── __init__.py
│   ├── bridge_node.py          # ROS2 节点主入口
│   ├── serial_protocol.py      # 串口帧协议编解码
│   └── odometry.py             # 里程计计算
├── config/
│   └── stm32_params.yaml       # 标定参数
└── launch/
    └── stm32_bridge.launch.py
```

### 2.3 类设计

```python
# serial_protocol.py
class SerialProtocol:
    """帧协议编解码器"""
    HEADER = b'\x5A\xA5'
    
    def encode_vel_cmd(self, linear_x: float, angular_z: float, seq: int) -> bytes: ...
    def decode_frame(self, data: bytes) -> Optional[Tuple[int, bytes]]: ...  # (type, payload)
    def decode_odom_data(self, payload: bytes) -> dict: ...
    def decode_status(self, payload: bytes) -> dict: ...
    @staticmethod
    def crc16_ccitt(data: bytes) -> int: ...

# odometry.py
class OdometryComputer:
    """从编码器+IMU计算里程计"""
    def __init__(self, params: dict):
        self.wheel_circumference = params['wheel_circumference']
        self.wheel_base = params['wheel_base']
        self.counts_per_rev = params['counts_per_rev']
    
    def update(self, left_enc, right_enc, gyro_z, dt) -> Tuple[float, float, float]:
        """返回 (x, y, yaw) 增量"""
        ...

# bridge_node.py
class STM32BridgeNode(Node):
    """主节点"""
    def __init__(self):
        # 参数
        self.declare_parameters(...)
        
        # 串口
        self.serial = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.01)
        
        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_sensor_data)
        
        # 订阅者
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # TF 广播
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时器
        self.read_timer = self.create_timer(0.02, self.read_serial_loop)   # 50Hz 读串口
        self.cmd_timer = self.create_timer(0.01, self.send_vel_loop)       # 100Hz 发速度
    
    def read_serial_loop(self):
        """持续读取串口，解析帧，发布数据"""
        ...
    
    def send_vel_loop(self):
        """周期性发送速度指令"""
        ...
    
    def cmd_callback(self, msg: Twist):
        """接收外部速度指令，缓存供 send_vel_loop 使用"""
        ...
```

### 2.4 配置参数（stm32_params.yaml）

```yaml
stm32_bridge:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    wheel_circumference: 0.478    # 轮子周长 (m)，需实测
    wheel_base: 0.35              # 左右轮间距 (m)，需实测
    counts_per_rev: 1560          # 编码器每圈脉冲数
    max_linear_vel: 0.5           # 限速 (m/s)
    max_angular_vel: 1.0          # 限角速度 (rad/s)
    cmd_timeout_ms: 200           # 速度指令超时 (ms)
    odom_frame: "odom"
    base_frame: "base_link"
```

### 2.5 关键行为

- **超时保护**：若 200ms 内未收到新的 `/cmd_vel`，发送零速指令
- **异常恢复**：串口读取异常时，标记里程计为不可靠，发布告警
- **帧序号**：每发一帧 `seq+=1`，STM32 侧检测丢帧，连续丢 10 帧触发告警

---

## 3. openmv_bridge 模块

### 3.1 职责

- 通过 USB 串口与 OpenMV 通信
- 接收书籍检测结果，发布 `/book_detection`
- 发送控制命令给 OpenMV（切换模式、调参）

### 3.2 文件结构

```
openmv_bridge/
├── CMakeLists.txt
├── package.xml
├── openmv_bridge/
│   ├── __init__.py
│   ├── openmv_bridge_node.py    # ROS2 节点
│   └── openmv_protocol.py       # 文本协议解析
├── config/
│   └── openmv_params.yaml
└── launch/
    └── openmv_bridge.launch.py
```

### 3.3 OpenMV 端程序（H7 Plus）

OpenMV Cam H7 Plus 基于 MicroPython，需要单独烧录：

```python
# openmv_firmware/book_detector.py
import sensor, image, time
from pyb import UART

# 初始化
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # QR检测推荐灰度
sensor.set_framesize(sensor.QVGA)       # 320x240
sensor.set_windowing((320, 240))
sensor.skip_frames(time=2000)
uart = UART(3, 115200)

while True:
    img = sensor.snapshot()
    
    # QR 码检测（H7 Plus OpenMV 固件 4.x 支持）
    qr_codes = img.find_qrcodes()
    
    if qr_codes:
        for qr in qr_codes:
            # qr.rect() 返回 (x, y, w, h)
            x, y, w, h = qr.rect()
            cx, cy = x + w // 2, y + h // 2
            line = f"DETECT:{qr.payload()};" \
                   f"x={cx};y={cy};" \
                   f"w={w};h={h};" \
                   f"confidence={qr.quality():.2f}\n"
            uart.write(line)
    else:
        uart.write("DETECT:none\n")
    
    # 检查命令（非阻塞）
    if uart.any():
        cmd = uart.readline()
        if cmd:
            cmd_str = cmd.decode().strip()
            # 解析 CMD:search_qr;content=book_042 等命令
            if cmd_str.startswith("CMD:search_qr"):
                # 提取目标QR内容，切换为搜索模式
                ...
            elif cmd_str.startswith("CMD:continuous_detect"):
                # 切换为连续检测模式
                ...
```

### 3.4 openmv_bridge 节点中的距离/角度估算

由于 QR 码不能像 AprilTag 那样直接返回 6DOF 姿态，距离和角度在 ROS2 侧估算：

```python
# openmv_bridge_node.py 中的估算逻辑
class QRDistanceEstimator:
    def __init__(self, focal_length_px: float, qr_real_size_mm: float):
        self.focal_length = focal_length_px   # 标定得到，H7 Plus QVGA ≈ 450
        self.real_size = qr_real_size_mm       # 20mm
    
    def estimate(self, width_px: float, height_px: float, center_x: float, img_w: int = 320):
        """根据QR码像素尺寸估算距离和角度"""
        # 距离：取宽高中较大者（更稳定）
        pixel_size = max(width_px, height_px)
        if pixel_size <= 0:
            return -1.0, -1.0
        distance_mm = (self.real_size * self.focal_length) / pixel_size
        distance_m = distance_mm / 1000.0
        
        # 角度：由水平位置偏离图像中心的程度估算
        # small angle approx: angle ≈ (cx - center) / focal_length * 180/π
        angle_deg = (center_x - img_w / 2.0) / self.focal_length * 57.3
        
        return distance_m, angle_deg
```

### 3.5 关键配置

```yaml
openmv_bridge:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    qr_real_size_mm: 20            # QR码物理边长(mm)
    focal_length_px: 450           # 相机焦距(px)，需实际标定
    img_width: 320                 # 图像宽度(QVGA)
    img_height: 240                # 图像高度(QVGA)
    detection_interval_ms: 100     # 检测间隔
```

---

## 4. arm_controller 模块

### 4.1 职责

- 与机械臂控制器通信
- 执行取书动作序列
- 发布 `/joint_states`
- 提供 `/pick_book` Action 服务端
- 提供 `/arm_command` Service 服务端

### 4.2 文件结构

```
arm_controller/
├── CMakeLists.txt
├── package.xml
├── arm_controller/
│   ├── __init__.py
│   ├── arm_controller_node.py    # ROS2 节点
│   ├── arm_driver.py             # 硬件驱动抽象层
│   ├── pick_sequence.py          # 取书动作序列状态机
│   └── kinematics.py             # 逆运动学（如需）
├── config/
│   ├── arm_params.yaml
│   └── pick_positions.yaml       # 预定义取书位置
└── launch/
    └── arm_controller.launch.py
```

### 4.3 取书动作序列（PickSequence）

```
           ┌─────────┐
           │  IDLE   │
           └────┬────┘
                │ pick_book action received
                ▼
           ┌─────────┐
           │  READY  │  移动到就绪位姿
           └────┬────┘
                │ 到达就绪位姿
                ▼
           ┌─────────────┐
           │  PRE_GRASP  │  移动到书前方5cm
           └──────┬──────┘
                  │ 到位
                  ▼
           ┌─────────────┐
           │   GRASP     │  前进→闭爪
           └──────┬──────┘
                  │ 抓取成功（夹爪力反馈）
                  ▼
           ┌─────────────┐
           │   LIFT      │  上提3cm
           └──────┬──────┘
                  │
                  ▼
           ┌─────────────┐
           │   RETREAT   │  后退10cm
           └──────┬──────┘
                  │
                  ▼
           ┌─────────────┐
           │   PLACE     │  移动到载书筐上方→开爪释放
           └──────┬──────┘
                  │
                  ▼
           ┌─────────┐
           │  READY  │  回归就绪位姿
           └────┬────┘
                │
                ▼
           ┌─────────┐
           │  IDLE   │  完成
           └─────────┘
```

### 4.4 动作序列参数（pick_positions.yaml）

```yaml
pick_positions:
  ready:       [0.0, 0.0, -1.57, 0.0, 1.57, 0.0]   # 各关节角度(rad)
  pre_grasp:   [0.1, 0.3, -1.2, 0.0, 1.0, 0.0]     # 根据实际示教修改
  lift_height: 0.03   # 提书高度(m)
  retreat_dist: 0.10  # 后退距离(m)
```

---

## 5. book_search（任务调度）模块

### 5.1 职责

- 整个系统的主状态机
- 管理与 Nav2、OpenMV、机械臂的协调
- 维护书籍位置数据库
- 对外提供 `/query_book` Service 和 `/pick_book` Action

### 5.2 文件结构

```
book_search/
├── CMakeLists.txt
├── package.xml
├── book_search/
│   ├── __init__.py
│   ├── book_search_master.py     # ROS2 主节点
│   ├── state_machine.py          # 任务状态机
│   └── book_database.py          # 书籍位置数据库管理
├── config/
│   ├── book_database.yaml        # 书籍→位置映射
│   └── shelf_map.yaml            # 书架布局
└── launch/
    └── book_search.launch.py
```

### 5.3 主状态机

```
                    ┌──────────┐
         ┌─────────→│   IDLE   │←──────────┐
         │          └────┬─────┘           │
         │               │ pick_book       │
         │               ▼                 │
         │          ┌──────────────┐       │
         │          │ QUERYING_BOOK│       │
         │          │  查询书籍位置 │       │
         │          └──────┬───────┘       │
         │                 │ found         │
         │                 ▼              │
         │          ┌──────────────┐       │
         │          │ NAVIGATING   │       │
         │          │ TO_SHELF     │───────┤
         │          │ 接近书架区域  │       │
         │          └──────┬───────┘       │
         │                 │ arrived       │
         │                 ▼              │
         │          ┌──────────────┐       │
         │          │ SCANNING     │       │
         │          │ 视觉搜索书籍  │       │
         │          └──────┬───────┘       │
         │                 │ detected      │
         │                 ▼              │
         │          ┌──────────────┐       │
         │          │ APPROACHING  │       │
         │          │ 精确对位     │───────┤
         │          └──────┬───────┘       │
         │                 │ aligned       │
         │                 ▼              │
         │          ┌──────────────┐       │
         │          │ PICKING      │       │
         │          │ 机械臂取书   │───────┤
         │          └──────┬───────┘       │
         │                 │ done          │
         │                 ▼              │
         │          ┌──────────────┐       │
         │          │ RETURNING    │       │
         │          │ 导航返回前台  │───────┤
         │          └──────┬───────┘       │
         │                 │ arrived       │
         │                 ▼              │
         │          ┌──────────────┐       │
         └──────────│  COMPLETED   │       │
                    └──────────────┘       │
                                           │
           任何状态 ────→ ┌──────────┐     │
                          │  ERROR   │─────┘
                          └──────────┘
```

### 5.4 状态转移表

| 当前状态 | 触发条件 | 下一状态 | 动作 |
|----------|----------|----------|------|
| IDLE | 收到 PickBook Goal | QUERYING_BOOK | 查数据库 |
| QUERYING_BOOK | 找到书籍 | NAVIGATING_TO_SHELF | 发送导航目标 |
| QUERYING_BOOK | 未找到 | IDLE | 回复错误 |
| NAVIGATING_TO_SHELF | 到达目标 | SCANNING | 启动视觉搜索 |
| NAVIGATING_TO_SHELF | 导航失败（重试3次）| ERROR | 报告导航失败 |
| SCANNING | 检测到目标书 | APPROACHING | 切换到视觉伺服 |
| SCANNING | 超时30s未找到 | ERROR | 报告未找到 |
| APPROACHING | 对位完成（距离<20cm且居中）| PICKING | 停止底盘，启动取书 |
| APPROACHING | 超时20s | ERROR | 报告对位失败 |
| PICKING | 取书完成 | RETURNING | 发送返航目标 |
| PICKING | 取书失败 | ERROR | 报告取书失败 |
| RETURNING | 到达前台 | COMPLETED | 通知用户取书完成 |
| ERROR | 接收到清除命令 | IDLE | 重置所有模块 |

### 5.5 书籍数据库设计

```yaml
# book_database.yaml
books:
  - id: 1
    name: "三体"
    author: "刘慈欣"
    shelf_id: 3
    row: 2        # 第2层（从下往上）
    col: 5        # 第5列（从左往右）
    
  - id: 2
    name: "活着"
    author: "余华"
    shelf_id: 1
    row: 3
    col: 12

shelves:
  - id: 1
    name: "A区-文学区"
    pose: {x: 2.5, y: 0.0, yaw: 0.0}    # 在地图坐标系中的位姿
    width: 2.0
    num_rows: 4
    num_cols: 15
    row_height: [0.2, 0.5, 0.8, 1.1]     # 每层距地面高度(m)
    col_width: 0.12                        # 每列宽度(m)
    
  - id: 3
    name: "B区-科幻区"
    pose: {x: 5.0, y: 0.0, yaw: 0.0}
    width: 2.0
    num_rows: 4
    num_cols: 15
    row_height: [0.2, 0.5, 0.8, 1.1]
    col_width: 0.12

# 前台/起点位置
home:
  pose: {x: 0.0, y: 0.0, yaw: 0.0}
```

### 5.6 书籍查询与坐标计算

```python
# book_database.py
class BookDatabase:
    def query(self, book_name: str) -> Optional[dict]:
        """模糊搜索书名，返回书籍位置信息"""
        ...
    
    def get_book_map_pose(self, book_entry: dict) -> Tuple[float, float, float, float]:
        """计算书籍在 map 坐标系中的坐标和臂需要到达的高度
        返回: (target_x, target_y, target_yaw, shelf_height)
        
        计算方式：
        shelf_pose = shelves[book_entry.shelf_id].pose
        target_x = shelf_pose.x + (book_entry.col - 1) * col_width
        target_y = shelf_pose.y  (走廊侧)
        target_yaw = shelf_pose.yaw
        shelf_height = row_height[book_entry.row - 1]
        """
        ...
```

### 5.7 视觉伺服对位算法

```python
# state_machine.py 中的 APPROACHING 阶段

class ApproachingState:
    """基于 OpenMV 检测结果做精确对位"""
    
    def execute(self, detection: BookDetection) -> Optional[Twist]:
        """
        输入：/book_detection 话题的最新值
        输出：发给 /cmd_vel 的速度指令，若对位完成返回 None
        
        控制策略：
        1. 角度伺服：根据 detection.angle_deg 做 P 控制 → angular_z
        2. 距离伺服：根据 detection.distance_m 做 P 控制 → linear_x  
        3. 对位完成条件：|angle| < 2° 且 |distance - target_dist| < 1cm
        """
        kp_angle = 0.5
        kp_dist = 0.3
        target_dist = 0.25   # 期望停靠在书前方25cm
        
        if abs(detection.angle_deg) < 2.0 and abs(detection.distance_m - target_dist) < 0.01:
            return None  # 对位完成
        
        cmd = Twist()
        cmd.angular.z = kp_angle * detection.angle_deg
        cmd.linear.x = kp_dist * (detection.distance_m - target_dist)
        return cmd
```

---

## 6. robot_bringup 模块

### 6.1 职责

- 统一启动所有节点
- 提供不同模式（建图、导航、完整）的启动文件
- 管理 URDF 模型和 Nav2/SLAM 配置

### 6.2 启动文件

```
robot_bringup/launch/
├── robot_bringup.launch.py      # 完整启动（全部节点）
├── mapping.launch.py            # 建图模式（LiDAR + SLAM + 底盘 + 键盘遥控）
├── navigation.launch.py         # 导航模式（LiDAR + Nav2 + 底盘，需已有地图）
├── book_search.launch.py        # 取书模式（导航 + OpenMV + 机械臂 + 任务调度）
└── include/
    ├── lidar.launch.py          # LiDAR 驱动
    ├── chassis.launch.py        # 底盘驱动（stm32_bridge）
    ├── openmv.launch.py         # OpenMV 驱动
    └── arm.launch.py            # 机械臂驱动
```

### 6.3 mapping.launch.py 典型内容

```python
def generate_launch_description():
    return LaunchDescription([
        # 底盘驱动
        IncludeLaunchDescription('include/chassis.launch.py'),
        # LiDAR
        IncludeLaunchDescription('include/lidar.launch.py'),
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            parameters=['config/mapper_params_online_async.yaml'],
        ),
        # 键盘遥控（建图时用）
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e',
        ),
    ])
```

---

## 7. Nav2 配置要点

### 7.1 关键参数（nav2_params.yaml）

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3           # 导航模式限速
      max_vel_theta: 0.8

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.05          # 目标点容差 5cm

local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.3        # 机器人半径（用于避障膨胀）
      inflation_layer:
        inflation_radius: 0.35
```

---

## 8. SLAM 配置要点

```yaml
# mapper_params_online_async.yaml
slam_toolbox:
  ros__parameters:
    mode: mapping              # mapping 或 localization
    map_file_name: library_map
    map_start_pose: [0.0, 0.0, 0.0]
    map_update_interval: 2.0
    resolution: 0.05           # 5cm/格
    max_laser_range: 10.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
```
