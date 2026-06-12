# 智能寻书机器人 — ROS2 接口定义

> 基于 ROS2 Humble，所有接口定义在 `custom_interfaces` 包中。

## 1. 包结构

```
custom_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── BookDetection.msg
│   ├── ShelfInfo.msg
│   ├── ArmJointState.msg
│   ├── RobotStatus.msg
│   └── ChassisStatus.msg
├── srv/
│   ├── QueryBook.srv
│   ├── ArmCommand.srv
│   └── SetMode.srv
└── action/
    └── PickBook.action
```

## 2. 自定义消息（Msg）

### 2.1 BookDetection.msg

OpenMV 检测到书籍 QR 码时发布的消息。

```yaml
# 检测时间戳（ROS 时间）
builtin_interfaces/Time stamp

# QR 码内容，如 "book_042"，空字符串表示未检测到
string qr_content

# 在图像中QR码包围框中心坐标（像素）
float32 center_x       # 像素X (0~320)
float32 center_y       # 像素Y (0~240)

# QR码包围框像素尺寸（宽、高）
float32 width_px       # 像素宽度
float32 height_px      # 像素高度

# 相机到书的估计距离（米），由像素宽度+已知物理尺寸估算，-1 表示无法估计
float32 distance_m

# 水平角度偏差（度），由 center_x 偏离图像中心的量估算，左负右正，-1 表示无法计算
float32 angle_deg

# QR 码解码置信度（OpenMV 返回的 match 值）
float32 confidence

# 是否检测到任何 QR 码
bool detected
```

### 2.2 ShelfInfo.msg

书架信息。

```yaml
# 书架唯一编号
uint16 shelf_id

# 书架在地图坐标系中的位姿
float32 pose_x
float32 pose_y
float32 pose_yaw

# 书架尺寸
float32 width_m          # 宽度（米）
float32 height_m         # 高度（米）

# 层数和列数
uint8 num_rows           # 层数（上下几层）
uint8 num_cols           # 列数（每层几个位置）

# 是否有面向走道
uint8 facing_direction   # 0=东 1=南 2=西 3=北
```

### 2.3 ArmJointState.msg

机械臂关节状态。

```yaml
std_msgs/Header header

# 6个关节角度（弧度）
float64 joint_1
float64 joint_2
float64 joint_3
float64 joint_4
float64 joint_5
float64 joint_6

# 夹爪状态：0=全开，1=全闭，-1=未知
float32 gripper_position

# 末端在 arm_base 坐标系下的位姿
float64 ee_x
float64 ee_y
float64 ee_z
float64 ee_roll
float64 ee_pitch
float64 ee_yaw
```

### 2.4 RobotStatus.msg

机器人整体状态。

```yaml
std_msgs/Header header

# 当前状态
uint8 state
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 SCANNING=2
uint8 APPROACHING=3
uint8 PICKING=4
uint8 RETURNING=5
uint8 ERROR=6


# 底盘
float32 linear_velocity     # 当前线速度 (m/s)
float32 angular_velocity    # 当前角速度 (rad/s)

# 当前任务
string  current_task        # 正在找的书名，空闲时为空

# 错误信息
string  error_message       # 错误描述，无错误时为空
```

### 2.5 ChassisStatus.msg

底盘底层状态（stm32_bridge 发布，用于诊断和监控）。

```yaml
std_msgs/Header header

# 4个电机使能状态 (位0~3，1=使能)
uint8 motor_enabled

# 传感器状态
bool emergency_stop      # 急停是否触发
bool collision_front     # 前碰撞传感器
bool collision_rear      # 后碰撞传感器

# 错误码（0=正常）
uint16 error_code
uint16 ERR_NONE=0
uint16 ERR_ESTOP=1
uint16 ERR_FRONT_COLLISION=2
uint16 ERR_REAR_COLLISION=4
uint16 ERR_COMM_TIMEOUT=256
uint16 ERR_IMU_FAULT=1024

# 通信质量
float32 cmd_latency_ms   # 最近一次速度指令的往返延迟
uint16  lost_frames      # 近1秒内丢帧数量
```

---

## 3. 自定义服务（Srv）

### 3.1 QueryBook.srv

查询书籍位置。

```yaml
# --- 请求 ---
string book_name            # 书名（支持模糊搜索）
# 或
uint16 book_id              # 书籍 ID（精确查询）

# --- 响应 ---
bool   success              # 查询成功
string book_name_found      # 匹配到的完整书名
uint16 book_id              # 书籍 ID
float64 shelf_x             # 书架在地图坐标系的 X (m)
float64 shelf_y             # 书架在地图坐标系的 Y (m)
float64 shelf_yaw           # 书架朝向 (rad)
uint8  shelf_row            # 在第几层（从下往上 1~N）
uint8  shelf_col            # 在第几列（从左往右 1~N）
float32 shelf_height_m      # 该层距地面高度 (m)
string message              # 附加信息，如 "找到1个匹配项"
```

### 3.2 ArmCommand.srv

通用机械臂控制服务。

```yaml
# --- 请求 ---
uint8 command
uint8 MOVE_JOINTS=0         # 按关节角度移动
uint8 MOVE_POSE=1           # 按末端位姿移动
uint8 GRIPPER=2             # 夹爪控制
uint8 HOME=3                # 归零
uint8 STOP=4                # 急停

# MOVE_JOINTS 时有效
float64[6] joint_angles     # 目标关节角度 (rad)

# MOVE_POSE 时有效
float64[6] target_pose      # x,y,z,roll,pitch,yaw

# GRIPPER 时有效
float32 gripper_target      # 目标开度 [0.0, 1.0]

# --- 响应 ---
bool   success
string message
```

### 3.3 SetMode.srv

切换系统运行模式。

```yaml
# --- 请求 ---
uint8 mode
uint8 MAPPING=0             # 建图模式
uint8 NAVIGATION=1          # 导航模式
uint8 IDLE=2                # 待机
uint8 EMERGENCY=3           # 紧急模式

# --- 响应 ---
bool success
```

---

## 4. 自定义动作（Action）

### 4.1 PickBook.action

取书动作，长时间执行，需要反馈。

```yaml
# --- 目标 (Goal) ---
uint16 book_id              # 要取的书籍 ID
string book_name            # 书名（备选）
float64 approach_x          # 接近位置 X（地图坐标）
float64 approach_y          # 接近位置 Y（地图坐标）
float64 approach_yaw        # 接近位置朝向

# --- 结果 (Result) ---
bool success                # 是否成功取到书
uint8 error_code            # 错误码（0=成功）
string error_message        # 错误描述

# --- 反馈 (Feedback) ---
uint8 phase                 # 当前阶段
uint8 PHASE_NAV=0           #   导航到书架
uint8 PHASE_SCAN=1          #   视觉搜索书籍
uint8 PHASE_ALIGN=2         #   精确对位
uint8 PHASE_PICK=3          #   机械臂取书
uint8 PHASE_RETURN=4        #   返回前台
uint8 PHASE_DONE=5          #   完成

float32 progress            # 当前阶段进度 [0.0, 1.0]
string  status_text         # 状态文本，如 "正在导航到第3排书架"
```

---

## 5. Topic 定义汇总

### 5.1 传感器话题（发布）

| Topic | 消息类型 | QoS | 频率 | 发布者 | 说明 |
|-------|----------|-----|------|--------|------|
| `/scan` | `sensor_msgs/LaserScan` | Sensor Data | 10Hz | `ydlidar_node` | LiDAR扫描数据 |
| `/odom` | `nav_msgs/Odometry` | Sensor Data | 50Hz | `stm32_bridge` | 里程计 |
| `/imu` | `sensor_msgs/Imu` | Sensor Data | 50Hz | `stm32_bridge` | IMU数据 |
| `/book_detection` | `custom_interfaces/BookDetection` | Sensor Data | 10Hz | `openmv_bridge` | 书籍检测结果 |
| `/joint_states` | `sensor_msgs/JointState` | Sensor Data | 20Hz | `arm_controller` | 关节状态 |
| `/robot_status` | `custom_interfaces/RobotStatus` | Status | 2Hz | `book_search_master` | 机器人整体状态 |
| `/tf` | `tf2_msgs/TFMessage` | Default | 动态 | 各节点 | 坐标变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | Transient Local | 一次性 | `robot_bringup` | 静态坐标变换 |

### 5.2 控制话题（订阅）

| Topic | 消息类型 | QoS | 订阅者 | 说明 |
|-------|----------|-----|--------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | System Default | `stm32_bridge` | 底盘速度指令 |
| `/led_control` | `std_msgs/Int8` | System Default | `stm32_bridge` | 灯光控制 |

### 5.3 QoS 策略说明

| 策略名称 | Reliability | Durability | History |
|----------|-------------|------------|---------|
| Sensor Data | Best Effort | Volatile | Keep Last(5) |
| System Default | Reliable | Volatile | Keep Last(10) |
| Status | Reliable | Transient Local | Keep Last(1) |
| Transient Local | Reliable | Transient Local | Keep Last(1) |

---

## 6. Service 定义汇总

| Service | 服务端 | 客户端 | 说明 |
|---------|--------|--------|------|
| `/query_book` | `book_search_master` | Web前端 / 外部 | 查询书籍位置 |
| `/arm_command` | `arm_controller` | `book_search_master` | 机械臂控制 |
| `/set_mode` | `book_search_master` | Web前端 / 外部 | 切换模式 |

---

## 7. Action 定义汇总

| Action | 服务端 | 客户端 | 说明 |
|--------|--------|--------|------|
| `/pick_book` | `book_search_master` | Web前端 / 外部 | 取书任务 |
| `/navigate_to_pose` | Nav2内置 | `book_search_master` | 导航到目标位姿 |

---

## 8. 与 Nav2 / SLAM 集成使用的标准话题

这些话题由 Nav2 和 SLAM Toolbox 自动创建和管理，我们的节点只需正确发布和订阅：

| Topic | 发布者 | 说明 |
|-------|--------|------|
| `/map` | `slam_toolbox` | 地图数据，Nav2 使用 |
| `/tf` (map→odom) | `slam_toolbox` | 地图到里程计的变换 |
| `/tf` (odom→base_link) | `stm32_bridge` | 里程计到机器人的变换 |
| `/tf` (base_link→laser) | `robot_state_publisher` | URDF 静态TF |
| `/tf` (base_link→camera) | `robot_state_publisher` | URDF 静态TF |
| `/global_costmap/costmap` | Nav2 | 全局代价地图 |
| `/local_costmap/costmap` | Nav2 | 局部代价地图 |
| `/plan` | Nav2 Planner | 全局路径 |
| `/local_plan` | Nav2 Controller | 局部路径 |
