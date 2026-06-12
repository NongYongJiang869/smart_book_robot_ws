# 智能寻书机器人 — TF 坐标系树设计

## 1. 完整 TF 树

```
map                           ← SLAM 全局地图原点（启动建图时的位姿）
  │
  │  slam_toolbox 发布
  │  (持续修正，漂移约 0.01~0.05m 每100m)
  │
  ▼
odom                          ← 里程计坐标系（编码器推算，有漂移）
  │
  │  stm32_bridge 发布
  │  (50Hz，基于编码器+IMU融合)
  │
  ▼
base_footprint                ← 机器人在地面的投影（底面中心）
  │
  │  robot_state_publisher 发布 (URDF)
  │  变换: z = wheel_radius (约 0.076m)
  │
  ▼
base_link                     ← 机器人本体质心（底盘上表面中心）
  │
  ├──→ laser                  ← 激光雷达
  │    static transform, URDF 定义
  │    x=0.12, y=0, z=0.15
  │
  ├──→ camera_link            ← OpenMV 相机
  │    static transform, URDF 定义
  │    x=0.08, y=0, z=0.35, pitch=0.35 (20°上仰)
  │    │
  │    └──→ camera_optical    ← 相机光心（OpenMV 标准）
  │         static transform (ROS REP-103)
  │         x=0, y=0, z=0, yaw=-π/2, pitch=0, roll=-π/2
  │
  ├──→ arm_base               ← 机械臂底座
  │    static transform, URDF 定义
  │    x=0, y=0, z=0.25
  │    │
  │    ├──→ arm_link1         ← 臂关节1
  │    │    dynamic transform, arm_controller 发布
  │    │
  │    ├──→ ...               ← 臂连杆2~5
  │    │
  │    └──→ arm_ee            ← 末端执行器（夹爪中心）
  │         dynamic transform, arm_controller 发布
  │
  └──→ imu_link               ← IMU 传感器
       static transform, URDF 定义
       x=0, y=0, z=0.05 (IMU 通常安装在 STM32 板上)
```

## 2. 各坐标系详细定义

### 2.1 map

| 属性 | 值 |
|------|-----|
| 父坐标系 | 无（顶层） |
| 发布者 | `slam_toolbox` / `nav2_amcl` |
| 建立方式 | 第一次启动 SLAM 时，以机器人当前位置为原点 |
| 坐标轴 | X: 前, Y: 左, Z: 上 (REP-103) |
| 稳定性 | 全局固定，不随机器人移动 |

### 2.2 odom

| 属性 | 值 |
|------|-----|
| 父坐标系 | map |
| 发布者 | `stm32_bridge`（里程计推算） / `slam_toolbox`（修正后的 map→odom 变换） |
| 频率 | 50Hz（原始里程计），`slam_toolbox` 在每次回环检测时修正 |
| 特点 | 随机器人移动而持续平滑变化，但长期有漂移。map→odom 的变换负责修正漂移 |

### 2.3 base_footprint

| 属性 | 值 |
|------|-----|
| 父坐标系 | odom |
| 含义 | 机器人在地面上的投影点，位于底盘几何中心的正下方 |
| 发布者 | `robot_state_publisher`（根据 URDF 中的 `base_link → base_footprint` 静态变换） |
| 与 base_link 关系 | `base_footprint` 的 (x, y) = `base_link` 的 (x, y)，`base_footprint` 的 z = 0 |

### 2.4 base_link

| 属性 | 值 |
|------|-----|
| 父坐标系 | base_footprint |
| 含义 | 机器人本体质心，一般为底盘上表面中心 |
| 发布者 | `stm32_bridge`（发布 `odom→base_link` 的 TF） |
| 高度 | = 车轮半径 + 底盘板厚 ≈ 0.076m |

### 2.5 laser

| 属性 | 值 |
|------|-----|
| 父坐标系 | base_link |
| 变换类型 | static_transform（在 URDF 中定义） |
| 位置 | 激光雷达在底座上的安装位置 |
| 方向 | 通常激光雷达正面朝机器人前方，即 yaw=0 |

### 2.6 camera_link / camera_optical

| 属性 | 值 |
|------|-----|
| 父坐标系 | base_link |
| camera_link 位置 | OpenMV 在底座上的安装位置 |
| camera_optical | REP-103 光学坐标系：Z前，X右，Y下（与 OpenCV 一致） |

### 2.7 arm_base / arm_ee

| 属性 | 值 |
|------|-----|
| arm_base 父坐标系 | base_link |
| arm_ee 父坐标系 | arm_link6 |
| arm_base 位置 | 机械臂底座在底盘上的安装位置 |
| arm_ee 含义 | 末端执行器（夹爪）中心，z 轴指向夹爪开合方向 |

## 3. URDF 建模关键片段

```xml
<!-- 关键 joint 定义示例 -->
<!-- 1. base_footprint → base_link -->
<joint name="base_footprint_joint" type="fixed">
  <parent link="base_footprint"/>
  <child  link="base_link"/>
  <origin xyz="0 0 0.076" rpy="0 0 0"/>
</joint>

<!-- 2. base_link → laser -->
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="laser"/>
  <origin xyz="0.12 0 0.15" rpy="0 0 0"/>
</joint>

<!-- 3. base_link → camera_link -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="camera_link"/>
  <origin xyz="0.08 0 0.35" rpy="0 0.35 0"/>
</joint>

<!-- 4. camera_link → camera_optical (REP-103) -->
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child  link="camera_optical"/>
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>

<!-- 5. base_link → arm_base -->
<joint name="arm_base_joint" type="fixed">
  <parent link="base_link"/>
  <child  link="arm_base"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
</joint>
```

## 4. 传感器到机器人中心的标定表

| 传感器 | 相对 base_link 的位置 (x, y, z) (m) | 相对 base_link 的姿态 (roll, pitch, yaw) (rad) | 标定方法 |
|--------|-------------------------------------|-----------------------------------------------|----------|
| LiDAR | (+0.12, 0, +0.15) | (0, 0, 0) | 直尺测量 + 手动微调 |
| OpenMV | (+0.08, 0, +0.35) | (0, ~0.35, 0) | 直尺测量 + 棋盘格标定查表 |
| IMU | (0, 0, +0.05) | (0, 0, 0) | 查阅 STM32 板 IMU 布局 |
| Arm Base | (0, 0, +0.25) | (0, 0, 0) | 直尺测量 |

## 5. 标定步骤

### 5.1 LiDAR 标定

```bash
# 1. 手动测量 laser 相对 base_link 的 x,y,z
# 2. 将估算值写入 URDF
# 3. 运行 SLAM，推着车走一个矩形回路
# 4. 观察回环是否闭合，若不闭合则微调 yaw 值
# 5. 重复直到回环误差 <0.05m
```

### 5.2 OpenMV 相机标定

```python
# 在 OpenMV 端执行
# 1. 用已知尺寸的棋盘格/标签
# 2. 置于固定距离（30cm），读取像素坐标
# 3. 计算出 camera→base_link 的平移关系（或直接在采集数据时做手眼标定）
```

### 5.3 里程计标定

```
# 1. 机器人直线前进 2 米，读取 /odom 输出的距离
# 2. 计算轮径修正系数：correction = real_distance / odom_distance
# 3. 机器人原地旋转 10 圈（3600°），读取 odom 的角度
# 4. 计算轮距修正系数
# 5. 写入 stm32_bridge 的配置参数
```

## 6. TF 发布者责任矩阵

| TF 变换 | 发布者 | 发布方式 | 频率 |
|---------|--------|----------|------|
| map → odom | `slam_toolbox` | 动态（回环检测时跳变） | 10Hz |
| odom → base_link | `stm32_bridge` | 持续动态 | 50Hz |
| base_footprint → base_link | `robot_state_publisher` | 静态（URL读取） | 一次性+latched |
| base_link → laser | `robot_state_publisher` | 静态 | 一次性+latched |
| base_link → camera_link | `robot_state_publisher` | 静态 | 一次性+latched |
| camera_link → camera_optical | `robot_state_publisher` | 静态 | 一次性+latched |
| base_link → arm_base | `robot_state_publisher` | 静态 | 一次性+latched |
| arm_base → arm_link1 | `arm_controller` | 动态 | 20Hz |
| arm_link1 → ... → arm_ee | `arm_controller` | 动态 | 20Hz |

## 7. 使用 TF 的典型场景

```python
# 场景1: 将激光点云从 laser 坐标系变换到 map 坐标系
# SLAM/Nav2 自动完成，通过监听 tf buffer

# 场景2: 将书籍检测位置从 camera_optical 变换到 map
# openmv_bridge 中：
import tf2_ros
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

# 等待变换可用
transform = buffer.lookup_transform(
    'map', 'camera_optical', rclpy.time.Time()
)

# 场景3: 发送导航目标，需要目标在 map 坐标系下
# book_search_master 中：
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = target_x  # 书架在地图中的坐标
goal_pose.pose.position.y = target_y
```
