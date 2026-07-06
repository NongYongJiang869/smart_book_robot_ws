# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

Smart Book-Finding Robot (智能寻书机器人) — an autonomous ROS2-based robot that navigates a library, identifies books via QR codes on book spines, and retrieves them with a 6-axis robotic arm.

**Hardware stack**: RDK X5 (ROS2 Humble main controller) → STM32F103C8T6 (chassis MCU, UART serial) → 4-wheel differential-drive chassis. RDK X5 also connects via USB to YDLidar 2D LiDAR, OpenMV Cam H7 Plus (QR detection), and a 6-axis Arduino-based robotic arm (UART serial, proprietary text protocol).

**Current project phase**: Design documents are complete. STM32 firmware and robotic arm firmware are implemented at a basic level. The ROS2 workspace (`smart_book_robot_ws/`) nodes are designed but not yet implemented — they exist only in the design docs.

## Repository Structure

```
smart_book_robot_ws/
├── RDKX5/                     # ROS2 Humble workspace — RDK X5 侧所有节点
│   └── src/
│       ├── custom_interfaces/  # 自定义消息 (ChassisStatus, RobotStatus)
│       └── stm32_bridge/       # STM32 串口桥接 (serial_protocol, odometry, bridge_node)
├── design_docs/               # 设计文档 — 权威架构参考
│   ├── 01_project_overview.md
│   ├── 02_hardware_architecture.md
│   ├── 03_communication_protocols.md
│   ├── 04_ros2_interfaces.md
│   ├── 05_software_modules.md
│   ├── 06_tf_coordinate_tree.md
│   ├── 07_execution_plan.md
│   └── 08_rdk_x5_gpio_peripherals.md
├── chassis/                   # STM32F103C8T6 firmware — chassis motor control
│   ├── inc/                  # Headers (motor.h, encoder.h, bsp_usart.h, etc.)
│   ├── src/                  # Sources (main.c, motor.c, encoder.c, bsp_usart.c, etc.)
│   ├── startup/              # Startup assembly (startup_stm32f10x_md.s)
│   ├── ld/                   # Linker script (stm32f103c8t6.ld)
│   ├── stdlib/               # CMSIS + STM32F10x StdPeriph Driver
│   ├── tools/openocd/        # OpenOCD config for flashing/debugging
│   ├── Makefile              # arm-none-eabi-gcc build
│   ├── README.md             # Pin assignments, timer allocation, serial output format
│   └── .clangd               # clangd config for IDE support
├── tools/
│   └── chassis_serial_monitor.py  # RDK X5 串口监控程序 (接收 STM32 编码器数据)
├── 6zrobotic_arm/
│   └── source.c              # Arduino firmware — 6-axis arm with IK, servo control, serial protocol
└── Lidar/
    ├── YDLidar-SDK-master/   # YDLidar C++ SDK
    └── yahboomcar_ws/        # ROS2 workspace with ydlidar_ros2_driver
```

## Build Commands

### STM32 Firmware (`chassis/`)

```bash
cd chassis/ros_car_stm32
make                          # Build → build/app.elf, build/app.hex, build/app.bin
make flash                    # Flash via OpenOCD
make erase                    # Mass erase chip via OpenOCD
make clean                    # Remove build/ directory
make size                     # Print ELF section sizes
```

Toolchain: `arm-none-eabi-gcc` targeting Cortex-M3 (`-mcpu=cortex-m3 -mthumb`). Uses STM32F10x Standard Peripheral Library (not HAL). The Makefile links against `stdlib/STM32F10x_StdPeriph_Driver` and `stdlib/CMSIS`.

### Robotic Arm (`6zrobotic_arm/source.c`)

This is Arduino firmware. Build and upload via Arduino IDE (board: Arduino Mega or compatible). No CLI build command — edit in Arduino IDE.

### ROS2 (`RDKX5/`)

```bash
cd RDKX5
source /opt/ros/humble/setup.bash
colcon build                                          # 全量编译
colcon build --packages-select stm32_bridge           # 只编译单个包
rm -rf install/<pkg> build/<pkg> && colcon build      # 重新安装 (setuptools 兼容)
```

> **注意**: 当前 setuptools 版本不支持 `--uninstall` 和 `--editable`，重新编译前需手动 `rm -rf install/<pkg> build/<pkg>`。不要用 `--symlink-install`。

**运行**:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动 STM32 桥接
ros2 launch stm32_bridge stm32_bridge.launch.py

# 启动底盘 + 键盘遥控
ros2 launch stm32_bridge chassis_bringup.launch.py teleop:=true

# 查看话题
ros2 topic echo /odom
ros2 topic echo /chassis_status
```

The ROS2 workspace path defined in docs is `smart_book_robot_ws/` (sibling to this repo's `Lidar/` directory). Build command (once implemented):
```bash
cd smart_book_robot_ws
colcon build --symlink-install
```

## Architecture — Big Picture

### Hardware Connections
- **RDK X5 ↔ STM32**: UART serial, 115200 baud, 3.3V TTL, TX/RX/GND (PA9/PA10 on STM32 side). Binary framed protocol with CRC16-CCITT.
- **RDK X5 ↔ OpenMV**: USB virtual serial (CDC), 115200 baud, text-line protocol (`CMD:...\n` / `DETECT:...\n`).
- **RDK X5 ↔ YDLidar**: USB, via `ydlidar_ros2_driver` node.
- **RDK X5 ↔ Robotic Arm**: UART serial, 115200 baud, text-delimiter protocol (`$KMS:...!`, `#xxxPxxxTxxx!`, `$DST!`, etc.).
- **STM32 ↔ Motors**: TB6612 dual H-bridge driver. Left motors (A channel): PB3/AIN1, PA4/AIN2, PA0/PWMA (TIM2_CH1). Right motors (B channel): PA5/BIN1, PA6/BIN2, PA1/PWMB (TIM2_CH2). STBY: PA12. PWM frequency: 1kHz, duty cycle 0–999.

### Chassis Kinematics
Differential drive (skid-steering, 4 wheels, same-side motors in parallel):
```
v = (v_left + v_right) / 2
ω = (v_right - v_left) / L     (L = wheel base, needs calibration)
```

### Communication Protocols

**STM32 binary framed protocol**: `0x5A 0xA5` header + length + type + seq + payload + CRC16-CCITT (polynomial 0x1021). All multi-byte fields little-endian. Upstream types: ODOM_DATA (0x01, 50Hz), STATUS (0x02, 10Hz), ERROR (0x04), ACK (0x05), HEARTBEAT (0x06). Downstream types: VEL_CMD (0x81, 100Hz), LED_CTRL (0x82), BUZZER (0x83), RESET_ODOM (0x84), MOTOR_BRAKE (0x85), HEARTBEAT (0x86). **Important**: 200ms VEL_CMD timeout on STM32 side triggers auto-brake.

**Robotic arm serial protocol** (implemented in `source.c`):
- `$KMS:x,y,z,time!` — Inverse kinematics move to Cartesian (x,y,z in mm). Returns `@KMS_OK,<alpha>!` on success or `@KMS_ERR!` on failure.
- `#xxxPyyyyTzzzz!` — Single servo control (xxx=servo ID 0-5, yyyy=PWM 500-2500, zzzz=time ms). `#255PyyyyTzzzz!` controls all 6 servos at once.
- `$DST!` / `$DST:N!` — Stop all / stop servo N.
- `$RST!` — Soft reset.
- `$QSTAT!` — Query status, returns `@STATUS:<IDLE|BUSY|ERROR>,x=...,y=...,z=...,alpha=...!`
- `$QPWM!` — Query all servo PWM values, returns `@PWM:v0,v1,v2,v3,v4,v5!`
- `$DGT:start-end,times!` — Execute pre-programmed action group from flash.
- Arm kinematics params: L0=100, L1=105, L2=75, L3=180 (mm). 6 servos on pins {7, 3, 5, 6, 9, 8}. Servos 3 and 4 are inverted (3000-PWM). IK alpha search range: 0° to -135°.

**OpenMV text protocol**:
- Commands from RDK X5: `CMD:<name>;<key>=<value>;...\n`
- Detection results from OpenMV: `DETECT:<qr_content>;x=<px>;y=<px>;w=<px>;h=<px>;confidence=<0-1>\n`
- QR code physical size: 20mm, content format: `book_<ID>`, printed white-on-black with 4-module quiet zone.
- Distance estimation: `distance = (real_size_mm * focal_length_px) / pixel_width`, focal length ~450px for OpenMV H7 Plus QVGA.

### ROS2 Software Architecture (from design docs)

**TF Tree**: `map` → (slam_toolbox) → `odom` → (stm32_bridge) → `base_footprint` → `base_link` → `laser`, `camera_link→camera_optical`, `arm_base→…→arm_ee`

**Planned ROS2 nodes** (in `smart_book_robot_ws/src/`):
- `stm32_bridge/` — Serial framing + odometry. Publishes `/odom`, `/imu`, `/tf`(odom→base_link). Subscribes `/cmd_vel`.
- `openmv_bridge/` — Text protocol over USB. Publishes `/book_detection` (custom BookDetection msg). Estimates distance/angle from QR pixel size.
- `arm_controller/` — Adapter over arm's serial protocol. Provides `/arm_command` service. Runs pick sequence state machine. Publishes `/joint_states`.
- `book_search/` — Master state machine (IDLE → QUERYING → NAVIGATING → SCANNING → APPROACHING → PICKING → RETURNING → COMPLETED). Provides `/query_book` service and `/pick_book` action. Maintains book→shelf YAML database.
- `custom_interfaces/` — ROS2 custom `.msg`, `.srv`, `.action` definitions.
- `robot_bringup/` — Launch files (mapping, navigation, book_search modes), URDF, Nav2 + SLAM Toolbox config.

**Key ROS2 topics**: `/scan` (LaserScan), `/odom` (Odometry), `/book_detection` (BookDetection), `/cmd_vel` (Twist), `/joint_states` (JointState), `/robot_status` (RobotStatus)

**Custom QoS policies**: Sensor Data (best_effort, volatile, keep_last=5), Status (reliable, transient_local, keep_last=1), System Default (reliable, volatile, keep_last=10)

### Visual Servoing
Fine alignment uses P-controller on OpenMV detection results:
- Angular servo: `angular.z = kp_angle * detection.angle_deg`
- Distance servo: `linear.x = kp_dist * (detection.distance_m - target_dist)`
- Completion: `|angle| < 2° AND |distance - target| < 1cm`, target distance = 25cm from book

## Design Constraints
- Speed limits: 0.5 m/s linear, 1.0 rad/s angular
- Navigation accuracy: ±5cm (Nav2), ±1cm (visual servoing)
- Single retrieval time target: ≤3 minutes
- Success rate target: ≥95%
- QR codes: Version 1-3, error correction level M (15%), 20mm physical size
- Safety: physical emergency stop, LiDAR obstacle detection (<30cm → stop), collision sensors, 200ms communication timeout auto-brake

## STM32 Code Notes
- MCU: STM32F103C8T6 (medium density, 64KB flash)
- Uses StdPeriph Driver (not HAL), CMSIS Core + Device Support
- Serial: USART1 (PA9/TX, PA10/RX), 115200 8N1, no hardware flow control, `_write()` retargets printf to USART1
- PWM: TIM2, prescaler=71 (1MHz → 1μs tick), period=999 (1kHz), CH1=PA0/left, CH2=PA1/right
- Current `main.c` is a demo: cycles through forward/backward/left-turn/right-turn/stop in a loop. The serial protocol frame handling (from design doc 03) is NOT yet implemented in the firmware — `bsp_usart.c` only does basic TX via `_write()`.
- clangd config (`.clangd`) strips GCC-specific flags and sets `--target=arm-none-eabi` for IDE support.

## Robotic Arm Code Notes
- Platform: Arduino (likely Mega 2560), uses `<Servo.h>` and `<winbondflash.h>` (W25Q64 flash for action group storage)
- Serial: `Serial` (hardware UART), 115200 baud
- `SerialEvent()`-driven command parsing: character `$` → command mode, `#` → servo control mode, `{` → action group execution, `<` → action group download
- IK solver (`kinematics_analysis`) does geometric inverse kinematics: given (x,y,z,Alpha) → computes joint angles θ3-θ6 → converts to PWM via linear mapping.
- The `kinematics_move()` function brute-force searches Alpha from 0° down to -135° to find a valid solution, then executes via `set_servo()`.

## Keepout Zones（禁区蒙版）

Robot can be restricted from certain areas WITHOUT modifying the SLAM map (so AMCL localization is unaffected).

**Architecture**: Keepout mask PGM → `keepout_mask_publisher` → `/keepout_mask` + `/costmap_filter_info` → `KeepoutFilter` in global costmap → path planning avoids zones.

**Files**:
- `/root/library_keepout_mask.pgm` — keepout mask (800×800, same dimensions as map)
- `/root/library_keepout_mask.yaml` — mask config (mode: scale, resolution: 0.05, origin: [-20, -20])
- `RDKX5/src/stm32_bridge/stm32_bridge/keepout_mask_publisher.py` — publishes mask + CostmapFilterInfo
- `RDKX5/src/stm32_bridge/config/nav2_params.yaml` — global costmap has `keepout_filter` plugin
- `tools/map_editor_gui.py` — GUI editor (draw = left-drag, clear = right-drag, save = Ctrl+S)
- `tools/edit_map.py` — CLI editor (`--add-keepout-rect`, `--clear-keepout-rect`, `--show-keepout`)

**Key details**:
- `keepout_mask_publisher` publishes only at startup and on file change (not at fixed rate, to avoid flooding KeepoutFilter with "new" messages)
- KeepoutFilter needs BOTH `CostmapFilterInfo` (on `/costmap_filter_info`) and mask `OccupancyGrid` (on `/keepout_mask`)
- The publisher uses `np.flipud` before flattening — y-axis must be flipped for OccupancyGrid alignment
- PGM pixel 0 (black) = keepout active, 254 (white) = free
- Do NOT use `costmap_filter_info_server` (C++ lifecycle node) — it fails to configure. Our Python node handles both publications.

## Navigation Goals

**Send navigation goals via BOTH `NavigateToPose` action AND `/goal_pose` topic:**
- `NavigateToPose` action → drives Nav2 path planning + controller (bt_navigator)
- `/goal_pose` topic → triggers `rotate_to_goal` node for final yaw alignment
- `rotate_to_goal` watches Nav2 reach xy position, then takes over `/cmd_vel` for pure rotation
- **MUST send BOTH**, otherwise robot reaches position but doesn't rotate to target yaw
```python
# NavigateToPose action (drives navigation)
ac = ActionClient(node, NavigateToPose, 'navigate_to_pose')
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
# ... set position and orientation ...
ac.send_goal_async(goal)

# /goal_pose topic (triggers rotate_to_goal for final rotation)
pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
pg = PoseStamped()
pg.header.frame_id = 'map'
pg.header.stamp = node.get_clock().now().to_msg()
pg.pose = goal.pose.pose
pub.publish(pg)
```

`rotate_to_goal` params: `xy_tolerance=0.25m`, `yaw_tolerance=0.087rad(5°)`, `stuck_timeout=5s`.

## Task Manager / Locations

**Location config**: `RDKX5/src/robot_task_manager/config/locations.json`
- `docking_stations` — robot charging/docking locations
- `bookshelves` — bookshelf positions
- `seats` — seat/table positions
- Fields: `x, y, z` (meters, z = height), `yaw` (degrees, 0=+x东, 90=+y北)

**Book mapping**: `RDKX5/src/robot_task_manager/config/books.json`

**Launch**:
```bash
ros2 launch robot_task_manager task_manager.launch.py use_rviz:=true robot_name:=robot-01
```

## RViz Config

Default RViz config at `RDKX5/src/stm32_bridge/config/map_view.rviz` includes:
- Map (/map), LaserScan (/scan), Odometry (/odom_fused), TF
- **Global Costmap** — `/global_costmap/costmap` (shows keepout zones in purple)
- **Local Costmap** — `/local_costmap/costmap`
- "2D Goal Pose" tool publishes to `/goal_pose`
