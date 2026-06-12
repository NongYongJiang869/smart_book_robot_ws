# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Purpose

This is the **design documentation** repository for a smart book-finding robot — an autonomous ROS2-based robot that navigates a library, identifies books via QR codes on book spines, and retrieves them with a 6-axis robotic arm. The actual ROS2 workspace (`smart_book_robot_ws/`) has not been created yet; these docs define the full architecture before implementation begins.

## Architecture (Big Picture)

**Hardware stack**: RDK X5 (ROS2 Humble) → STM32 (chassis MCU) via UART serial → 4-wheel differential-drive chassis + motors/encoders + IMU + collision sensors + emergency stop. RDK X5 also connects via USB to a YDLidar 2D LiDAR, an OpenMV Cam H7 Plus (book QR detection), and a 6-axis robotic arm controller (UART).

**Software stack (to be built in `smart_book_robot_ws/src/`)**:
- `stm32_bridge/` — serial framing protocol, odometry computation, publishes `/odom`, `/imu`, subscribes `/cmd_vel`
- `openmv_bridge/` — text-line protocol over USB serial, publishes `/book_detection`, estimates distance from QR pixel width
- `arm_controller/` — adapter pattern over the arm's proprietary serial protocol, exposes an `ArmDriver` abstract interface, runs a pick sequence state machine
- `book_search/` — master state machine (IDLE → QUERYING → NAVIGATING → SCANNING → APPROACHING → PICKING → RETURNING → COMPLETED), maintains a book→shelf location YAML database
- `robot_bringup/` — launch files (mapping, navigation, book_search modes), URDF model, Nav2 + SLAM Toolbox config YAMLs
- `custom_interfaces/` — ROS2 `.msg`/`.srv`/`.action` definitions (BookDetection, PickBook, QueryBook, etc.)

**TF tree**: `map` → (slam_toolbox) → `odom` → (stm32_bridge) → `base_footprint` → `base_link` → `laser`, `camera_link→camera_optical`, `arm_base→…→arm_ee`

**Key data flow**: User query → book_search queries book_database.yaml → sends Nav2 goal → robot navigates to shelf → OpenMV scans for QR code → visual servoing aligns to book → arm executes pick sequence → returns to home

## Communication Protocols

1. **STM32 ↔ RDK X5**: Binary framed protocol at 115200 baud. Frame: `0x5A 0xA5` header + length + type + seq + payload + CRC16-CCITT. Upstream types: ODOM_DATA (50Hz), STATUS (10Hz), ERROR, ACK, HEARTBEAT. Downstream: VEL_CMD (100Hz), LED_CTRL, BUZZER, RESET_ODOM, MOTOR_BRAKE, HEARTBEAT. All multi-byte fields are little-endian.

2. **OpenMV ↔ RDK X5**: Text-line protocol over USB CDC at 115200. Commands: `CMD:<name>;<key>=<value>;...\n`. Results: `DETECT:<qr_content>;x=...;y=...;w=...;h=...;confidence=...\n`. Distance estimated via pinhole model from QR pixel width (known physical size 20mm, focal length ~450px calibrated).

3. **Arm controller**: Uses the arm's own proprietary serial protocol. An `ArmDriver` abstract base class defines the adapter interface (`set_joint_angles`, `get_joint_angles`, `set_gripper`, `get_status`, `emergency_stop`). The specific arm model is not yet selected.

## ROS2 Interface Summary

**Custom QoS policies**: Sensor Data (best_effort, volatile, keep_last=5), Status (reliable, transient_local, keep_last=1), System Default (reliable, volatile, keep_last=10).

**Key topics**: `/scan` (LaserScan), `/odom` (Odometry), `/book_detection` (BookDetection), `/cmd_vel` (Twist), `/joint_states` (JointState), `/robot_status` (RobotStatus), `/tf` + `/tf_static`

**Key services**: `/query_book` (QueryBook.srv), `/arm_command` (ArmCommand.srv), `/set_mode` (SetMode.srv)

**Key actions**: `/pick_book` (PickBook.action — long-running task with phase feedback), `/navigate_to_pose` (Nav2 built-in)

## Document Index

| Document | Content |
|----------|---------|
| `01_project_overview.md` | System architecture, hardware BOM, ROS2 node graph, data flow, key metrics, safety requirements |
| `02_hardware_architecture.md` | Wiring topology, pin mappings (STM32↔motors/encoders/IMU), power tree, kinematics, OpenMV mounting |
| `03_communication_protocols.md` | Full binary frame format for STM32, text protocol for OpenMV, arm adapter interface |
| `04_ros2_interfaces.md` | All custom .msg/.srv/.action definitions, topic/service/action tables, QoS policies |
| `05_software_modules.md` | Per-module class designs, state machines, pick sequence, visual servoing algorithm, config YAMLs |
| `06_tf_coordinate_tree.md` | Complete TF tree, coordinate frame definitions, URDF snippets, calibration procedures |
| `07_execution_plan.md` | 7-phase implementation plan (~26 days), milestones, test plans, risk list, operation commands |

## Hardware Constraints

- **Differential drive** (skid-steering), not mecanum — kinematics: `v = (vL+vR)/2, ω = (vR-vL)/L`
- **QR codes** on book spines (20mm, Version 1-3, error correction M), content format `book_<ID>`
- **Speed limits**: 0.5 m/s linear, 1.0 rad/s angular
- **Navigation accuracy target**: ±5cm (Nav2), ±1cm (visual servoing fine alignment)
- **Motor timeout**: STM32 auto-brakes if no VEL_CMD received for 200ms
- **OpenMV QVGA** (320×240), grayscale mode for QR detection

## Key Architectural Decisions

- The arm driver uses an **abstract base class + adapter pattern** so the pick sequence state machine never depends on a specific arm protocol
- QR codes (not AprilTags) are used for book identification because they encode arbitrary strings; the trade-off is no native 6DOF pose — distance is estimated from pixel width ratio
- `stm32_bridge` publishes `odom→base_link` TF at 50Hz; `slam_toolbox` publishes `map→odom` TF with loop-closure corrections
- Visual servoing for fine alignment uses a simple P-controller on angle_deg and distance_m from the `/book_detection` topic
- The book location database is a static YAML file (not a live DB) — books map to shelf/row/col, and the robot computes the world coordinate from shelf pose + column offset
