# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **Note**: This directory contains the design documentation for the Smart Book-Finding Robot project. See the root-level [CLAUDE.md](../CLAUDE.md) for the full repository guide including build commands, implemented firmware code, and overall architecture. This file covers only details specific to the design docs.

## Design Document Index

| # | Document | Contents |
|---|----------|----------|
| 01 | `01_project_overview.md` | System architecture, hardware BOM, ROS2 node graph, core data flow, key metrics, safety requirements |
| 02 | `02_hardware_architecture.md` | Wiring topology, STM32 pin mappings (TB6612 motors/encoders/IMU), power tree, differential drive kinematics, OpenMV mounting |
| 03 | `03_communication_protocols.md` | STM32 binary frame protocol (CRC16-CCITT), OpenMV text-line protocol, robotic arm serial protocol (`$KMS`, `#xxxPxxx`, `$QSTAT`, etc.) |
| 04 | `04_ros2_interfaces.md` | All custom `.msg`/`.srv`/`.action` definitions, topic/service/action tables, QoS policies |
| 05 | `05_software_modules.md` | Per-module class designs, state machines, pick sequence, visual servoing P-controller, config YAMLs |
| 06 | `06_tf_coordinate_tree.md` | Complete TF tree (`map→odom→base_footprint→base_link→sensors`), URDF snippets, calibration procedures |
| 07 | `07_execution_plan.md` | 7-phase implementation plan (~26 days), milestones, test plans, risk list, operation commands |
| 08 | `08_rdk_x5_gpio_peripherals.md` | RDK X5 GPIO/UART/PWM/I2C/SPI programming guide based on official samples — Hobot.GPIO API, pin table, patterns for emergency stop, collision sensor, serial communication |

## Implementation Status

- [x] STM32 firmware — motor control (PWM + TB6612), encoder reading (TIM1/TIM4), USART2 serial output
- [x] Robotic arm firmware — IK solver, servo control, serial protocol (see `6zrobotic_arm/source.c`)
- [x] LiDAR SDK and ROS2 driver available (see `Lidar/`)
- [x] **ROS2 custom_interfaces** — ChassisStatus.msg, RobotStatus.msg (see `RDKX5/src/custom_interfaces/`)
- [x] **ROS2 stm32_bridge** — serial_protocol.py, odometry.py, bridge_node.py (see `RDKX5/src/stm32_bridge/`)
- [ ] STM32 serial framed protocol (CRC, odometry packets) — STM32 固件侧尚未实现二进制帧协议, 当前为 printf 文本输出
- [ ] OpenMV firmware (`book_detector.py`) — not yet implemented
- [ ] ROS2 arm_controller, openmv_bridge, book_search nodes — not yet implemented
- [ ] System integration — not yet started

## How to Use These Docs

These documents are **normative** for implementation — the planned ROS2 nodes, message definitions, state machines, and communication protocols should be implemented as specified here. When implementing a module, read its section in `05_software_modules.md` together with the relevant protocol doc (03) and interface definitions (04).
