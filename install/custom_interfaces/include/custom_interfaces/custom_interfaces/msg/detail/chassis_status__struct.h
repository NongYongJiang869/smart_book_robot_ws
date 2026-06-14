// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/ChassisStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ERR_NONE'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_NONE = 0
};

/// Constant 'ERR_ESTOP'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_ESTOP = 1
};

/// Constant 'ERR_FRONT_COLLISION'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_FRONT_COLLISION = 2
};

/// Constant 'ERR_REAR_COLLISION'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_REAR_COLLISION = 4
};

/// Constant 'ERR_COMM_TIMEOUT'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_COMM_TIMEOUT = 256
};

/// Constant 'ERR_IMU_FAULT'.
enum
{
  custom_interfaces__msg__ChassisStatus__ERR_IMU_FAULT = 1024
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ChassisStatus in the package custom_interfaces.
/**
  * 底盘底层状态 (stm32_bridge 发布, 用于诊断和监控)
 */
typedef struct custom_interfaces__msg__ChassisStatus
{
  std_msgs__msg__Header header;
  /// 4个电机使能状态 (位0~3, 1=使能)
  uint8_t motor_enabled;
  /// 传感器状态
  /// 急停是否触发
  bool emergency_stop;
  /// 前碰撞传感器
  bool collision_front;
  /// 后碰撞传感器
  bool collision_rear;
  /// 错误码 (0=正常)
  uint16_t error_code;
  /// 通信质量
  /// 最近一次速度指令的往返延迟
  float cmd_latency_ms;
  /// 近1秒内丢帧数量
  uint16_t lost_frames;
} custom_interfaces__msg__ChassisStatus;

// Struct for a sequence of custom_interfaces__msg__ChassisStatus.
typedef struct custom_interfaces__msg__ChassisStatus__Sequence
{
  custom_interfaces__msg__ChassisStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__ChassisStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_H_
