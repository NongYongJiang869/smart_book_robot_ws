// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
enum
{
  custom_interfaces__msg__RobotStatus__IDLE = 0
};

/// Constant 'NAVIGATING'.
enum
{
  custom_interfaces__msg__RobotStatus__NAVIGATING = 1
};

/// Constant 'SCANNING'.
enum
{
  custom_interfaces__msg__RobotStatus__SCANNING = 2
};

/// Constant 'APPROACHING'.
enum
{
  custom_interfaces__msg__RobotStatus__APPROACHING = 3
};

/// Constant 'PICKING'.
enum
{
  custom_interfaces__msg__RobotStatus__PICKING = 4
};

/// Constant 'RETURNING'.
enum
{
  custom_interfaces__msg__RobotStatus__RETURNING = 5
};

/// Constant 'ERROR'.
enum
{
  custom_interfaces__msg__RobotStatus__ERROR = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'current_task'
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotStatus in the package custom_interfaces.
/**
  * 机器人整体状态 (book_search_master 发布)
 */
typedef struct custom_interfaces__msg__RobotStatus
{
  std_msgs__msg__Header header;
  /// 当前状态
  uint8_t state;
  /// 底盘
  /// 当前线速度 (m/s)
  float linear_velocity;
  /// 当前角速度 (rad/s)
  float angular_velocity;
  /// 当前任务
  /// 正在找的书名, 空闲时为空
  rosidl_runtime_c__String current_task;
  /// 错误信息
  /// 错误描述, 无错误时为空
  rosidl_runtime_c__String error_message;
} custom_interfaces__msg__RobotStatus;

// Struct for a sequence of custom_interfaces__msg__RobotStatus.
typedef struct custom_interfaces__msg__RobotStatus__Sequence
{
  custom_interfaces__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
