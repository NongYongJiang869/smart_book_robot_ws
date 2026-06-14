// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/ChassisStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/chassis_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_ChassisStatus_lost_frames
{
public:
  explicit Init_ChassisStatus_lost_frames(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::ChassisStatus lost_frames(::custom_interfaces::msg::ChassisStatus::_lost_frames_type arg)
  {
    msg_.lost_frames = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_cmd_latency_ms
{
public:
  explicit Init_ChassisStatus_cmd_latency_ms(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_lost_frames cmd_latency_ms(::custom_interfaces::msg::ChassisStatus::_cmd_latency_ms_type arg)
  {
    msg_.cmd_latency_ms = std::move(arg);
    return Init_ChassisStatus_lost_frames(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_error_code
{
public:
  explicit Init_ChassisStatus_error_code(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_cmd_latency_ms error_code(::custom_interfaces::msg::ChassisStatus::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ChassisStatus_cmd_latency_ms(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_collision_rear
{
public:
  explicit Init_ChassisStatus_collision_rear(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_error_code collision_rear(::custom_interfaces::msg::ChassisStatus::_collision_rear_type arg)
  {
    msg_.collision_rear = std::move(arg);
    return Init_ChassisStatus_error_code(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_collision_front
{
public:
  explicit Init_ChassisStatus_collision_front(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_collision_rear collision_front(::custom_interfaces::msg::ChassisStatus::_collision_front_type arg)
  {
    msg_.collision_front = std::move(arg);
    return Init_ChassisStatus_collision_rear(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_emergency_stop
{
public:
  explicit Init_ChassisStatus_emergency_stop(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_collision_front emergency_stop(::custom_interfaces::msg::ChassisStatus::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_ChassisStatus_collision_front(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_motor_enabled
{
public:
  explicit Init_ChassisStatus_motor_enabled(::custom_interfaces::msg::ChassisStatus & msg)
  : msg_(msg)
  {}
  Init_ChassisStatus_emergency_stop motor_enabled(::custom_interfaces::msg::ChassisStatus::_motor_enabled_type arg)
  {
    msg_.motor_enabled = std::move(arg);
    return Init_ChassisStatus_emergency_stop(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

class Init_ChassisStatus_header
{
public:
  Init_ChassisStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisStatus_motor_enabled header(::custom_interfaces::msg::ChassisStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChassisStatus_motor_enabled(msg_);
  }

private:
  ::custom_interfaces::msg::ChassisStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::ChassisStatus>()
{
  return custom_interfaces::msg::builder::Init_ChassisStatus_header();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__BUILDER_HPP_
