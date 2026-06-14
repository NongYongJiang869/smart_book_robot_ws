// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_error_message
{
public:
  explicit Init_RobotStatus_error_message(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RobotStatus error_message(::custom_interfaces::msg::RobotStatus::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_current_task
{
public:
  explicit Init_RobotStatus_current_task(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_error_message current_task(::custom_interfaces::msg::RobotStatus::_current_task_type arg)
  {
    msg_.current_task = std::move(arg);
    return Init_RobotStatus_error_message(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_angular_velocity
{
public:
  explicit Init_RobotStatus_angular_velocity(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_current_task angular_velocity(::custom_interfaces::msg::RobotStatus::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_RobotStatus_current_task(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_linear_velocity
{
public:
  explicit Init_RobotStatus_linear_velocity(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_angular_velocity linear_velocity(::custom_interfaces::msg::RobotStatus::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_RobotStatus_angular_velocity(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_state
{
public:
  explicit Init_RobotStatus_state(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_linear_velocity state(::custom_interfaces::msg::RobotStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_RobotStatus_linear_velocity(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_header
{
public:
  Init_RobotStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_state header(::custom_interfaces::msg::RobotStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotStatus_state(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RobotStatus>()
{
  return custom_interfaces::msg::builder::Init_RobotStatus_header();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
