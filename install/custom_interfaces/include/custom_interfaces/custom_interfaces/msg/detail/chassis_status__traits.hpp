// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/ChassisStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/chassis_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChassisStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_enabled
  {
    out << "motor_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_enabled, out);
    out << ", ";
  }

  // member: emergency_stop
  {
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << ", ";
  }

  // member: collision_front
  {
    out << "collision_front: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_front, out);
    out << ", ";
  }

  // member: collision_rear
  {
    out << "collision_rear: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_rear, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: cmd_latency_ms
  {
    out << "cmd_latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_latency_ms, out);
    out << ", ";
  }

  // member: lost_frames
  {
    out << "lost_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.lost_frames, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChassisStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: motor_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_enabled, out);
    out << "\n";
  }

  // member: emergency_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << "\n";
  }

  // member: collision_front
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_front: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_front, out);
    out << "\n";
  }

  // member: collision_rear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_rear: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_rear, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: cmd_latency_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_latency_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_latency_ms, out);
    out << "\n";
  }

  // member: lost_frames
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lost_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.lost_frames, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChassisStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::msg::ChassisStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::ChassisStatus & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::ChassisStatus>()
{
  return "custom_interfaces::msg::ChassisStatus";
}

template<>
inline const char * name<custom_interfaces::msg::ChassisStatus>()
{
  return "custom_interfaces/msg/ChassisStatus";
}

template<>
struct has_fixed_size<custom_interfaces::msg::ChassisStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<custom_interfaces::msg::ChassisStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<custom_interfaces::msg::ChassisStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__TRAITS_HPP_
