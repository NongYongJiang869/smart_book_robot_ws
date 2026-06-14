// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/ChassisStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__ChassisStatus __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__ChassisStatus __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChassisStatus_
{
  using Type = ChassisStatus_<ContainerAllocator>;

  explicit ChassisStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_enabled = 0;
      this->emergency_stop = false;
      this->collision_front = false;
      this->collision_rear = false;
      this->error_code = 0;
      this->cmd_latency_ms = 0.0f;
      this->lost_frames = 0;
    }
  }

  explicit ChassisStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_enabled = 0;
      this->emergency_stop = false;
      this->collision_front = false;
      this->collision_rear = false;
      this->error_code = 0;
      this->cmd_latency_ms = 0.0f;
      this->lost_frames = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_enabled_type =
    uint8_t;
  _motor_enabled_type motor_enabled;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;
  using _collision_front_type =
    bool;
  _collision_front_type collision_front;
  using _collision_rear_type =
    bool;
  _collision_rear_type collision_rear;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;
  using _cmd_latency_ms_type =
    float;
  _cmd_latency_ms_type cmd_latency_ms;
  using _lost_frames_type =
    uint16_t;
  _lost_frames_type lost_frames;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_enabled(
    const uint8_t & _arg)
  {
    this->motor_enabled = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }
  Type & set__collision_front(
    const bool & _arg)
  {
    this->collision_front = _arg;
    return *this;
  }
  Type & set__collision_rear(
    const bool & _arg)
  {
    this->collision_rear = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__cmd_latency_ms(
    const float & _arg)
  {
    this->cmd_latency_ms = _arg;
    return *this;
  }
  Type & set__lost_frames(
    const uint16_t & _arg)
  {
    this->lost_frames = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t ERR_NONE =
    0u;
  static constexpr uint16_t ERR_ESTOP =
    1u;
  static constexpr uint16_t ERR_FRONT_COLLISION =
    2u;
  static constexpr uint16_t ERR_REAR_COLLISION =
    4u;
  static constexpr uint16_t ERR_COMM_TIMEOUT =
    256u;
  static constexpr uint16_t ERR_IMU_FAULT =
    1024u;

  // pointer types
  using RawPtr =
    custom_interfaces::msg::ChassisStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::ChassisStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::ChassisStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::ChassisStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__ChassisStatus
    std::shared_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__ChassisStatus
    std::shared_ptr<custom_interfaces::msg::ChassisStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChassisStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_enabled != other.motor_enabled) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    if (this->collision_front != other.collision_front) {
      return false;
    }
    if (this->collision_rear != other.collision_rear) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->cmd_latency_ms != other.cmd_latency_ms) {
      return false;
    }
    if (this->lost_frames != other.lost_frames) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChassisStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChassisStatus_

// alias to use template instance with default allocator
using ChassisStatus =
  custom_interfaces::msg::ChassisStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_ESTOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_FRONT_COLLISION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_REAR_COLLISION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_COMM_TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChassisStatus_<ContainerAllocator>::ERR_IMU_FAULT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__CHASSIS_STATUS__STRUCT_HPP_
