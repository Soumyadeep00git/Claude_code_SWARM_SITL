// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__SwarmCommand __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__SwarmCommand __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SwarmCommand_
{
  using Type = SwarmCommand_<ContainerAllocator>;

  explicit SwarmCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->vn = 0.0f;
      this->ve = 0.0f;
      this->vd = 0.0f;
      this->lat = 0.0;
      this->lon = 0.0;
      this->speed = 0.0f;
    }
  }

  explicit SwarmCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->vn = 0.0f;
      this->ve = 0.0f;
      this->vd = 0.0f;
      this->lat = 0.0;
      this->lon = 0.0;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _vn_type =
    float;
  _vn_type vn;
  using _ve_type =
    float;
  _ve_type ve;
  using _vd_type =
    float;
  _vd_type vd;
  using _lat_type =
    double;
  _lat_type lat;
  using _lon_type =
    double;
  _lon_type lon;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__cmd(
    const uint8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__vn(
    const float & _arg)
  {
    this->vn = _arg;
    return *this;
  }
  Type & set__ve(
    const float & _arg)
  {
    this->ve = _arg;
    return *this;
  }
  Type & set__vd(
    const float & _arg)
  {
    this->vd = _arg;
    return *this;
  }
  Type & set__lat(
    const double & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const double & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CMD_RTL =
    1u;
  static constexpr uint8_t CMD_LAND =
    2u;
  static constexpr uint8_t CMD_KILL =
    3u;
  static constexpr uint8_t CMD_FOLLOW =
    4u;
  static constexpr uint8_t CMD_HOVER =
    5u;
  static constexpr uint8_t CMD_TAKEOFF =
    6u;
  static constexpr uint8_t CMD_WASD =
    10u;
  static constexpr uint8_t CMD_WAYPOINT =
    11u;
  static constexpr uint8_t CMD_SPEED =
    12u;
  static constexpr uint8_t CMD_ALTITUDE =
    13u;

  // pointer types
  using RawPtr =
    swarm_msgs::msg::SwarmCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::SwarmCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::SwarmCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::SwarmCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__SwarmCommand
    std::shared_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__SwarmCommand
    std::shared_ptr<swarm_msgs::msg::SwarmCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SwarmCommand_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->vn != other.vn) {
      return false;
    }
    if (this->ve != other.ve) {
      return false;
    }
    if (this->vd != other.vd) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const SwarmCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SwarmCommand_

// alias to use template instance with default allocator
using SwarmCommand =
  swarm_msgs::msg::SwarmCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_RTL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_LAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_KILL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_FOLLOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_HOVER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_TAKEOFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_WASD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_WAYPOINT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_SPEED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SwarmCommand_<ContainerAllocator>::CMD_ALTITUDE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_HPP_
