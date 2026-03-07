// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__PeerState __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__PeerState __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PeerState_
{
  using Type = PeerState_<ContainerAllocator>;

  explicit PeerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0ul;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->vn = 0.0;
      this->ve = 0.0;
      this->vd = 0.0;
      this->heading = 0.0;
      this->stamp = 0.0;
      this->guidance_mode = 0;
      this->fc_mode_code = 0;
      this->fc_armed = false;
      this->fc_connected = false;
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_remaining = 0.0f;
    }
  }

  explicit PeerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0ul;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->vn = 0.0;
      this->ve = 0.0;
      this->vd = 0.0;
      this->heading = 0.0;
      this->stamp = 0.0;
      this->guidance_mode = 0;
      this->fc_mode_code = 0;
      this->fc_armed = false;
      this->fc_connected = false;
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_remaining = 0.0f;
    }
  }

  // field types and members
  using _drone_id_type =
    uint32_t;
  _drone_id_type drone_id;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _vn_type =
    double;
  _vn_type vn;
  using _ve_type =
    double;
  _ve_type ve;
  using _vd_type =
    double;
  _vd_type vd;
  using _heading_type =
    double;
  _heading_type heading;
  using _stamp_type =
    double;
  _stamp_type stamp;
  using _guidance_mode_type =
    uint8_t;
  _guidance_mode_type guidance_mode;
  using _fc_mode_code_type =
    uint8_t;
  _fc_mode_code_type fc_mode_code;
  using _fc_armed_type =
    bool;
  _fc_armed_type fc_armed;
  using _fc_connected_type =
    bool;
  _fc_connected_type fc_connected;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_current_type =
    float;
  _battery_current_type battery_current;
  using _battery_remaining_type =
    float;
  _battery_remaining_type battery_remaining;

  // setters for named parameter idiom
  Type & set__drone_id(
    const uint32_t & _arg)
  {
    this->drone_id = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__vn(
    const double & _arg)
  {
    this->vn = _arg;
    return *this;
  }
  Type & set__ve(
    const double & _arg)
  {
    this->ve = _arg;
    return *this;
  }
  Type & set__vd(
    const double & _arg)
  {
    this->vd = _arg;
    return *this;
  }
  Type & set__heading(
    const double & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__stamp(
    const double & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__guidance_mode(
    const uint8_t & _arg)
  {
    this->guidance_mode = _arg;
    return *this;
  }
  Type & set__fc_mode_code(
    const uint8_t & _arg)
  {
    this->fc_mode_code = _arg;
    return *this;
  }
  Type & set__fc_armed(
    const bool & _arg)
  {
    this->fc_armed = _arg;
    return *this;
  }
  Type & set__fc_connected(
    const bool & _arg)
  {
    this->fc_connected = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_current(
    const float & _arg)
  {
    this->battery_current = _arg;
    return *this;
  }
  Type & set__battery_remaining(
    const float & _arg)
  {
    this->battery_remaining = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_msgs::msg::PeerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::PeerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::PeerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::PeerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__PeerState
    std::shared_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__PeerState
    std::shared_ptr<swarm_msgs::msg::PeerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PeerState_ & other) const
  {
    if (this->drone_id != other.drone_id) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
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
    if (this->heading != other.heading) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->guidance_mode != other.guidance_mode) {
      return false;
    }
    if (this->fc_mode_code != other.fc_mode_code) {
      return false;
    }
    if (this->fc_armed != other.fc_armed) {
      return false;
    }
    if (this->fc_connected != other.fc_connected) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_current != other.battery_current) {
      return false;
    }
    if (this->battery_remaining != other.battery_remaining) {
      return false;
    }
    return true;
  }
  bool operator!=(const PeerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PeerState_

// alias to use template instance with default allocator
using PeerState =
  swarm_msgs::msg::PeerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_HPP_
