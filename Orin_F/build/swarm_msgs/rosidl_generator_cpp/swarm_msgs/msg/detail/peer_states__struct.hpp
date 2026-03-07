// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'peers'
#include "swarm_msgs/msg/detail/peer_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__PeerStates __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__PeerStates __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PeerStates_
{
  using Type = PeerStates_<ContainerAllocator>;

  explicit PeerStates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->peer_jetson_alive = false;
      this->peer_last_heartbeat = 0.0;
      this->own_battery_voltage = 0.0f;
      this->own_battery_current = 0.0f;
      this->own_battery_remaining = 0.0f;
      this->radio_rssi = 0;
      this->radio_remrssi = 0;
      this->radio_txbuf = 0;
      this->radio_noise = 0;
      this->radio_remnoise = 0;
      this->radio_rxerrors = 0;
      this->radio_link_alive = false;
      this->radio_last_status = 0.0;
    }
  }

  explicit PeerStates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header_stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->peer_jetson_alive = false;
      this->peer_last_heartbeat = 0.0;
      this->own_battery_voltage = 0.0f;
      this->own_battery_current = 0.0f;
      this->own_battery_remaining = 0.0f;
      this->radio_rssi = 0;
      this->radio_remrssi = 0;
      this->radio_txbuf = 0;
      this->radio_noise = 0;
      this->radio_remnoise = 0;
      this->radio_rxerrors = 0;
      this->radio_link_alive = false;
      this->radio_last_status = 0.0;
    }
  }

  // field types and members
  using _header_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _header_stamp_type header_stamp;
  using _peers_type =
    std::vector<swarm_msgs::msg::PeerState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<swarm_msgs::msg::PeerState_<ContainerAllocator>>>;
  _peers_type peers;
  using _peer_jetson_alive_type =
    bool;
  _peer_jetson_alive_type peer_jetson_alive;
  using _peer_last_heartbeat_type =
    double;
  _peer_last_heartbeat_type peer_last_heartbeat;
  using _own_battery_voltage_type =
    float;
  _own_battery_voltage_type own_battery_voltage;
  using _own_battery_current_type =
    float;
  _own_battery_current_type own_battery_current;
  using _own_battery_remaining_type =
    float;
  _own_battery_remaining_type own_battery_remaining;
  using _radio_rssi_type =
    uint8_t;
  _radio_rssi_type radio_rssi;
  using _radio_remrssi_type =
    uint8_t;
  _radio_remrssi_type radio_remrssi;
  using _radio_txbuf_type =
    uint8_t;
  _radio_txbuf_type radio_txbuf;
  using _radio_noise_type =
    uint8_t;
  _radio_noise_type radio_noise;
  using _radio_remnoise_type =
    uint8_t;
  _radio_remnoise_type radio_remnoise;
  using _radio_rxerrors_type =
    uint16_t;
  _radio_rxerrors_type radio_rxerrors;
  using _radio_link_alive_type =
    bool;
  _radio_link_alive_type radio_link_alive;
  using _radio_last_status_type =
    double;
  _radio_last_status_type radio_last_status;

  // setters for named parameter idiom
  Type & set__header_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->header_stamp = _arg;
    return *this;
  }
  Type & set__peers(
    const std::vector<swarm_msgs::msg::PeerState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<swarm_msgs::msg::PeerState_<ContainerAllocator>>> & _arg)
  {
    this->peers = _arg;
    return *this;
  }
  Type & set__peer_jetson_alive(
    const bool & _arg)
  {
    this->peer_jetson_alive = _arg;
    return *this;
  }
  Type & set__peer_last_heartbeat(
    const double & _arg)
  {
    this->peer_last_heartbeat = _arg;
    return *this;
  }
  Type & set__own_battery_voltage(
    const float & _arg)
  {
    this->own_battery_voltage = _arg;
    return *this;
  }
  Type & set__own_battery_current(
    const float & _arg)
  {
    this->own_battery_current = _arg;
    return *this;
  }
  Type & set__own_battery_remaining(
    const float & _arg)
  {
    this->own_battery_remaining = _arg;
    return *this;
  }
  Type & set__radio_rssi(
    const uint8_t & _arg)
  {
    this->radio_rssi = _arg;
    return *this;
  }
  Type & set__radio_remrssi(
    const uint8_t & _arg)
  {
    this->radio_remrssi = _arg;
    return *this;
  }
  Type & set__radio_txbuf(
    const uint8_t & _arg)
  {
    this->radio_txbuf = _arg;
    return *this;
  }
  Type & set__radio_noise(
    const uint8_t & _arg)
  {
    this->radio_noise = _arg;
    return *this;
  }
  Type & set__radio_remnoise(
    const uint8_t & _arg)
  {
    this->radio_remnoise = _arg;
    return *this;
  }
  Type & set__radio_rxerrors(
    const uint16_t & _arg)
  {
    this->radio_rxerrors = _arg;
    return *this;
  }
  Type & set__radio_link_alive(
    const bool & _arg)
  {
    this->radio_link_alive = _arg;
    return *this;
  }
  Type & set__radio_last_status(
    const double & _arg)
  {
    this->radio_last_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_msgs::msg::PeerStates_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::PeerStates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::PeerStates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::PeerStates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__PeerStates
    std::shared_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__PeerStates
    std::shared_ptr<swarm_msgs::msg::PeerStates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PeerStates_ & other) const
  {
    if (this->header_stamp != other.header_stamp) {
      return false;
    }
    if (this->peers != other.peers) {
      return false;
    }
    if (this->peer_jetson_alive != other.peer_jetson_alive) {
      return false;
    }
    if (this->peer_last_heartbeat != other.peer_last_heartbeat) {
      return false;
    }
    if (this->own_battery_voltage != other.own_battery_voltage) {
      return false;
    }
    if (this->own_battery_current != other.own_battery_current) {
      return false;
    }
    if (this->own_battery_remaining != other.own_battery_remaining) {
      return false;
    }
    if (this->radio_rssi != other.radio_rssi) {
      return false;
    }
    if (this->radio_remrssi != other.radio_remrssi) {
      return false;
    }
    if (this->radio_txbuf != other.radio_txbuf) {
      return false;
    }
    if (this->radio_noise != other.radio_noise) {
      return false;
    }
    if (this->radio_remnoise != other.radio_remnoise) {
      return false;
    }
    if (this->radio_rxerrors != other.radio_rxerrors) {
      return false;
    }
    if (this->radio_link_alive != other.radio_link_alive) {
      return false;
    }
    if (this->radio_last_status != other.radio_last_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const PeerStates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PeerStates_

// alias to use template instance with default allocator
using PeerStates =
  swarm_msgs::msg::PeerStates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_HPP_
