// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATE__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/peer_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_PeerState_battery_remaining
{
public:
  explicit Init_PeerState_battery_remaining(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::PeerState battery_remaining(::swarm_msgs::msg::PeerState::_battery_remaining_type arg)
  {
    msg_.battery_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_battery_current
{
public:
  explicit Init_PeerState_battery_current(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_battery_remaining battery_current(::swarm_msgs::msg::PeerState::_battery_current_type arg)
  {
    msg_.battery_current = std::move(arg);
    return Init_PeerState_battery_remaining(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_battery_voltage
{
public:
  explicit Init_PeerState_battery_voltage(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_battery_current battery_voltage(::swarm_msgs::msg::PeerState::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_PeerState_battery_current(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_fc_connected
{
public:
  explicit Init_PeerState_fc_connected(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_battery_voltage fc_connected(::swarm_msgs::msg::PeerState::_fc_connected_type arg)
  {
    msg_.fc_connected = std::move(arg);
    return Init_PeerState_battery_voltage(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_fc_armed
{
public:
  explicit Init_PeerState_fc_armed(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_fc_connected fc_armed(::swarm_msgs::msg::PeerState::_fc_armed_type arg)
  {
    msg_.fc_armed = std::move(arg);
    return Init_PeerState_fc_connected(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_fc_mode_code
{
public:
  explicit Init_PeerState_fc_mode_code(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_fc_armed fc_mode_code(::swarm_msgs::msg::PeerState::_fc_mode_code_type arg)
  {
    msg_.fc_mode_code = std::move(arg);
    return Init_PeerState_fc_armed(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_guidance_mode
{
public:
  explicit Init_PeerState_guidance_mode(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_fc_mode_code guidance_mode(::swarm_msgs::msg::PeerState::_guidance_mode_type arg)
  {
    msg_.guidance_mode = std::move(arg);
    return Init_PeerState_fc_mode_code(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_stamp
{
public:
  explicit Init_PeerState_stamp(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_guidance_mode stamp(::swarm_msgs::msg::PeerState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_PeerState_guidance_mode(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_heading
{
public:
  explicit Init_PeerState_heading(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_stamp heading(::swarm_msgs::msg::PeerState::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_PeerState_stamp(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_vd
{
public:
  explicit Init_PeerState_vd(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_heading vd(::swarm_msgs::msg::PeerState::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_PeerState_heading(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_ve
{
public:
  explicit Init_PeerState_ve(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_vd ve(::swarm_msgs::msg::PeerState::_ve_type arg)
  {
    msg_.ve = std::move(arg);
    return Init_PeerState_vd(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_vn
{
public:
  explicit Init_PeerState_vn(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_ve vn(::swarm_msgs::msg::PeerState::_vn_type arg)
  {
    msg_.vn = std::move(arg);
    return Init_PeerState_ve(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_altitude
{
public:
  explicit Init_PeerState_altitude(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_vn altitude(::swarm_msgs::msg::PeerState::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_PeerState_vn(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_longitude
{
public:
  explicit Init_PeerState_longitude(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_altitude longitude(::swarm_msgs::msg::PeerState::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_PeerState_altitude(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_latitude
{
public:
  explicit Init_PeerState_latitude(::swarm_msgs::msg::PeerState & msg)
  : msg_(msg)
  {}
  Init_PeerState_longitude latitude(::swarm_msgs::msg::PeerState::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_PeerState_longitude(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

class Init_PeerState_drone_id
{
public:
  Init_PeerState_drone_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PeerState_latitude drone_id(::swarm_msgs::msg::PeerState::_drone_id_type arg)
  {
    msg_.drone_id = std::move(arg);
    return Init_PeerState_latitude(msg_);
  }

private:
  ::swarm_msgs::msg::PeerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::PeerState>()
{
  return swarm_msgs::msg::builder::Init_PeerState_drone_id();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATE__BUILDER_HPP_
