// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATES__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/peer_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_PeerStates_radio_last_status
{
public:
  explicit Init_PeerStates_radio_last_status(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::PeerStates radio_last_status(::swarm_msgs::msg::PeerStates::_radio_last_status_type arg)
  {
    msg_.radio_last_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_link_alive
{
public:
  explicit Init_PeerStates_radio_link_alive(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_last_status radio_link_alive(::swarm_msgs::msg::PeerStates::_radio_link_alive_type arg)
  {
    msg_.radio_link_alive = std::move(arg);
    return Init_PeerStates_radio_last_status(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_rxerrors
{
public:
  explicit Init_PeerStates_radio_rxerrors(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_link_alive radio_rxerrors(::swarm_msgs::msg::PeerStates::_radio_rxerrors_type arg)
  {
    msg_.radio_rxerrors = std::move(arg);
    return Init_PeerStates_radio_link_alive(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_remnoise
{
public:
  explicit Init_PeerStates_radio_remnoise(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_rxerrors radio_remnoise(::swarm_msgs::msg::PeerStates::_radio_remnoise_type arg)
  {
    msg_.radio_remnoise = std::move(arg);
    return Init_PeerStates_radio_rxerrors(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_noise
{
public:
  explicit Init_PeerStates_radio_noise(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_remnoise radio_noise(::swarm_msgs::msg::PeerStates::_radio_noise_type arg)
  {
    msg_.radio_noise = std::move(arg);
    return Init_PeerStates_radio_remnoise(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_txbuf
{
public:
  explicit Init_PeerStates_radio_txbuf(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_noise radio_txbuf(::swarm_msgs::msg::PeerStates::_radio_txbuf_type arg)
  {
    msg_.radio_txbuf = std::move(arg);
    return Init_PeerStates_radio_noise(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_remrssi
{
public:
  explicit Init_PeerStates_radio_remrssi(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_txbuf radio_remrssi(::swarm_msgs::msg::PeerStates::_radio_remrssi_type arg)
  {
    msg_.radio_remrssi = std::move(arg);
    return Init_PeerStates_radio_txbuf(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_radio_rssi
{
public:
  explicit Init_PeerStates_radio_rssi(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_remrssi radio_rssi(::swarm_msgs::msg::PeerStates::_radio_rssi_type arg)
  {
    msg_.radio_rssi = std::move(arg);
    return Init_PeerStates_radio_remrssi(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_own_battery_remaining
{
public:
  explicit Init_PeerStates_own_battery_remaining(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_radio_rssi own_battery_remaining(::swarm_msgs::msg::PeerStates::_own_battery_remaining_type arg)
  {
    msg_.own_battery_remaining = std::move(arg);
    return Init_PeerStates_radio_rssi(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_own_battery_current
{
public:
  explicit Init_PeerStates_own_battery_current(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_own_battery_remaining own_battery_current(::swarm_msgs::msg::PeerStates::_own_battery_current_type arg)
  {
    msg_.own_battery_current = std::move(arg);
    return Init_PeerStates_own_battery_remaining(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_own_battery_voltage
{
public:
  explicit Init_PeerStates_own_battery_voltage(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_own_battery_current own_battery_voltage(::swarm_msgs::msg::PeerStates::_own_battery_voltage_type arg)
  {
    msg_.own_battery_voltage = std::move(arg);
    return Init_PeerStates_own_battery_current(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_peer_last_heartbeat
{
public:
  explicit Init_PeerStates_peer_last_heartbeat(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_own_battery_voltage peer_last_heartbeat(::swarm_msgs::msg::PeerStates::_peer_last_heartbeat_type arg)
  {
    msg_.peer_last_heartbeat = std::move(arg);
    return Init_PeerStates_own_battery_voltage(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_peer_jetson_alive
{
public:
  explicit Init_PeerStates_peer_jetson_alive(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_peer_last_heartbeat peer_jetson_alive(::swarm_msgs::msg::PeerStates::_peer_jetson_alive_type arg)
  {
    msg_.peer_jetson_alive = std::move(arg);
    return Init_PeerStates_peer_last_heartbeat(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_peers
{
public:
  explicit Init_PeerStates_peers(::swarm_msgs::msg::PeerStates & msg)
  : msg_(msg)
  {}
  Init_PeerStates_peer_jetson_alive peers(::swarm_msgs::msg::PeerStates::_peers_type arg)
  {
    msg_.peers = std::move(arg);
    return Init_PeerStates_peer_jetson_alive(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

class Init_PeerStates_header_stamp
{
public:
  Init_PeerStates_header_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PeerStates_peers header_stamp(::swarm_msgs::msg::PeerStates::_header_stamp_type arg)
  {
    msg_.header_stamp = std::move(arg);
    return Init_PeerStates_peers(msg_);
  }

private:
  ::swarm_msgs::msg::PeerStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::PeerStates>()
{
  return swarm_msgs::msg::builder::Init_PeerStates_header_stamp();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATES__BUILDER_HPP_
