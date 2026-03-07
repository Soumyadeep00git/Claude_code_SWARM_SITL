// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATES__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/peer_states__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'peers'
#include "swarm_msgs/msg/detail/peer_state__traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PeerStates & msg,
  std::ostream & out)
{
  out << "{";
  // member: header_stamp
  {
    out << "header_stamp: ";
    to_flow_style_yaml(msg.header_stamp, out);
    out << ", ";
  }

  // member: peers
  {
    if (msg.peers.size() == 0) {
      out << "peers: []";
    } else {
      out << "peers: [";
      size_t pending_items = msg.peers.size();
      for (auto item : msg.peers) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: peer_jetson_alive
  {
    out << "peer_jetson_alive: ";
    rosidl_generator_traits::value_to_yaml(msg.peer_jetson_alive, out);
    out << ", ";
  }

  // member: peer_last_heartbeat
  {
    out << "peer_last_heartbeat: ";
    rosidl_generator_traits::value_to_yaml(msg.peer_last_heartbeat, out);
    out << ", ";
  }

  // member: own_battery_voltage
  {
    out << "own_battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_voltage, out);
    out << ", ";
  }

  // member: own_battery_current
  {
    out << "own_battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_current, out);
    out << ", ";
  }

  // member: own_battery_remaining
  {
    out << "own_battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_remaining, out);
    out << ", ";
  }

  // member: radio_rssi
  {
    out << "radio_rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_rssi, out);
    out << ", ";
  }

  // member: radio_remrssi
  {
    out << "radio_remrssi: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_remrssi, out);
    out << ", ";
  }

  // member: radio_txbuf
  {
    out << "radio_txbuf: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_txbuf, out);
    out << ", ";
  }

  // member: radio_noise
  {
    out << "radio_noise: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_noise, out);
    out << ", ";
  }

  // member: radio_remnoise
  {
    out << "radio_remnoise: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_remnoise, out);
    out << ", ";
  }

  // member: radio_rxerrors
  {
    out << "radio_rxerrors: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_rxerrors, out);
    out << ", ";
  }

  // member: radio_link_alive
  {
    out << "radio_link_alive: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_link_alive, out);
    out << ", ";
  }

  // member: radio_last_status
  {
    out << "radio_last_status: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_last_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PeerStates & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header_stamp:\n";
    to_block_style_yaml(msg.header_stamp, out, indentation + 2);
  }

  // member: peers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.peers.size() == 0) {
      out << "peers: []\n";
    } else {
      out << "peers:\n";
      for (auto item : msg.peers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: peer_jetson_alive
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "peer_jetson_alive: ";
    rosidl_generator_traits::value_to_yaml(msg.peer_jetson_alive, out);
    out << "\n";
  }

  // member: peer_last_heartbeat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "peer_last_heartbeat: ";
    rosidl_generator_traits::value_to_yaml(msg.peer_last_heartbeat, out);
    out << "\n";
  }

  // member: own_battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "own_battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_voltage, out);
    out << "\n";
  }

  // member: own_battery_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "own_battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_current, out);
    out << "\n";
  }

  // member: own_battery_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "own_battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.own_battery_remaining, out);
    out << "\n";
  }

  // member: radio_rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_rssi, out);
    out << "\n";
  }

  // member: radio_remrssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_remrssi: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_remrssi, out);
    out << "\n";
  }

  // member: radio_txbuf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_txbuf: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_txbuf, out);
    out << "\n";
  }

  // member: radio_noise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_noise: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_noise, out);
    out << "\n";
  }

  // member: radio_remnoise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_remnoise: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_remnoise, out);
    out << "\n";
  }

  // member: radio_rxerrors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_rxerrors: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_rxerrors, out);
    out << "\n";
  }

  // member: radio_link_alive
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_link_alive: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_link_alive, out);
    out << "\n";
  }

  // member: radio_last_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radio_last_status: ";
    rosidl_generator_traits::value_to_yaml(msg.radio_last_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PeerStates & msg, bool use_flow_style = false)
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

}  // namespace swarm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use swarm_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swarm_msgs::msg::PeerStates & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::PeerStates & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::PeerStates>()
{
  return "swarm_msgs::msg::PeerStates";
}

template<>
inline const char * name<swarm_msgs::msg::PeerStates>()
{
  return "swarm_msgs/msg/PeerStates";
}

template<>
struct has_fixed_size<swarm_msgs::msg::PeerStates>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swarm_msgs::msg::PeerStates>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swarm_msgs::msg::PeerStates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATES__TRAITS_HPP_
