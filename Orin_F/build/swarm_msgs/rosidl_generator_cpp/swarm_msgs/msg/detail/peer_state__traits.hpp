// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATE__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/peer_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PeerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: drone_id
  {
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: vn
  {
    out << "vn: ";
    rosidl_generator_traits::value_to_yaml(msg.vn, out);
    out << ", ";
  }

  // member: ve
  {
    out << "ve: ";
    rosidl_generator_traits::value_to_yaml(msg.ve, out);
    out << ", ";
  }

  // member: vd
  {
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    rosidl_generator_traits::value_to_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: guidance_mode
  {
    out << "guidance_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.guidance_mode, out);
    out << ", ";
  }

  // member: fc_mode_code
  {
    out << "fc_mode_code: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_mode_code, out);
    out << ", ";
  }

  // member: fc_armed
  {
    out << "fc_armed: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_armed, out);
    out << ", ";
  }

  // member: fc_connected
  {
    out << "fc_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_connected, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_current
  {
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << ", ";
  }

  // member: battery_remaining
  {
    out << "battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PeerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: vn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vn: ";
    rosidl_generator_traits::value_to_yaml(msg.vn, out);
    out << "\n";
  }

  // member: ve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ve: ";
    rosidl_generator_traits::value_to_yaml(msg.ve, out);
    out << "\n";
  }

  // member: vd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vd: ";
    rosidl_generator_traits::value_to_yaml(msg.vd, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp: ";
    rosidl_generator_traits::value_to_yaml(msg.stamp, out);
    out << "\n";
  }

  // member: guidance_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "guidance_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.guidance_mode, out);
    out << "\n";
  }

  // member: fc_mode_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fc_mode_code: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_mode_code, out);
    out << "\n";
  }

  // member: fc_armed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fc_armed: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_armed, out);
    out << "\n";
  }

  // member: fc_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fc_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.fc_connected, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << "\n";
  }

  // member: battery_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PeerState & msg, bool use_flow_style = false)
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
  const swarm_msgs::msg::PeerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::PeerState & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::PeerState>()
{
  return "swarm_msgs::msg::PeerState";
}

template<>
inline const char * name<swarm_msgs::msg::PeerState>()
{
  return "swarm_msgs/msg/PeerState";
}

template<>
struct has_fixed_size<swarm_msgs::msg::PeerState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swarm_msgs::msg::PeerState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swarm_msgs::msg::PeerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATE__TRAITS_HPP_
