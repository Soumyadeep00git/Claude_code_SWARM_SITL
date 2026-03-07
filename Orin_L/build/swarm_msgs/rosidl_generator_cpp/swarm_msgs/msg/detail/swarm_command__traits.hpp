// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/swarm_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SwarmCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
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

  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SwarmCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
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

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SwarmCommand & msg, bool use_flow_style = false)
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
  const swarm_msgs::msg::SwarmCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::SwarmCommand & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::SwarmCommand>()
{
  return "swarm_msgs::msg::SwarmCommand";
}

template<>
inline const char * name<swarm_msgs::msg::SwarmCommand>()
{
  return "swarm_msgs/msg/SwarmCommand";
}

template<>
struct has_fixed_size<swarm_msgs::msg::SwarmCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swarm_msgs::msg::SwarmCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swarm_msgs::msg::SwarmCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__TRAITS_HPP_
