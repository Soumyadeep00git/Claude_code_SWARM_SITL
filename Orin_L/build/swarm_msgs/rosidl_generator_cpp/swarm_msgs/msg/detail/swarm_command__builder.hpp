// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/swarm_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_SwarmCommand_speed
{
public:
  explicit Init_SwarmCommand_speed(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::SwarmCommand speed(::swarm_msgs::msg::SwarmCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_lon
{
public:
  explicit Init_SwarmCommand_lon(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  Init_SwarmCommand_speed lon(::swarm_msgs::msg::SwarmCommand::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_SwarmCommand_speed(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_lat
{
public:
  explicit Init_SwarmCommand_lat(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  Init_SwarmCommand_lon lat(::swarm_msgs::msg::SwarmCommand::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_SwarmCommand_lon(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_vd
{
public:
  explicit Init_SwarmCommand_vd(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  Init_SwarmCommand_lat vd(::swarm_msgs::msg::SwarmCommand::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_SwarmCommand_lat(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_ve
{
public:
  explicit Init_SwarmCommand_ve(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  Init_SwarmCommand_vd ve(::swarm_msgs::msg::SwarmCommand::_ve_type arg)
  {
    msg_.ve = std::move(arg);
    return Init_SwarmCommand_vd(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_vn
{
public:
  explicit Init_SwarmCommand_vn(::swarm_msgs::msg::SwarmCommand & msg)
  : msg_(msg)
  {}
  Init_SwarmCommand_ve vn(::swarm_msgs::msg::SwarmCommand::_vn_type arg)
  {
    msg_.vn = std::move(arg);
    return Init_SwarmCommand_ve(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

class Init_SwarmCommand_cmd
{
public:
  Init_SwarmCommand_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwarmCommand_vn cmd(::swarm_msgs::msg::SwarmCommand::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_SwarmCommand_vn(msg_);
  }

private:
  ::swarm_msgs::msg::SwarmCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::SwarmCommand>()
{
  return swarm_msgs::msg::builder::Init_SwarmCommand_cmd();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__BUILDER_HPP_
