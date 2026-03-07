// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/peer_states__rosidl_typesupport_fastrtps_cpp.hpp"
#include "swarm_msgs/msg/detail/peer_states__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces

namespace swarm_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const swarm_msgs::msg::PeerState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  swarm_msgs::msg::PeerState &);
size_t get_serialized_size(
  const swarm_msgs::msg::PeerState &,
  size_t current_alignment);
size_t
max_serialized_size_PeerState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace swarm_msgs


namespace swarm_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_serialize(
  const swarm_msgs::msg::PeerStates & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header_stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header_stamp,
    cdr);
  // Member: peers
  {
    size_t size = ros_message.peers.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      swarm_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.peers[i],
        cdr);
    }
  }
  // Member: peer_jetson_alive
  cdr << (ros_message.peer_jetson_alive ? true : false);
  // Member: peer_last_heartbeat
  cdr << ros_message.peer_last_heartbeat;
  // Member: own_battery_voltage
  cdr << ros_message.own_battery_voltage;
  // Member: own_battery_current
  cdr << ros_message.own_battery_current;
  // Member: own_battery_remaining
  cdr << ros_message.own_battery_remaining;
  // Member: radio_rssi
  cdr << ros_message.radio_rssi;
  // Member: radio_remrssi
  cdr << ros_message.radio_remrssi;
  // Member: radio_txbuf
  cdr << ros_message.radio_txbuf;
  // Member: radio_noise
  cdr << ros_message.radio_noise;
  // Member: radio_remnoise
  cdr << ros_message.radio_remnoise;
  // Member: radio_rxerrors
  cdr << ros_message.radio_rxerrors;
  // Member: radio_link_alive
  cdr << (ros_message.radio_link_alive ? true : false);
  // Member: radio_last_status
  cdr << ros_message.radio_last_status;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  swarm_msgs::msg::PeerStates & ros_message)
{
  // Member: header_stamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header_stamp);

  // Member: peers
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    ros_message.peers.resize(size);
    for (size_t i = 0; i < size; i++) {
      swarm_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.peers[i]);
    }
  }

  // Member: peer_jetson_alive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.peer_jetson_alive = tmp ? true : false;
  }

  // Member: peer_last_heartbeat
  cdr >> ros_message.peer_last_heartbeat;

  // Member: own_battery_voltage
  cdr >> ros_message.own_battery_voltage;

  // Member: own_battery_current
  cdr >> ros_message.own_battery_current;

  // Member: own_battery_remaining
  cdr >> ros_message.own_battery_remaining;

  // Member: radio_rssi
  cdr >> ros_message.radio_rssi;

  // Member: radio_remrssi
  cdr >> ros_message.radio_remrssi;

  // Member: radio_txbuf
  cdr >> ros_message.radio_txbuf;

  // Member: radio_noise
  cdr >> ros_message.radio_noise;

  // Member: radio_remnoise
  cdr >> ros_message.radio_remnoise;

  // Member: radio_rxerrors
  cdr >> ros_message.radio_rxerrors;

  // Member: radio_link_alive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.radio_link_alive = tmp ? true : false;
  }

  // Member: radio_last_status
  cdr >> ros_message.radio_last_status;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
get_serialized_size(
  const swarm_msgs::msg::PeerStates & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header_stamp

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header_stamp, current_alignment);
  // Member: peers
  {
    size_t array_size = ros_message.peers.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        swarm_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.peers[index], current_alignment);
    }
  }
  // Member: peer_jetson_alive
  {
    size_t item_size = sizeof(ros_message.peer_jetson_alive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: peer_last_heartbeat
  {
    size_t item_size = sizeof(ros_message.peer_last_heartbeat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: own_battery_voltage
  {
    size_t item_size = sizeof(ros_message.own_battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: own_battery_current
  {
    size_t item_size = sizeof(ros_message.own_battery_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: own_battery_remaining
  {
    size_t item_size = sizeof(ros_message.own_battery_remaining);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_rssi
  {
    size_t item_size = sizeof(ros_message.radio_rssi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_remrssi
  {
    size_t item_size = sizeof(ros_message.radio_remrssi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_txbuf
  {
    size_t item_size = sizeof(ros_message.radio_txbuf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_noise
  {
    size_t item_size = sizeof(ros_message.radio_noise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_remnoise
  {
    size_t item_size = sizeof(ros_message.radio_remnoise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_rxerrors
  {
    size_t item_size = sizeof(ros_message.radio_rxerrors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_link_alive
  {
    size_t item_size = sizeof(ros_message.radio_link_alive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: radio_last_status
  {
    size_t item_size = sizeof(ros_message.radio_last_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
max_serialized_size_PeerStates(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header_stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: peers
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        swarm_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PeerState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: peer_jetson_alive
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: peer_last_heartbeat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: own_battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: own_battery_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: own_battery_remaining
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: radio_rssi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_remrssi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_txbuf
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_noise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_remnoise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_rxerrors
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: radio_link_alive
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: radio_last_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = swarm_msgs::msg::PeerStates;
    is_plain =
      (
      offsetof(DataType, radio_last_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PeerStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const swarm_msgs::msg::PeerStates *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PeerStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<swarm_msgs::msg::PeerStates *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PeerStates__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const swarm_msgs::msg::PeerStates *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PeerStates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PeerStates(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PeerStates__callbacks = {
  "swarm_msgs::msg",
  "PeerStates",
  _PeerStates__cdr_serialize,
  _PeerStates__cdr_deserialize,
  _PeerStates__get_serialized_size,
  _PeerStates__max_serialized_size
};

static rosidl_message_type_support_t _PeerStates__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PeerStates__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace swarm_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<swarm_msgs::msg::PeerStates>()
{
  return &swarm_msgs::msg::typesupport_fastrtps_cpp::_PeerStates__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, swarm_msgs, msg, PeerStates)() {
  return &swarm_msgs::msg::typesupport_fastrtps_cpp::_PeerStates__handle;
}

#ifdef __cplusplus
}
#endif
