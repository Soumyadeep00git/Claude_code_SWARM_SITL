// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/peer_states__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "swarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "swarm_msgs/msg/detail/peer_states__struct.h"
#include "swarm_msgs/msg/detail/peer_states__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "builtin_interfaces/msg/detail/time__functions.h"  // header_stamp
#include "swarm_msgs/msg/detail/peer_state__functions.h"  // peers

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_swarm_msgs
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_swarm_msgs
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_swarm_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();
size_t get_serialized_size_swarm_msgs__msg__PeerState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_swarm_msgs__msg__PeerState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, swarm_msgs, msg, PeerState)();


using _PeerStates__ros_msg_type = swarm_msgs__msg__PeerStates;

static bool _PeerStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PeerStates__ros_msg_type * ros_message = static_cast<const _PeerStates__ros_msg_type *>(untyped_ros_message);
  // Field name: header_stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header_stamp, cdr))
    {
      return false;
    }
  }

  // Field name: peers
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, swarm_msgs, msg, PeerState
      )()->data);
    size_t size = ros_message->peers.size;
    auto array_ptr = ros_message->peers.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: peer_jetson_alive
  {
    cdr << (ros_message->peer_jetson_alive ? true : false);
  }

  // Field name: peer_last_heartbeat
  {
    cdr << ros_message->peer_last_heartbeat;
  }

  // Field name: own_battery_voltage
  {
    cdr << ros_message->own_battery_voltage;
  }

  // Field name: own_battery_current
  {
    cdr << ros_message->own_battery_current;
  }

  // Field name: own_battery_remaining
  {
    cdr << ros_message->own_battery_remaining;
  }

  // Field name: radio_rssi
  {
    cdr << ros_message->radio_rssi;
  }

  // Field name: radio_remrssi
  {
    cdr << ros_message->radio_remrssi;
  }

  // Field name: radio_txbuf
  {
    cdr << ros_message->radio_txbuf;
  }

  // Field name: radio_noise
  {
    cdr << ros_message->radio_noise;
  }

  // Field name: radio_remnoise
  {
    cdr << ros_message->radio_remnoise;
  }

  // Field name: radio_rxerrors
  {
    cdr << ros_message->radio_rxerrors;
  }

  // Field name: radio_link_alive
  {
    cdr << (ros_message->radio_link_alive ? true : false);
  }

  // Field name: radio_last_status
  {
    cdr << ros_message->radio_last_status;
  }

  return true;
}

static bool _PeerStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PeerStates__ros_msg_type * ros_message = static_cast<_PeerStates__ros_msg_type *>(untyped_ros_message);
  // Field name: header_stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header_stamp))
    {
      return false;
    }
  }

  // Field name: peers
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, swarm_msgs, msg, PeerState
      )()->data);
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

    if (ros_message->peers.data) {
      swarm_msgs__msg__PeerState__Sequence__fini(&ros_message->peers);
    }
    if (!swarm_msgs__msg__PeerState__Sequence__init(&ros_message->peers, size)) {
      fprintf(stderr, "failed to create array for field 'peers'");
      return false;
    }
    auto array_ptr = ros_message->peers.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: peer_jetson_alive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->peer_jetson_alive = tmp ? true : false;
  }

  // Field name: peer_last_heartbeat
  {
    cdr >> ros_message->peer_last_heartbeat;
  }

  // Field name: own_battery_voltage
  {
    cdr >> ros_message->own_battery_voltage;
  }

  // Field name: own_battery_current
  {
    cdr >> ros_message->own_battery_current;
  }

  // Field name: own_battery_remaining
  {
    cdr >> ros_message->own_battery_remaining;
  }

  // Field name: radio_rssi
  {
    cdr >> ros_message->radio_rssi;
  }

  // Field name: radio_remrssi
  {
    cdr >> ros_message->radio_remrssi;
  }

  // Field name: radio_txbuf
  {
    cdr >> ros_message->radio_txbuf;
  }

  // Field name: radio_noise
  {
    cdr >> ros_message->radio_noise;
  }

  // Field name: radio_remnoise
  {
    cdr >> ros_message->radio_remnoise;
  }

  // Field name: radio_rxerrors
  {
    cdr >> ros_message->radio_rxerrors;
  }

  // Field name: radio_link_alive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->radio_link_alive = tmp ? true : false;
  }

  // Field name: radio_last_status
  {
    cdr >> ros_message->radio_last_status;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_swarm_msgs
size_t get_serialized_size_swarm_msgs__msg__PeerStates(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PeerStates__ros_msg_type * ros_message = static_cast<const _PeerStates__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header_stamp

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->header_stamp), current_alignment);
  // field.name peers
  {
    size_t array_size = ros_message->peers.size;
    auto array_ptr = ros_message->peers.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_swarm_msgs__msg__PeerState(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name peer_jetson_alive
  {
    size_t item_size = sizeof(ros_message->peer_jetson_alive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name peer_last_heartbeat
  {
    size_t item_size = sizeof(ros_message->peer_last_heartbeat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name own_battery_voltage
  {
    size_t item_size = sizeof(ros_message->own_battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name own_battery_current
  {
    size_t item_size = sizeof(ros_message->own_battery_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name own_battery_remaining
  {
    size_t item_size = sizeof(ros_message->own_battery_remaining);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_rssi
  {
    size_t item_size = sizeof(ros_message->radio_rssi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_remrssi
  {
    size_t item_size = sizeof(ros_message->radio_remrssi);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_txbuf
  {
    size_t item_size = sizeof(ros_message->radio_txbuf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_noise
  {
    size_t item_size = sizeof(ros_message->radio_noise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_remnoise
  {
    size_t item_size = sizeof(ros_message->radio_remnoise);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_rxerrors
  {
    size_t item_size = sizeof(ros_message->radio_rxerrors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_link_alive
  {
    size_t item_size = sizeof(ros_message->radio_link_alive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radio_last_status
  {
    size_t item_size = sizeof(ros_message->radio_last_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PeerStates__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_swarm_msgs__msg__PeerStates(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_swarm_msgs
size_t max_serialized_size_swarm_msgs__msg__PeerStates(
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

  // member: header_stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: peers
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
      size_t inner_size;
      inner_size =
        max_serialized_size_swarm_msgs__msg__PeerState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: peer_jetson_alive
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: peer_last_heartbeat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: own_battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: own_battery_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: own_battery_remaining
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: radio_rssi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_remrssi
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_txbuf
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_noise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_remnoise
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_rxerrors
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: radio_link_alive
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: radio_last_status
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
    using DataType = swarm_msgs__msg__PeerStates;
    is_plain =
      (
      offsetof(DataType, radio_last_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PeerStates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_swarm_msgs__msg__PeerStates(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PeerStates = {
  "swarm_msgs::msg",
  "PeerStates",
  _PeerStates__cdr_serialize,
  _PeerStates__cdr_deserialize,
  _PeerStates__get_serialized_size,
  _PeerStates__max_serialized_size
};

static rosidl_message_type_support_t _PeerStates__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PeerStates,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, swarm_msgs, msg, PeerStates)() {
  return &_PeerStates__type_support;
}

#if defined(__cplusplus)
}
#endif
