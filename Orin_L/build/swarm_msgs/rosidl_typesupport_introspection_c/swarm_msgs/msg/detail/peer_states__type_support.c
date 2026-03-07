// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarm_msgs/msg/detail/peer_states__rosidl_typesupport_introspection_c.h"
#include "swarm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarm_msgs/msg/detail/peer_states__functions.h"
#include "swarm_msgs/msg/detail/peer_states__struct.h"


// Include directives for member types
// Member `header_stamp`
#include "builtin_interfaces/msg/time.h"
// Member `header_stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `peers`
#include "swarm_msgs/msg/peer_state.h"
// Member `peers`
#include "swarm_msgs/msg/detail/peer_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarm_msgs__msg__PeerStates__init(message_memory);
}

void swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_fini_function(void * message_memory)
{
  swarm_msgs__msg__PeerStates__fini(message_memory);
}

size_t swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__size_function__PeerStates__peers(
  const void * untyped_member)
{
  const swarm_msgs__msg__PeerState__Sequence * member =
    (const swarm_msgs__msg__PeerState__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_const_function__PeerStates__peers(
  const void * untyped_member, size_t index)
{
  const swarm_msgs__msg__PeerState__Sequence * member =
    (const swarm_msgs__msg__PeerState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_function__PeerStates__peers(
  void * untyped_member, size_t index)
{
  swarm_msgs__msg__PeerState__Sequence * member =
    (swarm_msgs__msg__PeerState__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__fetch_function__PeerStates__peers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const swarm_msgs__msg__PeerState * item =
    ((const swarm_msgs__msg__PeerState *)
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_const_function__PeerStates__peers(untyped_member, index));
  swarm_msgs__msg__PeerState * value =
    (swarm_msgs__msg__PeerState *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__assign_function__PeerStates__peers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  swarm_msgs__msg__PeerState * item =
    ((swarm_msgs__msg__PeerState *)
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_function__PeerStates__peers(untyped_member, index));
  const swarm_msgs__msg__PeerState * value =
    (const swarm_msgs__msg__PeerState *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__resize_function__PeerStates__peers(
  void * untyped_member, size_t size)
{
  swarm_msgs__msg__PeerState__Sequence * member =
    (swarm_msgs__msg__PeerState__Sequence *)(untyped_member);
  swarm_msgs__msg__PeerState__Sequence__fini(member);
  return swarm_msgs__msg__PeerState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_member_array[15] = {
  {
    "header_stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, header_stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "peers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, peers),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__size_function__PeerStates__peers,  // size() function pointer
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_const_function__PeerStates__peers,  // get_const(index) function pointer
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__get_function__PeerStates__peers,  // get(index) function pointer
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__fetch_function__PeerStates__peers,  // fetch(index, &value) function pointer
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__assign_function__PeerStates__peers,  // assign(index, value) function pointer
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__resize_function__PeerStates__peers  // resize(index) function pointer
  },
  {
    "peer_jetson_alive",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, peer_jetson_alive),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "peer_last_heartbeat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, peer_last_heartbeat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "own_battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, own_battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "own_battery_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, own_battery_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "own_battery_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, own_battery_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_rssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_remrssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_remrssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_txbuf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_txbuf),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_noise",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_noise),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_remnoise",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_remnoise),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_rxerrors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_rxerrors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_link_alive",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_link_alive),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radio_last_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__PeerStates, radio_last_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_members = {
  "swarm_msgs__msg",  // message namespace
  "PeerStates",  // message name
  15,  // number of fields
  sizeof(swarm_msgs__msg__PeerStates),
  swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_member_array,  // message members
  swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_init_function,  // function to initialize message memory (memory has to be allocated)
  swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_type_support_handle = {
  0,
  &swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, PeerStates)() {
  swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, PeerState)();
  if (!swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_type_support_handle.typesupport_identifier) {
    swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarm_msgs__msg__PeerStates__rosidl_typesupport_introspection_c__PeerStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
