// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CMD_RTL'.
/**
  * Command constants
 */
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_RTL = 1
};

/// Constant 'CMD_LAND'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_LAND = 2
};

/// Constant 'CMD_KILL'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_KILL = 3
};

/// Constant 'CMD_FOLLOW'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_FOLLOW = 4
};

/// Constant 'CMD_HOVER'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_HOVER = 5
};

/// Constant 'CMD_TAKEOFF'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_TAKEOFF = 6
};

/// Constant 'CMD_WASD'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_WASD = 10
};

/// Constant 'CMD_WAYPOINT'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_WAYPOINT = 11
};

/// Constant 'CMD_SPEED'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_SPEED = 12
};

/// Constant 'CMD_ALTITUDE'.
enum
{
  swarm_msgs__msg__SwarmCommand__CMD_ALTITUDE = 13
};

/// Struct defined in msg/SwarmCommand in the package swarm_msgs.
/**
  * GCS command — matches UDP command wire format
 */
typedef struct swarm_msgs__msg__SwarmCommand
{
  uint8_t cmd;
  /// WASD north velocity (CMD_WASD)
  float vn;
  /// WASD east velocity (CMD_WASD)
  float ve;
  /// vertical velocity (CMD_ALTITUDE)
  float vd;
  /// waypoint latitude (CMD_WAYPOINT)
  double lat;
  /// waypoint longitude (CMD_WAYPOINT)
  double lon;
  /// speed setting (CMD_SPEED)
  float speed;
} swarm_msgs__msg__SwarmCommand;

// Struct for a sequence of swarm_msgs__msg__SwarmCommand.
typedef struct swarm_msgs__msg__SwarmCommand__Sequence
{
  swarm_msgs__msg__SwarmCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__SwarmCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__STRUCT_H_
