// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PeerState in the package swarm_msgs.
/**
  * Single peer drone state — matches UDP wire format
 */
typedef struct swarm_msgs__msg__PeerState
{
  uint32_t drone_id;
  /// degrees
  double latitude;
  /// degrees
  double longitude;
  /// meters (relative to home)
  double altitude;
  /// North velocity m/s
  double vn;
  /// East velocity m/s
  double ve;
  /// Down velocity m/s
  double vd;
  /// degrees
  double heading;
  /// Unix timestamp from sender
  double stamp;
  /// 0=NONE, 1=TRACKING, 2=CATCHUP, 3=EVASION, 4=FAILSAFE
  uint8_t guidance_mode;
  /// FC state (from peer HEARTBEAT over RFD900x radio)
  /// ArduPilot mode: 0=STABILIZE 4=GUIDED 5=LOITER 6=RTL etc.
  uint8_t fc_mode_code;
  /// Peer FC armed?
  bool fc_armed;
  /// Peer FC connected to MAVROS2?
  bool fc_connected;
  /// Battery state (from peer /mavros/battery relayed over RFD900x as NAMED_VALUE_FLOAT)
  /// Volts
  float battery_voltage;
  /// Amps (negative = discharging on some FCs)
  float battery_current;
  /// 0.0–1.0 fraction (-1.0 = unknown)
  float battery_remaining;
} swarm_msgs__msg__PeerState;

// Struct for a sequence of swarm_msgs__msg__PeerState.
typedef struct swarm_msgs__msg__PeerState__Sequence
{
  swarm_msgs__msg__PeerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__PeerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATE__STRUCT_H_
