// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'peers'
#include "swarm_msgs/msg/detail/peer_state__struct.h"

/// Struct defined in msg/PeerStates in the package swarm_msgs.
/**
  * Array of all non-stale peer states
 */
typedef struct swarm_msgs__msg__PeerStates
{
  builtin_interfaces__msg__Time header_stamp;
  swarm_msgs__msg__PeerState__Sequence peers;
  /// Peer Jetson liveness (from HEARTBEAT over RFD900x radio)
  /// true if HEARTBEAT received within timeout
  bool peer_jetson_alive;
  /// Unix timestamp of last peer HEARTBEAT
  double peer_last_heartbeat;
  /// Own battery state (from local /mavros/battery — SYS_STATUS ID1)
  /// Volts
  float own_battery_voltage;
  /// Amps
  float own_battery_current;
  /// 0.0–1.0 fraction (-1.0 = unknown)
  float own_battery_remaining;
  /// RFD900x radio link quality (from RADIO_STATUS ID109 injected by modem)
  /// Local received signal strength (0-255)
  uint8_t radio_rssi;
  /// Remote received signal strength (0-255)
  uint8_t radio_remrssi;
  /// Transmit buffer free space % (0-100)
  uint8_t radio_txbuf;
  /// Local background noise (0-255)
  uint8_t radio_noise;
  /// Remote background noise (0-255)
  uint8_t radio_remnoise;
  /// Receive error count
  uint16_t radio_rxerrors;
  /// true if RADIO_STATUS received within timeout
  bool radio_link_alive;
  /// Unix timestamp of last RADIO_STATUS
  double radio_last_status;
} swarm_msgs__msg__PeerStates;

// Struct for a sequence of swarm_msgs__msg__PeerStates.
typedef struct swarm_msgs__msg__PeerStates__Sequence
{
  swarm_msgs__msg__PeerStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__PeerStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__PEER_STATES__STRUCT_H_
