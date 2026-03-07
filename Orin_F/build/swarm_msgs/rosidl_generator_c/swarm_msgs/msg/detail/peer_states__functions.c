// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/PeerStates.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/peer_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `peers`
#include "swarm_msgs/msg/detail/peer_state__functions.h"

bool
swarm_msgs__msg__PeerStates__init(swarm_msgs__msg__PeerStates * msg)
{
  if (!msg) {
    return false;
  }
  // header_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->header_stamp)) {
    swarm_msgs__msg__PeerStates__fini(msg);
    return false;
  }
  // peers
  if (!swarm_msgs__msg__PeerState__Sequence__init(&msg->peers, 0)) {
    swarm_msgs__msg__PeerStates__fini(msg);
    return false;
  }
  // peer_jetson_alive
  // peer_last_heartbeat
  // own_battery_voltage
  // own_battery_current
  // own_battery_remaining
  // radio_rssi
  // radio_remrssi
  // radio_txbuf
  // radio_noise
  // radio_remnoise
  // radio_rxerrors
  // radio_link_alive
  // radio_last_status
  return true;
}

void
swarm_msgs__msg__PeerStates__fini(swarm_msgs__msg__PeerStates * msg)
{
  if (!msg) {
    return;
  }
  // header_stamp
  builtin_interfaces__msg__Time__fini(&msg->header_stamp);
  // peers
  swarm_msgs__msg__PeerState__Sequence__fini(&msg->peers);
  // peer_jetson_alive
  // peer_last_heartbeat
  // own_battery_voltage
  // own_battery_current
  // own_battery_remaining
  // radio_rssi
  // radio_remrssi
  // radio_txbuf
  // radio_noise
  // radio_remnoise
  // radio_rxerrors
  // radio_link_alive
  // radio_last_status
}

bool
swarm_msgs__msg__PeerStates__are_equal(const swarm_msgs__msg__PeerStates * lhs, const swarm_msgs__msg__PeerStates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->header_stamp), &(rhs->header_stamp)))
  {
    return false;
  }
  // peers
  if (!swarm_msgs__msg__PeerState__Sequence__are_equal(
      &(lhs->peers), &(rhs->peers)))
  {
    return false;
  }
  // peer_jetson_alive
  if (lhs->peer_jetson_alive != rhs->peer_jetson_alive) {
    return false;
  }
  // peer_last_heartbeat
  if (lhs->peer_last_heartbeat != rhs->peer_last_heartbeat) {
    return false;
  }
  // own_battery_voltage
  if (lhs->own_battery_voltage != rhs->own_battery_voltage) {
    return false;
  }
  // own_battery_current
  if (lhs->own_battery_current != rhs->own_battery_current) {
    return false;
  }
  // own_battery_remaining
  if (lhs->own_battery_remaining != rhs->own_battery_remaining) {
    return false;
  }
  // radio_rssi
  if (lhs->radio_rssi != rhs->radio_rssi) {
    return false;
  }
  // radio_remrssi
  if (lhs->radio_remrssi != rhs->radio_remrssi) {
    return false;
  }
  // radio_txbuf
  if (lhs->radio_txbuf != rhs->radio_txbuf) {
    return false;
  }
  // radio_noise
  if (lhs->radio_noise != rhs->radio_noise) {
    return false;
  }
  // radio_remnoise
  if (lhs->radio_remnoise != rhs->radio_remnoise) {
    return false;
  }
  // radio_rxerrors
  if (lhs->radio_rxerrors != rhs->radio_rxerrors) {
    return false;
  }
  // radio_link_alive
  if (lhs->radio_link_alive != rhs->radio_link_alive) {
    return false;
  }
  // radio_last_status
  if (lhs->radio_last_status != rhs->radio_last_status) {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__PeerStates__copy(
  const swarm_msgs__msg__PeerStates * input,
  swarm_msgs__msg__PeerStates * output)
{
  if (!input || !output) {
    return false;
  }
  // header_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->header_stamp), &(output->header_stamp)))
  {
    return false;
  }
  // peers
  if (!swarm_msgs__msg__PeerState__Sequence__copy(
      &(input->peers), &(output->peers)))
  {
    return false;
  }
  // peer_jetson_alive
  output->peer_jetson_alive = input->peer_jetson_alive;
  // peer_last_heartbeat
  output->peer_last_heartbeat = input->peer_last_heartbeat;
  // own_battery_voltage
  output->own_battery_voltage = input->own_battery_voltage;
  // own_battery_current
  output->own_battery_current = input->own_battery_current;
  // own_battery_remaining
  output->own_battery_remaining = input->own_battery_remaining;
  // radio_rssi
  output->radio_rssi = input->radio_rssi;
  // radio_remrssi
  output->radio_remrssi = input->radio_remrssi;
  // radio_txbuf
  output->radio_txbuf = input->radio_txbuf;
  // radio_noise
  output->radio_noise = input->radio_noise;
  // radio_remnoise
  output->radio_remnoise = input->radio_remnoise;
  // radio_rxerrors
  output->radio_rxerrors = input->radio_rxerrors;
  // radio_link_alive
  output->radio_link_alive = input->radio_link_alive;
  // radio_last_status
  output->radio_last_status = input->radio_last_status;
  return true;
}

swarm_msgs__msg__PeerStates *
swarm_msgs__msg__PeerStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerStates * msg = (swarm_msgs__msg__PeerStates *)allocator.allocate(sizeof(swarm_msgs__msg__PeerStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__PeerStates));
  bool success = swarm_msgs__msg__PeerStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__PeerStates__destroy(swarm_msgs__msg__PeerStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__PeerStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__PeerStates__Sequence__init(swarm_msgs__msg__PeerStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerStates * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__PeerStates *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__PeerStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__PeerStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__PeerStates__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
swarm_msgs__msg__PeerStates__Sequence__fini(swarm_msgs__msg__PeerStates__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      swarm_msgs__msg__PeerStates__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

swarm_msgs__msg__PeerStates__Sequence *
swarm_msgs__msg__PeerStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerStates__Sequence * array = (swarm_msgs__msg__PeerStates__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__PeerStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__PeerStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__PeerStates__Sequence__destroy(swarm_msgs__msg__PeerStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__PeerStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__PeerStates__Sequence__are_equal(const swarm_msgs__msg__PeerStates__Sequence * lhs, const swarm_msgs__msg__PeerStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__PeerStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__PeerStates__Sequence__copy(
  const swarm_msgs__msg__PeerStates__Sequence * input,
  swarm_msgs__msg__PeerStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__PeerStates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__PeerStates * data =
      (swarm_msgs__msg__PeerStates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__PeerStates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__PeerStates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__PeerStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
