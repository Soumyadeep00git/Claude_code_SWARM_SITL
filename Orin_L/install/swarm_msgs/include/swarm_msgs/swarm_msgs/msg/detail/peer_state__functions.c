// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/peer_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
swarm_msgs__msg__PeerState__init(swarm_msgs__msg__PeerState * msg)
{
  if (!msg) {
    return false;
  }
  // drone_id
  // latitude
  // longitude
  // altitude
  // vn
  // ve
  // vd
  // heading
  // stamp
  // guidance_mode
  // fc_mode_code
  // fc_armed
  // fc_connected
  // battery_voltage
  // battery_current
  // battery_remaining
  return true;
}

void
swarm_msgs__msg__PeerState__fini(swarm_msgs__msg__PeerState * msg)
{
  if (!msg) {
    return;
  }
  // drone_id
  // latitude
  // longitude
  // altitude
  // vn
  // ve
  // vd
  // heading
  // stamp
  // guidance_mode
  // fc_mode_code
  // fc_armed
  // fc_connected
  // battery_voltage
  // battery_current
  // battery_remaining
}

bool
swarm_msgs__msg__PeerState__are_equal(const swarm_msgs__msg__PeerState * lhs, const swarm_msgs__msg__PeerState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drone_id
  if (lhs->drone_id != rhs->drone_id) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // vn
  if (lhs->vn != rhs->vn) {
    return false;
  }
  // ve
  if (lhs->ve != rhs->ve) {
    return false;
  }
  // vd
  if (lhs->vd != rhs->vd) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // stamp
  if (lhs->stamp != rhs->stamp) {
    return false;
  }
  // guidance_mode
  if (lhs->guidance_mode != rhs->guidance_mode) {
    return false;
  }
  // fc_mode_code
  if (lhs->fc_mode_code != rhs->fc_mode_code) {
    return false;
  }
  // fc_armed
  if (lhs->fc_armed != rhs->fc_armed) {
    return false;
  }
  // fc_connected
  if (lhs->fc_connected != rhs->fc_connected) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // battery_current
  if (lhs->battery_current != rhs->battery_current) {
    return false;
  }
  // battery_remaining
  if (lhs->battery_remaining != rhs->battery_remaining) {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__PeerState__copy(
  const swarm_msgs__msg__PeerState * input,
  swarm_msgs__msg__PeerState * output)
{
  if (!input || !output) {
    return false;
  }
  // drone_id
  output->drone_id = input->drone_id;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // vn
  output->vn = input->vn;
  // ve
  output->ve = input->ve;
  // vd
  output->vd = input->vd;
  // heading
  output->heading = input->heading;
  // stamp
  output->stamp = input->stamp;
  // guidance_mode
  output->guidance_mode = input->guidance_mode;
  // fc_mode_code
  output->fc_mode_code = input->fc_mode_code;
  // fc_armed
  output->fc_armed = input->fc_armed;
  // fc_connected
  output->fc_connected = input->fc_connected;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // battery_current
  output->battery_current = input->battery_current;
  // battery_remaining
  output->battery_remaining = input->battery_remaining;
  return true;
}

swarm_msgs__msg__PeerState *
swarm_msgs__msg__PeerState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerState * msg = (swarm_msgs__msg__PeerState *)allocator.allocate(sizeof(swarm_msgs__msg__PeerState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__PeerState));
  bool success = swarm_msgs__msg__PeerState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__PeerState__destroy(swarm_msgs__msg__PeerState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__PeerState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__PeerState__Sequence__init(swarm_msgs__msg__PeerState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerState * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__PeerState *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__PeerState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__PeerState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__PeerState__fini(&data[i - 1]);
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
swarm_msgs__msg__PeerState__Sequence__fini(swarm_msgs__msg__PeerState__Sequence * array)
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
      swarm_msgs__msg__PeerState__fini(&array->data[i]);
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

swarm_msgs__msg__PeerState__Sequence *
swarm_msgs__msg__PeerState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__PeerState__Sequence * array = (swarm_msgs__msg__PeerState__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__PeerState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__PeerState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__PeerState__Sequence__destroy(swarm_msgs__msg__PeerState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__PeerState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__PeerState__Sequence__are_equal(const swarm_msgs__msg__PeerState__Sequence * lhs, const swarm_msgs__msg__PeerState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__PeerState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__PeerState__Sequence__copy(
  const swarm_msgs__msg__PeerState__Sequence * input,
  swarm_msgs__msg__PeerState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__PeerState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__PeerState * data =
      (swarm_msgs__msg__PeerState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__PeerState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__PeerState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__PeerState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
