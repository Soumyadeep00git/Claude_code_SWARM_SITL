// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/swarm_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
swarm_msgs__msg__SwarmCommand__init(swarm_msgs__msg__SwarmCommand * msg)
{
  if (!msg) {
    return false;
  }
  // cmd
  // vn
  // ve
  // vd
  // lat
  // lon
  // speed
  return true;
}

void
swarm_msgs__msg__SwarmCommand__fini(swarm_msgs__msg__SwarmCommand * msg)
{
  if (!msg) {
    return;
  }
  // cmd
  // vn
  // ve
  // vd
  // lat
  // lon
  // speed
}

bool
swarm_msgs__msg__SwarmCommand__are_equal(const swarm_msgs__msg__SwarmCommand * lhs, const swarm_msgs__msg__SwarmCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd
  if (lhs->cmd != rhs->cmd) {
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
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__SwarmCommand__copy(
  const swarm_msgs__msg__SwarmCommand * input,
  swarm_msgs__msg__SwarmCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd
  output->cmd = input->cmd;
  // vn
  output->vn = input->vn;
  // ve
  output->ve = input->ve;
  // vd
  output->vd = input->vd;
  // lat
  output->lat = input->lat;
  // lon
  output->lon = input->lon;
  // speed
  output->speed = input->speed;
  return true;
}

swarm_msgs__msg__SwarmCommand *
swarm_msgs__msg__SwarmCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__SwarmCommand * msg = (swarm_msgs__msg__SwarmCommand *)allocator.allocate(sizeof(swarm_msgs__msg__SwarmCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__SwarmCommand));
  bool success = swarm_msgs__msg__SwarmCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__SwarmCommand__destroy(swarm_msgs__msg__SwarmCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__SwarmCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__SwarmCommand__Sequence__init(swarm_msgs__msg__SwarmCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__SwarmCommand * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__SwarmCommand *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__SwarmCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__SwarmCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__SwarmCommand__fini(&data[i - 1]);
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
swarm_msgs__msg__SwarmCommand__Sequence__fini(swarm_msgs__msg__SwarmCommand__Sequence * array)
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
      swarm_msgs__msg__SwarmCommand__fini(&array->data[i]);
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

swarm_msgs__msg__SwarmCommand__Sequence *
swarm_msgs__msg__SwarmCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__SwarmCommand__Sequence * array = (swarm_msgs__msg__SwarmCommand__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__SwarmCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__SwarmCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__SwarmCommand__Sequence__destroy(swarm_msgs__msg__SwarmCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__SwarmCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__SwarmCommand__Sequence__are_equal(const swarm_msgs__msg__SwarmCommand__Sequence * lhs, const swarm_msgs__msg__SwarmCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__SwarmCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__SwarmCommand__Sequence__copy(
  const swarm_msgs__msg__SwarmCommand__Sequence * input,
  swarm_msgs__msg__SwarmCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__SwarmCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__SwarmCommand * data =
      (swarm_msgs__msg__SwarmCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__SwarmCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__SwarmCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__SwarmCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
