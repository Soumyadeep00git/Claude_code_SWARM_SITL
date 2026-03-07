// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from swarm_msgs:msg/SwarmCommand.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__FUNCTIONS_H_
#define SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "swarm_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "swarm_msgs/msg/detail/swarm_command__struct.h"

/// Initialize msg/SwarmCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * swarm_msgs__msg__SwarmCommand
 * )) before or use
 * swarm_msgs__msg__SwarmCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__init(swarm_msgs__msg__SwarmCommand * msg);

/// Finalize msg/SwarmCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
void
swarm_msgs__msg__SwarmCommand__fini(swarm_msgs__msg__SwarmCommand * msg);

/// Create msg/SwarmCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * swarm_msgs__msg__SwarmCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
swarm_msgs__msg__SwarmCommand *
swarm_msgs__msg__SwarmCommand__create();

/// Destroy msg/SwarmCommand message.
/**
 * It calls
 * swarm_msgs__msg__SwarmCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
void
swarm_msgs__msg__SwarmCommand__destroy(swarm_msgs__msg__SwarmCommand * msg);

/// Check for msg/SwarmCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__are_equal(const swarm_msgs__msg__SwarmCommand * lhs, const swarm_msgs__msg__SwarmCommand * rhs);

/// Copy a msg/SwarmCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__copy(
  const swarm_msgs__msg__SwarmCommand * input,
  swarm_msgs__msg__SwarmCommand * output);

/// Initialize array of msg/SwarmCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * swarm_msgs__msg__SwarmCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__Sequence__init(swarm_msgs__msg__SwarmCommand__Sequence * array, size_t size);

/// Finalize array of msg/SwarmCommand messages.
/**
 * It calls
 * swarm_msgs__msg__SwarmCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
void
swarm_msgs__msg__SwarmCommand__Sequence__fini(swarm_msgs__msg__SwarmCommand__Sequence * array);

/// Create array of msg/SwarmCommand messages.
/**
 * It allocates the memory for the array and calls
 * swarm_msgs__msg__SwarmCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
swarm_msgs__msg__SwarmCommand__Sequence *
swarm_msgs__msg__SwarmCommand__Sequence__create(size_t size);

/// Destroy array of msg/SwarmCommand messages.
/**
 * It calls
 * swarm_msgs__msg__SwarmCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
void
swarm_msgs__msg__SwarmCommand__Sequence__destroy(swarm_msgs__msg__SwarmCommand__Sequence * array);

/// Check for msg/SwarmCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__Sequence__are_equal(const swarm_msgs__msg__SwarmCommand__Sequence * lhs, const swarm_msgs__msg__SwarmCommand__Sequence * rhs);

/// Copy an array of msg/SwarmCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarm_msgs
bool
swarm_msgs__msg__SwarmCommand__Sequence__copy(
  const swarm_msgs__msg__SwarmCommand__Sequence * input,
  swarm_msgs__msg__SwarmCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__SWARM_COMMAND__FUNCTIONS_H_
