// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/PlaneMatch.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'first'
// Member 'second'
#include "custom_msgs/msg/detail/plane__struct.h"

/// Struct defined in msg/PlaneMatch in the package custom_msgs.
typedef struct custom_msgs__msg__PlaneMatch
{
  custom_msgs__msg__Plane first;
  custom_msgs__msg__Plane second;
} custom_msgs__msg__PlaneMatch;

// Struct for a sequence of custom_msgs__msg__PlaneMatch.
typedef struct custom_msgs__msg__PlaneMatch__Sequence
{
  custom_msgs__msg__PlaneMatch * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__PlaneMatch__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_H_
