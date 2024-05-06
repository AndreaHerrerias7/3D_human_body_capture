// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Plane.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'n'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Plane in the package custom_msgs.
typedef struct custom_msgs__msg__Plane
{
  geometry_msgs__msg__Vector3 n;
  float d;
} custom_msgs__msg__Plane;

// Struct for a sequence of custom_msgs__msg__Plane.
typedef struct custom_msgs__msg__Plane__Sequence
{
  custom_msgs__msg__Plane * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Plane__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_H_
