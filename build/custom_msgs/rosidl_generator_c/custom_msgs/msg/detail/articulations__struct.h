// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Articulations.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coordinates'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Articulations in the package custom_msgs.
typedef struct custom_msgs__msg__Articulations
{
  geometry_msgs__msg__Vector3 coordinates;
  float visibility;
  int32_t id;
} custom_msgs__msg__Articulations;

// Struct for a sequence of custom_msgs__msg__Articulations.
typedef struct custom_msgs__msg__Articulations__Sequence
{
  custom_msgs__msg__Articulations * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Articulations__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_H_
