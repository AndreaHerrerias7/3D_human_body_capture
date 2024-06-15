// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Correspondences.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'correspondences'
#include "custom_msgs/msg/detail/plane_match__struct.h"
// Member 'first_label'
// Member 'second_label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Correspondences in the package custom_msgs.
typedef struct custom_msgs__msg__Correspondences
{
  custom_msgs__msg__PlaneMatch__Sequence correspondences;
  rosidl_runtime_c__String first_label;
  rosidl_runtime_c__String second_label;
} custom_msgs__msg__Correspondences;

// Struct for a sequence of custom_msgs__msg__Correspondences.
typedef struct custom_msgs__msg__Correspondences__Sequence
{
  custom_msgs__msg__Correspondences * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Correspondences__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_H_
