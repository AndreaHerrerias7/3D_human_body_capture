// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/PointsCamera.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'points1'
// Member 'points2'
#include "custom_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/PointsCamera in the package custom_msgs.
typedef struct custom_msgs__msg__PointsCamera
{
  custom_msgs__msg__Point__Sequence points1;
  custom_msgs__msg__Point__Sequence points2;
} custom_msgs__msg__PointsCamera;

// Struct for a sequence of custom_msgs__msg__PointsCamera.
typedef struct custom_msgs__msg__PointsCamera__Sequence
{
  custom_msgs__msg__PointsCamera * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__PointsCamera__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_H_
