// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from datos_camara:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_H_
#define DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vect_n'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Num in the package datos_camara.
typedef struct datos_camara__msg__Num
{
  int32_t num;
  geometry_msgs__msg__Vector3 vect_n;
  float d;
} datos_camara__msg__Num;

// Struct for a sequence of datos_camara__msg__Num.
typedef struct datos_camara__msg__Num__Sequence
{
  datos_camara__msg__Num * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} datos_camara__msg__Num__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_H_
