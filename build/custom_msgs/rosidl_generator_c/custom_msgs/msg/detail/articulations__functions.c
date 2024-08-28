// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/Articulations.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/articulations__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `coordinates`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
custom_msgs__msg__Articulations__init(custom_msgs__msg__Articulations * msg)
{
  if (!msg) {
    return false;
  }
  // coordinates
  if (!geometry_msgs__msg__Vector3__init(&msg->coordinates)) {
    custom_msgs__msg__Articulations__fini(msg);
    return false;
  }
  // visibility
  // id
  return true;
}

void
custom_msgs__msg__Articulations__fini(custom_msgs__msg__Articulations * msg)
{
  if (!msg) {
    return;
  }
  // coordinates
  geometry_msgs__msg__Vector3__fini(&msg->coordinates);
  // visibility
  // id
}

bool
custom_msgs__msg__Articulations__are_equal(const custom_msgs__msg__Articulations * lhs, const custom_msgs__msg__Articulations * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // coordinates
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->coordinates), &(rhs->coordinates)))
  {
    return false;
  }
  // visibility
  if (lhs->visibility != rhs->visibility) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__Articulations__copy(
  const custom_msgs__msg__Articulations * input,
  custom_msgs__msg__Articulations * output)
{
  if (!input || !output) {
    return false;
  }
  // coordinates
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->coordinates), &(output->coordinates)))
  {
    return false;
  }
  // visibility
  output->visibility = input->visibility;
  // id
  output->id = input->id;
  return true;
}

custom_msgs__msg__Articulations *
custom_msgs__msg__Articulations__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Articulations * msg = (custom_msgs__msg__Articulations *)allocator.allocate(sizeof(custom_msgs__msg__Articulations), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__Articulations));
  bool success = custom_msgs__msg__Articulations__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__Articulations__destroy(custom_msgs__msg__Articulations * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__Articulations__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__Articulations__Sequence__init(custom_msgs__msg__Articulations__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Articulations * data = NULL;

  if (size) {
    data = (custom_msgs__msg__Articulations *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__Articulations), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__Articulations__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__Articulations__fini(&data[i - 1]);
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
custom_msgs__msg__Articulations__Sequence__fini(custom_msgs__msg__Articulations__Sequence * array)
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
      custom_msgs__msg__Articulations__fini(&array->data[i]);
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

custom_msgs__msg__Articulations__Sequence *
custom_msgs__msg__Articulations__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Articulations__Sequence * array = (custom_msgs__msg__Articulations__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__Articulations__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__Articulations__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__Articulations__Sequence__destroy(custom_msgs__msg__Articulations__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__Articulations__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__Articulations__Sequence__are_equal(const custom_msgs__msg__Articulations__Sequence * lhs, const custom_msgs__msg__Articulations__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__Articulations__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__Articulations__Sequence__copy(
  const custom_msgs__msg__Articulations__Sequence * input,
  custom_msgs__msg__Articulations__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__Articulations);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__Articulations * data =
      (custom_msgs__msg__Articulations *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__Articulations__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__Articulations__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__Articulations__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
