// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from custom_msgs:msg/Correspondences.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/correspondences__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "custom_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom_msgs/msg/detail/correspondences__struct.h"
#include "custom_msgs/msg/detail/correspondences__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "custom_msgs/msg/detail/plane_match__functions.h"  // correspondences
#include "rosidl_runtime_c/string.h"  // first_label, second_label
#include "rosidl_runtime_c/string_functions.h"  // first_label, second_label

// forward declare type support functions
size_t get_serialized_size_custom_msgs__msg__PlaneMatch(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_custom_msgs__msg__PlaneMatch(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_msgs, msg, PlaneMatch)();


using _Correspondences__ros_msg_type = custom_msgs__msg__Correspondences;

static bool _Correspondences__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Correspondences__ros_msg_type * ros_message = static_cast<const _Correspondences__ros_msg_type *>(untyped_ros_message);
  // Field name: correspondences
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom_msgs, msg, PlaneMatch
      )()->data);
    size_t size = ros_message->correspondences.size;
    auto array_ptr = ros_message->correspondences.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: first_label
  {
    const rosidl_runtime_c__String * str = &ros_message->first_label;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: second_label
  {
    const rosidl_runtime_c__String * str = &ros_message->second_label;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Correspondences__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Correspondences__ros_msg_type * ros_message = static_cast<_Correspondences__ros_msg_type *>(untyped_ros_message);
  // Field name: correspondences
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, custom_msgs, msg, PlaneMatch
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->correspondences.data) {
      custom_msgs__msg__PlaneMatch__Sequence__fini(&ros_message->correspondences);
    }
    if (!custom_msgs__msg__PlaneMatch__Sequence__init(&ros_message->correspondences, size)) {
      fprintf(stderr, "failed to create array for field 'correspondences'");
      return false;
    }
    auto array_ptr = ros_message->correspondences.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: first_label
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->first_label.data) {
      rosidl_runtime_c__String__init(&ros_message->first_label);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->first_label,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'first_label'\n");
      return false;
    }
  }

  // Field name: second_label
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->second_label.data) {
      rosidl_runtime_c__String__init(&ros_message->second_label);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->second_label,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'second_label'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_msgs
size_t get_serialized_size_custom_msgs__msg__Correspondences(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Correspondences__ros_msg_type * ros_message = static_cast<const _Correspondences__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name correspondences
  {
    size_t array_size = ros_message->correspondences.size;
    auto array_ptr = ros_message->correspondences.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_custom_msgs__msg__PlaneMatch(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name first_label
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->first_label.size + 1);
  // field.name second_label
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->second_label.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Correspondences__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_custom_msgs__msg__Correspondences(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_msgs
size_t max_serialized_size_custom_msgs__msg__Correspondences(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: correspondences
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_custom_msgs__msg__PlaneMatch(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: first_label
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: second_label
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom_msgs__msg__Correspondences;
    is_plain =
      (
      offsetof(DataType, second_label) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Correspondences__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_custom_msgs__msg__Correspondences(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Correspondences = {
  "custom_msgs::msg",
  "Correspondences",
  _Correspondences__cdr_serialize,
  _Correspondences__cdr_deserialize,
  _Correspondences__get_serialized_size,
  _Correspondences__max_serialized_size
};

static rosidl_message_type_support_t _Correspondences__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Correspondences,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_msgs, msg, Correspondences)() {
  return &_Correspondences__type_support;
}

#if defined(__cplusplus)
}
#endif
