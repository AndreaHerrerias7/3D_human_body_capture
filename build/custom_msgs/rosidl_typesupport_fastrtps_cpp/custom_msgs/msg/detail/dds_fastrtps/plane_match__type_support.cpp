// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from custom_msgs:msg/PlaneMatch.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/plane_match__rosidl_typesupport_fastrtps_cpp.hpp"
#include "custom_msgs/msg/detail/plane_match__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace custom_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const custom_msgs::msg::Plane &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  custom_msgs::msg::Plane &);
size_t get_serialized_size(
  const custom_msgs::msg::Plane &,
  size_t current_alignment);
size_t
max_serialized_size_Plane(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace custom_msgs

// functions for custom_msgs::msg::Plane already declared above


namespace custom_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_msgs
cdr_serialize(
  const custom_msgs::msg::PlaneMatch & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: first
  custom_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.first,
    cdr);
  // Member: second
  custom_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.second,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  custom_msgs::msg::PlaneMatch & ros_message)
{
  // Member: first
  custom_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.first);

  // Member: second
  custom_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.second);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_msgs
get_serialized_size(
  const custom_msgs::msg::PlaneMatch & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: first

  current_alignment +=
    custom_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.first, current_alignment);
  // Member: second

  current_alignment +=
    custom_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.second, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_msgs
max_serialized_size_PlaneMatch(
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


  // Member: first
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        custom_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Plane(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: second
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        custom_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Plane(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom_msgs::msg::PlaneMatch;
    is_plain =
      (
      offsetof(DataType, second) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PlaneMatch__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const custom_msgs::msg::PlaneMatch *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PlaneMatch__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<custom_msgs::msg::PlaneMatch *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PlaneMatch__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const custom_msgs::msg::PlaneMatch *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PlaneMatch__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PlaneMatch(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PlaneMatch__callbacks = {
  "custom_msgs::msg",
  "PlaneMatch",
  _PlaneMatch__cdr_serialize,
  _PlaneMatch__cdr_deserialize,
  _PlaneMatch__get_serialized_size,
  _PlaneMatch__max_serialized_size
};

static rosidl_message_type_support_t _PlaneMatch__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PlaneMatch__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_custom_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_msgs::msg::PlaneMatch>()
{
  return &custom_msgs::msg::typesupport_fastrtps_cpp::_PlaneMatch__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_msgs, msg, PlaneMatch)() {
  return &custom_msgs::msg::typesupport_fastrtps_cpp::_PlaneMatch__handle;
}

#ifdef __cplusplus
}
#endif
