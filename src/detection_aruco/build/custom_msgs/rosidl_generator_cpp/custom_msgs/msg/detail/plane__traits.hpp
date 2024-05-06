// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Plane.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/plane__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'n'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Plane & msg,
  std::ostream & out)
{
  out << "{";
  // member: n
  {
    out << "n: ";
    to_flow_style_yaml(msg.n, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Plane & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n:\n";
    to_block_style_yaml(msg.n, out, indentation + 2);
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Plane & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::Plane & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Plane & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Plane>()
{
  return "custom_msgs::msg::Plane";
}

template<>
inline const char * name<custom_msgs::msg::Plane>()
{
  return "custom_msgs/msg/Plane";
}

template<>
struct has_fixed_size<custom_msgs::msg::Plane>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::Plane>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<custom_msgs::msg::Plane>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE__TRAITS_HPP_
