// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Articulations.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/articulations__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'coordinates'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Articulations & msg,
  std::ostream & out)
{
  out << "{";
  // member: coordinates
  {
    out << "coordinates: ";
    to_flow_style_yaml(msg.coordinates, out);
    out << ", ";
  }

  // member: visibility
  {
    out << "visibility: ";
    rosidl_generator_traits::value_to_yaml(msg.visibility, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Articulations & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: coordinates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coordinates:\n";
    to_block_style_yaml(msg.coordinates, out, indentation + 2);
  }

  // member: visibility
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visibility: ";
    rosidl_generator_traits::value_to_yaml(msg.visibility, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Articulations & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::Articulations & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Articulations & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Articulations>()
{
  return "custom_msgs::msg::Articulations";
}

template<>
inline const char * name<custom_msgs::msg::Articulations>()
{
  return "custom_msgs/msg/Articulations";
}

template<>
struct has_fixed_size<custom_msgs::msg::Articulations>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::Articulations>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<custom_msgs::msg::Articulations>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__TRAITS_HPP_
