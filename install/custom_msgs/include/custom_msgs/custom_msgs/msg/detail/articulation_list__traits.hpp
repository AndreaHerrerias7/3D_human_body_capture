// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/ArticulationList.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/articulation_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'articulations'
#include "custom_msgs/msg/detail/articulations__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArticulationList & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: articulations
  {
    if (msg.articulations.size() == 0) {
      out << "articulations: []";
    } else {
      out << "articulations: [";
      size_t pending_items = msg.articulations.size();
      for (auto item : msg.articulations) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArticulationList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: articulations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.articulations.size() == 0) {
      out << "articulations: []\n";
    } else {
      out << "articulations:\n";
      for (auto item : msg.articulations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArticulationList & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::ArticulationList & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::ArticulationList & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::ArticulationList>()
{
  return "custom_msgs::msg::ArticulationList";
}

template<>
inline const char * name<custom_msgs::msg::ArticulationList>()
{
  return "custom_msgs/msg/ArticulationList";
}

template<>
struct has_fixed_size<custom_msgs::msg::ArticulationList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::ArticulationList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::ArticulationList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__TRAITS_HPP_
