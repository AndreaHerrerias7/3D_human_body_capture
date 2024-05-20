// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/PointsCamera.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/points_camera__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'points1'
// Member 'points2'
#include "custom_msgs/msg/detail/point__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointsCamera & msg,
  std::ostream & out)
{
  out << "{";
  // member: points1
  {
    if (msg.points1.size() == 0) {
      out << "points1: []";
    } else {
      out << "points1: [";
      size_t pending_items = msg.points1.size();
      for (auto item : msg.points1) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: points2
  {
    if (msg.points2.size() == 0) {
      out << "points2: []";
    } else {
      out << "points2: [";
      size_t pending_items = msg.points2.size();
      for (auto item : msg.points2) {
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
  const PointsCamera & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points1.size() == 0) {
      out << "points1: []\n";
    } else {
      out << "points1:\n";
      for (auto item : msg.points1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: points2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points2.size() == 0) {
      out << "points2: []\n";
    } else {
      out << "points2:\n";
      for (auto item : msg.points2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointsCamera & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::PointsCamera & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::PointsCamera & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::PointsCamera>()
{
  return "custom_msgs::msg::PointsCamera";
}

template<>
inline const char * name<custom_msgs::msg::PointsCamera>()
{
  return "custom_msgs/msg/PointsCamera";
}

template<>
struct has_fixed_size<custom_msgs::msg::PointsCamera>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::PointsCamera>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::PointsCamera>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__TRAITS_HPP_
