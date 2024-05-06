// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from datos_camara:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef DATOS_CAMARA__MSG__DETAIL__NUM__TRAITS_HPP_
#define DATOS_CAMARA__MSG__DETAIL__NUM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "datos_camara/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vect_n'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace datos_camara
{

namespace msg
{

inline void to_flow_style_yaml(
  const Num & msg,
  std::ostream & out)
{
  out << "{";
  // member: num
  {
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << ", ";
  }

  // member: vect_n
  {
    out << "vect_n: ";
    to_flow_style_yaml(msg.vect_n, out);
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
  const Num & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << "\n";
  }

  // member: vect_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vect_n:\n";
    to_block_style_yaml(msg.vect_n, out, indentation + 2);
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

inline std::string to_yaml(const Num & msg, bool use_flow_style = false)
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

}  // namespace datos_camara

namespace rosidl_generator_traits
{

[[deprecated("use datos_camara::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const datos_camara::msg::Num & msg,
  std::ostream & out, size_t indentation = 0)
{
  datos_camara::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use datos_camara::msg::to_yaml() instead")]]
inline std::string to_yaml(const datos_camara::msg::Num & msg)
{
  return datos_camara::msg::to_yaml(msg);
}

template<>
inline const char * data_type<datos_camara::msg::Num>()
{
  return "datos_camara::msg::Num";
}

template<>
inline const char * name<datos_camara::msg::Num>()
{
  return "datos_camara/msg/Num";
}

template<>
struct has_fixed_size<datos_camara::msg::Num>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<datos_camara::msg::Num>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<datos_camara::msg::Num>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DATOS_CAMARA__MSG__DETAIL__NUM__TRAITS_HPP_
