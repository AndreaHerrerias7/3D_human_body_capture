// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Articulations.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/articulations__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Articulations_id
{
public:
  explicit Init_Articulations_id(::custom_msgs::msg::Articulations & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Articulations id(::custom_msgs::msg::Articulations::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Articulations msg_;
};

class Init_Articulations_visibility
{
public:
  explicit Init_Articulations_visibility(::custom_msgs::msg::Articulations & msg)
  : msg_(msg)
  {}
  Init_Articulations_id visibility(::custom_msgs::msg::Articulations::_visibility_type arg)
  {
    msg_.visibility = std::move(arg);
    return Init_Articulations_id(msg_);
  }

private:
  ::custom_msgs::msg::Articulations msg_;
};

class Init_Articulations_coordinates
{
public:
  Init_Articulations_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Articulations_visibility coordinates(::custom_msgs::msg::Articulations::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return Init_Articulations_visibility(msg_);
  }

private:
  ::custom_msgs::msg::Articulations msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Articulations>()
{
  return custom_msgs::msg::builder::Init_Articulations_coordinates();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__BUILDER_HPP_
