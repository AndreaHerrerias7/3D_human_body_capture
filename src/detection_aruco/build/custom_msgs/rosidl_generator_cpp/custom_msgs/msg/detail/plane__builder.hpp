// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Plane.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/plane__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Plane_d
{
public:
  explicit Init_Plane_d(::custom_msgs::msg::Plane & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Plane d(::custom_msgs::msg::Plane::_d_type arg)
  {
    msg_.d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Plane msg_;
};

class Init_Plane_n
{
public:
  Init_Plane_n()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Plane_d n(::custom_msgs::msg::Plane::_n_type arg)
  {
    msg_.n = std::move(arg);
    return Init_Plane_d(msg_);
  }

private:
  ::custom_msgs::msg::Plane msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Plane>()
{
  return custom_msgs::msg::builder::Init_Plane_n();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE__BUILDER_HPP_
