// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/PlaneMatch.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/plane_match__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_PlaneMatch_second
{
public:
  explicit Init_PlaneMatch_second(::custom_msgs::msg::PlaneMatch & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::PlaneMatch second(::custom_msgs::msg::PlaneMatch::_second_type arg)
  {
    msg_.second = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::PlaneMatch msg_;
};

class Init_PlaneMatch_first
{
public:
  Init_PlaneMatch_first()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlaneMatch_second first(::custom_msgs::msg::PlaneMatch::_first_type arg)
  {
    msg_.first = std::move(arg);
    return Init_PlaneMatch_second(msg_);
  }

private:
  ::custom_msgs::msg::PlaneMatch msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::PlaneMatch>()
{
  return custom_msgs::msg::builder::Init_PlaneMatch_first();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__BUILDER_HPP_
