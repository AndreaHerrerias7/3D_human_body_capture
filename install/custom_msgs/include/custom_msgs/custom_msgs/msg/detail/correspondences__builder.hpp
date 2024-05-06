// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Correspondences.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/correspondences__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Correspondences_correspondences
{
public:
  Init_Correspondences_correspondences()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_msgs::msg::Correspondences correspondences(::custom_msgs::msg::Correspondences::_correspondences_type arg)
  {
    msg_.correspondences = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Correspondences msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Correspondences>()
{
  return custom_msgs::msg::builder::Init_Correspondences_correspondences();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__BUILDER_HPP_
