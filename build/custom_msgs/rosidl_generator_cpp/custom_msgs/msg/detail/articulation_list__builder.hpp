// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ArticulationList.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/articulation_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ArticulationList_articulations
{
public:
  explicit Init_ArticulationList_articulations(::custom_msgs::msg::ArticulationList & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ArticulationList articulations(::custom_msgs::msg::ArticulationList::_articulations_type arg)
  {
    msg_.articulations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ArticulationList msg_;
};

class Init_ArticulationList_header
{
public:
  Init_ArticulationList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArticulationList_articulations header(::custom_msgs::msg::ArticulationList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArticulationList_articulations(msg_);
  }

private:
  ::custom_msgs::msg::ArticulationList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ArticulationList>()
{
  return custom_msgs::msg::builder::Init_ArticulationList_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATION_LIST__BUILDER_HPP_
