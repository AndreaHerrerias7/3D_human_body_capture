// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/PointsCamera.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/points_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_PointsCamera_points2
{
public:
  explicit Init_PointsCamera_points2(::custom_msgs::msg::PointsCamera & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::PointsCamera points2(::custom_msgs::msg::PointsCamera::_points2_type arg)
  {
    msg_.points2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::PointsCamera msg_;
};

class Init_PointsCamera_points1
{
public:
  Init_PointsCamera_points1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PointsCamera_points2 points1(::custom_msgs::msg::PointsCamera::_points1_type arg)
  {
    msg_.points1 = std::move(arg);
    return Init_PointsCamera_points2(msg_);
  }

private:
  ::custom_msgs::msg::PointsCamera msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::PointsCamera>()
{
  return custom_msgs::msg::builder::Init_PointsCamera_points1();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__BUILDER_HPP_
