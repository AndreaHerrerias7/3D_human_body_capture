// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/PointsCamera.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'points1'
// Member 'points2'
#include "custom_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__PointsCamera __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__PointsCamera __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointsCamera_
{
  using Type = PointsCamera_<ContainerAllocator>;

  explicit PointsCamera_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PointsCamera_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points1_type =
    std::vector<custom_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::Point_<ContainerAllocator>>>;
  _points1_type points1;
  using _points2_type =
    std::vector<custom_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::Point_<ContainerAllocator>>>;
  _points2_type points2;

  // setters for named parameter idiom
  Type & set__points1(
    const std::vector<custom_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points1 = _arg;
    return *this;
  }
  Type & set__points2(
    const std::vector<custom_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::PointsCamera_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::PointsCamera_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PointsCamera_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PointsCamera_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__PointsCamera
    std::shared_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__PointsCamera
    std::shared_ptr<custom_msgs::msg::PointsCamera_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointsCamera_ & other) const
  {
    if (this->points1 != other.points1) {
      return false;
    }
    if (this->points2 != other.points2) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointsCamera_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointsCamera_

// alias to use template instance with default allocator
using PointsCamera =
  custom_msgs::msg::PointsCamera_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__POINTS_CAMERA__STRUCT_HPP_
