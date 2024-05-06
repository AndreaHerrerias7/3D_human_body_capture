// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/PlaneMatch.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'first'
// Member 'second'
#include "custom_msgs/msg/detail/plane__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__PlaneMatch __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__PlaneMatch __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlaneMatch_
{
  using Type = PlaneMatch_<ContainerAllocator>;

  explicit PlaneMatch_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : first(_init),
    second(_init)
  {
    (void)_init;
  }

  explicit PlaneMatch_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : first(_alloc, _init),
    second(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _first_type =
    custom_msgs::msg::Plane_<ContainerAllocator>;
  _first_type first;
  using _second_type =
    custom_msgs::msg::Plane_<ContainerAllocator>;
  _second_type second;

  // setters for named parameter idiom
  Type & set__first(
    const custom_msgs::msg::Plane_<ContainerAllocator> & _arg)
  {
    this->first = _arg;
    return *this;
  }
  Type & set__second(
    const custom_msgs::msg::Plane_<ContainerAllocator> & _arg)
  {
    this->second = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::PlaneMatch_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::PlaneMatch_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PlaneMatch_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PlaneMatch_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__PlaneMatch
    std::shared_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__PlaneMatch
    std::shared_ptr<custom_msgs::msg::PlaneMatch_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlaneMatch_ & other) const
  {
    if (this->first != other.first) {
      return false;
    }
    if (this->second != other.second) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlaneMatch_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlaneMatch_

// alias to use template instance with default allocator
using PlaneMatch =
  custom_msgs::msg::PlaneMatch_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE_MATCH__STRUCT_HPP_
