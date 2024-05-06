// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Plane.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'n'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Plane __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Plane __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Plane_
{
  using Type = Plane_<ContainerAllocator>;

  explicit Plane_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : n(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->d = 0.0f;
    }
  }

  explicit Plane_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : n(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->d = 0.0f;
    }
  }

  // field types and members
  using _n_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _n_type n;
  using _d_type =
    float;
  _d_type d;

  // setters for named parameter idiom
  Type & set__n(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->n = _arg;
    return *this;
  }
  Type & set__d(
    const float & _arg)
  {
    this->d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Plane_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Plane_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Plane_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Plane_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Plane_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Plane_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Plane_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Plane_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Plane_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Plane_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Plane
    std::shared_ptr<custom_msgs::msg::Plane_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Plane
    std::shared_ptr<custom_msgs::msg::Plane_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Plane_ & other) const
  {
    if (this->n != other.n) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    return true;
  }
  bool operator!=(const Plane_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Plane_

// alias to use template instance with default allocator
using Plane =
  custom_msgs::msg::Plane_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PLANE__STRUCT_HPP_
