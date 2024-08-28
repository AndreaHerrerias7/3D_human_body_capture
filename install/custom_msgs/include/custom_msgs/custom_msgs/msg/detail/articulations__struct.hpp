// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Articulations.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'coordinates'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Articulations __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Articulations __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Articulations_
{
  using Type = Articulations_<ContainerAllocator>;

  explicit Articulations_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coordinates(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visibility = 0.0f;
      this->id = 0l;
    }
  }

  explicit Articulations_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coordinates(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visibility = 0.0f;
      this->id = 0l;
    }
  }

  // field types and members
  using _coordinates_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _coordinates_type coordinates;
  using _visibility_type =
    float;
  _visibility_type visibility;
  using _id_type =
    int32_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__coordinates(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->coordinates = _arg;
    return *this;
  }
  Type & set__visibility(
    const float & _arg)
  {
    this->visibility = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Articulations_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Articulations_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Articulations_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Articulations_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Articulations_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Articulations_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Articulations_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Articulations_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Articulations_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Articulations_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Articulations
    std::shared_ptr<custom_msgs::msg::Articulations_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Articulations
    std::shared_ptr<custom_msgs::msg::Articulations_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Articulations_ & other) const
  {
    if (this->coordinates != other.coordinates) {
      return false;
    }
    if (this->visibility != other.visibility) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Articulations_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Articulations_

// alias to use template instance with default allocator
using Articulations =
  custom_msgs::msg::Articulations_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ARTICULATIONS__STRUCT_HPP_
