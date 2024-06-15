// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Correspondences.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'correspondences'
#include "custom_msgs/msg/detail/plane_match__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Correspondences __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Correspondences __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Correspondences_
{
  using Type = Correspondences_<ContainerAllocator>;

  explicit Correspondences_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_label = "";
      this->second_label = "";
    }
  }

  explicit Correspondences_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : first_label(_alloc),
    second_label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->first_label = "";
      this->second_label = "";
    }
  }

  // field types and members
  using _correspondences_type =
    std::vector<custom_msgs::msg::PlaneMatch_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::PlaneMatch_<ContainerAllocator>>>;
  _correspondences_type correspondences;
  using _first_label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _first_label_type first_label;
  using _second_label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _second_label_type second_label;

  // setters for named parameter idiom
  Type & set__correspondences(
    const std::vector<custom_msgs::msg::PlaneMatch_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_msgs::msg::PlaneMatch_<ContainerAllocator>>> & _arg)
  {
    this->correspondences = _arg;
    return *this;
  }
  Type & set__first_label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->first_label = _arg;
    return *this;
  }
  Type & set__second_label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->second_label = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Correspondences_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Correspondences_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Correspondences_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Correspondences_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Correspondences
    std::shared_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Correspondences
    std::shared_ptr<custom_msgs::msg::Correspondences_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Correspondences_ & other) const
  {
    if (this->correspondences != other.correspondences) {
      return false;
    }
    if (this->first_label != other.first_label) {
      return false;
    }
    if (this->second_label != other.second_label) {
      return false;
    }
    return true;
  }
  bool operator!=(const Correspondences_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Correspondences_

// alias to use template instance with default allocator
using Correspondences =
  custom_msgs::msg::Correspondences_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CORRESPONDENCES__STRUCT_HPP_
