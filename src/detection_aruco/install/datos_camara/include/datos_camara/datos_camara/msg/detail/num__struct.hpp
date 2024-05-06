// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from datos_camara:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_HPP_
#define DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vect_n'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__datos_camara__msg__Num __attribute__((deprecated))
#else
# define DEPRECATED__datos_camara__msg__Num __declspec(deprecated)
#endif

namespace datos_camara
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Num_
{
  using Type = Num_<ContainerAllocator>;

  explicit Num_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vect_n(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0l;
      this->d = 0.0f;
    }
  }

  explicit Num_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vect_n(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0l;
      this->d = 0.0f;
    }
  }

  // field types and members
  using _num_type =
    int32_t;
  _num_type num;
  using _vect_n_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vect_n_type vect_n;
  using _d_type =
    float;
  _d_type d;

  // setters for named parameter idiom
  Type & set__num(
    const int32_t & _arg)
  {
    this->num = _arg;
    return *this;
  }
  Type & set__vect_n(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vect_n = _arg;
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
    datos_camara::msg::Num_<ContainerAllocator> *;
  using ConstRawPtr =
    const datos_camara::msg::Num_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<datos_camara::msg::Num_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<datos_camara::msg::Num_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      datos_camara::msg::Num_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<datos_camara::msg::Num_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      datos_camara::msg::Num_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<datos_camara::msg::Num_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<datos_camara::msg::Num_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<datos_camara::msg::Num_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__datos_camara__msg__Num
    std::shared_ptr<datos_camara::msg::Num_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__datos_camara__msg__Num
    std::shared_ptr<datos_camara::msg::Num_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Num_ & other) const
  {
    if (this->num != other.num) {
      return false;
    }
    if (this->vect_n != other.vect_n) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    return true;
  }
  bool operator!=(const Num_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Num_

// alias to use template instance with default allocator
using Num =
  datos_camara::msg::Num_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace datos_camara

#endif  // DATOS_CAMARA__MSG__DETAIL__NUM__STRUCT_HPP_
