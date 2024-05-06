// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from datos_camara:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef DATOS_CAMARA__MSG__DETAIL__NUM__BUILDER_HPP_
#define DATOS_CAMARA__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "datos_camara/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace datos_camara
{

namespace msg
{

namespace builder
{

class Init_Num_d
{
public:
  explicit Init_Num_d(::datos_camara::msg::Num & msg)
  : msg_(msg)
  {}
  ::datos_camara::msg::Num d(::datos_camara::msg::Num::_d_type arg)
  {
    msg_.d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::datos_camara::msg::Num msg_;
};

class Init_Num_vect_n
{
public:
  explicit Init_Num_vect_n(::datos_camara::msg::Num & msg)
  : msg_(msg)
  {}
  Init_Num_d vect_n(::datos_camara::msg::Num::_vect_n_type arg)
  {
    msg_.vect_n = std::move(arg);
    return Init_Num_d(msg_);
  }

private:
  ::datos_camara::msg::Num msg_;
};

class Init_Num_num
{
public:
  Init_Num_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Num_vect_n num(::datos_camara::msg::Num::_num_type arg)
  {
    msg_.num = std::move(arg);
    return Init_Num_vect_n(msg_);
  }

private:
  ::datos_camara::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::datos_camara::msg::Num>()
{
  return datos_camara::msg::builder::Init_Num_num();
}

}  // namespace datos_camara

#endif  // DATOS_CAMARA__MSG__DETAIL__NUM__BUILDER_HPP_
