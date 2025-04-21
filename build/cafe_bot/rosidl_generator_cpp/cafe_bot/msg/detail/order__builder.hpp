// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cafe_bot:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef CAFE_BOT__MSG__DETAIL__ORDER__BUILDER_HPP_
#define CAFE_BOT__MSG__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cafe_bot/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cafe_bot
{

namespace msg
{

namespace builder
{

class Init_Order_item
{
public:
  explicit Init_Order_item(::cafe_bot::msg::Order & msg)
  : msg_(msg)
  {}
  ::cafe_bot::msg::Order item(::cafe_bot::msg::Order::_item_type arg)
  {
    msg_.item = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cafe_bot::msg::Order msg_;
};

class Init_Order_table
{
public:
  Init_Order_table()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_item table(::cafe_bot::msg::Order::_table_type arg)
  {
    msg_.table = std::move(arg);
    return Init_Order_item(msg_);
  }

private:
  ::cafe_bot::msg::Order msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cafe_bot::msg::Order>()
{
  return cafe_bot::msg::builder::Init_Order_table();
}

}  // namespace cafe_bot

#endif  // CAFE_BOT__MSG__DETAIL__ORDER__BUILDER_HPP_
