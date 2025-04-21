// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cafe_bot:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef CAFE_BOT__MSG__DETAIL__ORDER__TRAITS_HPP_
#define CAFE_BOT__MSG__DETAIL__ORDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cafe_bot/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cafe_bot
{

namespace msg
{

inline void to_flow_style_yaml(
  const Order & msg,
  std::ostream & out)
{
  out << "{";
  // member: table
  {
    out << "table: ";
    rosidl_generator_traits::value_to_yaml(msg.table, out);
    out << ", ";
  }

  // member: item
  {
    out << "item: ";
    rosidl_generator_traits::value_to_yaml(msg.item, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: table
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "table: ";
    rosidl_generator_traits::value_to_yaml(msg.table, out);
    out << "\n";
  }

  // member: item
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "item: ";
    rosidl_generator_traits::value_to_yaml(msg.item, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Order & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cafe_bot

namespace rosidl_generator_traits
{

[[deprecated("use cafe_bot::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cafe_bot::msg::Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  cafe_bot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cafe_bot::msg::to_yaml() instead")]]
inline std::string to_yaml(const cafe_bot::msg::Order & msg)
{
  return cafe_bot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cafe_bot::msg::Order>()
{
  return "cafe_bot::msg::Order";
}

template<>
inline const char * name<cafe_bot::msg::Order>()
{
  return "cafe_bot/msg/Order";
}

template<>
struct has_fixed_size<cafe_bot::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cafe_bot::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cafe_bot::msg::Order>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CAFE_BOT__MSG__DETAIL__ORDER__TRAITS_HPP_
