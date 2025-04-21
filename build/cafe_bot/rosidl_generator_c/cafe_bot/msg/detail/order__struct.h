// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cafe_bot:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef CAFE_BOT__MSG__DETAIL__ORDER__STRUCT_H_
#define CAFE_BOT__MSG__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'table'
// Member 'item'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Order in the package cafe_bot.
typedef struct cafe_bot__msg__Order
{
  rosidl_runtime_c__String table;
  rosidl_runtime_c__String item;
} cafe_bot__msg__Order;

// Struct for a sequence of cafe_bot__msg__Order.
typedef struct cafe_bot__msg__Order__Sequence
{
  cafe_bot__msg__Order * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cafe_bot__msg__Order__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAFE_BOT__MSG__DETAIL__ORDER__STRUCT_H_
