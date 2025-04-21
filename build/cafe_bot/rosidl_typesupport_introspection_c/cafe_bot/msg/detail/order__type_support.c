// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cafe_bot:msg/Order.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cafe_bot/msg/detail/order__rosidl_typesupport_introspection_c.h"
#include "cafe_bot/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cafe_bot/msg/detail/order__functions.h"
#include "cafe_bot/msg/detail/order__struct.h"


// Include directives for member types
// Member `table`
// Member `item`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cafe_bot__msg__Order__init(message_memory);
}

void cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function(void * message_memory)
{
  cafe_bot__msg__Order__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array[2] = {
  {
    "table",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cafe_bot__msg__Order, table),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "item",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cafe_bot__msg__Order, item),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_members = {
  "cafe_bot__msg",  // message namespace
  "Order",  // message name
  2,  // number of fields
  sizeof(cafe_bot__msg__Order),
  cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array,  // message members
  cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_init_function,  // function to initialize message memory (memory has to be allocated)
  cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle = {
  0,
  &cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cafe_bot
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cafe_bot, msg, Order)() {
  if (!cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier) {
    cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cafe_bot__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
