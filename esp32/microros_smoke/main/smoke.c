#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <cleany_base_interfaces/msg/detail/wheel_command__type_support.h>
#include <cleany_base_interfaces/msg/detail/wheel_state__type_support.h>

static const rosidl_message_type_support_t * (* volatile wheel_command_type)(void) =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_c, cleany_base_interfaces, msg, WheelCommand);
static const rosidl_message_type_support_t * (* volatile wheel_state_type)(void) =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_c, cleany_base_interfaces, msg, WheelState);
static __typeof__(&rcl_get_default_allocator) volatile rcl_symbol =
    &rcl_get_default_allocator;
static __typeof__(&rclc_support_init) volatile rclc_symbol =
    &rclc_support_init;

void app_main(void)
{
  (void)wheel_command_type();
  (void)wheel_state_type();
  (void)rcl_symbol;
  (void)rclc_symbol;
}
