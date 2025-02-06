#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void right_motor_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
    int command = msg->data;
    if (command >= 0) {
        gpio_put(4, 1);
        pwm_set_gpio_level(3, (command > 255 ? 255 : command));
    } else {
        gpio_put(4, 0);
        pwm_set_gpio_level(3, (-command > 255 ? 255 : -command));
    }
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *) msgin;
    float linear = twist->linear.x;
    float angular = twist->angular.z;
    float half_wheel_base = 0.5f; // adjust based on your robot geometry
    int left_command = (int)((linear - angular * half_wheel_base) * 255.0f);
    int right_command = (int)((linear + angular * half_wheel_base) * 255.0f);
    if (left_command > 255) left_command = 255;
    if (left_command < -255) left_command = -255;
    if (right_command > 255) right_command = 255;
    if (right_command < -255) right_command = -255;
    std_msgs__msg__Int32 left_msg;
    left_msg.data = left_command;
    left_motor_callback(&left_msg);
    std_msgs__msg__Int32 right_msg;
    right_msg.data = right_command;
    right_motor_callback(&right_msg);
}

void left_motor_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
    int command = msg->data;
    if (command >= 0) {
        gpio_put(8, 1);
        pwm_set_gpio_level(7, (command > 255 ? 255 : command));
    } else {
        gpio_put(8, 0);
        pwm_set_gpio_level(7, (-command > 255 ? 255 : -command));
    }
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize Motor Driver pins:
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 1); // VCC for Motor Driver

    // Initialize Motor PWM on GP3:
    gpio_init(3);
    gpio_set_function(3, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(3);
    pwm_set_wrap(slice_num, 255);
    pwm_set_gpio_level(3, 0); // Motor off

    // Initialize Motor Direction on GP4:
    gpio_init(4);
    gpio_set_dir(4, GPIO_OUT);
    gpio_put(4, 0); // default direction

    // Initialize Left Motor VCC, PWM, and Direction:
    gpio_init(6);
    gpio_set_dir(6, GPIO_OUT);
    gpio_put(6, 1); // VCC for Left Motor

    gpio_init(7);
    gpio_set_function(7, GPIO_FUNC_PWM);
    uint left_slice_num = pwm_gpio_to_slice_num(7);
    pwm_set_wrap(left_slice_num, 255);
    pwm_set_gpio_level(7, 0); // Motor off

    gpio_init(8);
    gpio_set_dir(8, GPIO_OUT);
    gpio_put(8, 0); // default direction for left motor

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rcl_subscription_t cmd_vel_sub;
    geometry_msgs__msg__Twist cmd_vel_msg;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 0);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(10);
    }
    return 0;
}
