#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t hello_publisher;
std_msgs__msg__String hello_msg;

rcl_publisher_t connect_publisher;
std_msgs__msg__String connect_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

void hello_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void) last_call_time;
    printf("Publishing hello world message\n");
    fflush(stdout);
    rcl_publish(&hello_publisher, &hello_msg, NULL);
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);
    gpio_put(16, 1);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_timer_t hello_timer;

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
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");
    rclc_publisher_init_default(
        &hello_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "hello_world");
    rclc_publisher_init_default(
        &connect_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "connect_topic");

    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true);
    rclc_timer_init_default2(
        &hello_timer,
        &support,
        RCL_MS_TO_NS(1000),
        hello_timer_callback,
        true);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_timer(&executor, &hello_timer);

    gpio_put(LED_PIN, 1);
    printf("Hello world from micro-ROS\n");
    fflush(stdout);
    rosidl_runtime_c__String__assign((rosidl_runtime_c__String *)&connect_msg, "Connected");
    rcl_publish(&connect_publisher, &connect_msg, NULL);
    rosidl_runtime_c__String__assign((rosidl_runtime_c__String *)&hello_msg, "Hello World");

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
