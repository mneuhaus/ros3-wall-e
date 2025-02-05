#include <stdio.h>
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"
#include <string.h>

rcl_publisher_t publisher;
std_msgs__msg__String msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    (void) ret;
    printf("Published: %s\n", msg.data.data);
    fflush(stdout);
}

int main() {
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close,
                                   pico_serial_transport_write, pico_serial_transport_read);
    stdio_init_all();
    // Initialize neopixel on GPIO16 without external dependency
    #define NEOPIXEL_PIN 16
    #define NEOPIXEL_COUNT 1
    gpio_init(NEOPIXEL_PIN);
    gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);
    // Minimal manual implementation (stub): set neopixel to green placeholder
    gpio_put(NEOPIXEL_PIN, 1);

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        printf("micro-ROS agent not available. Exiting.\n");
        return -1;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "tracks_node", "", &support);

    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "tracks_topic");

    const char * my_message = "Hello from tracks firmware";
    msg.data.data = (char *) my_message;
    msg.data.size = strlen(my_message);
    msg.data.capacity = msg.data.size + 1;

    rcl_timer_t timer;
    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(1000), timer_callback, true);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
