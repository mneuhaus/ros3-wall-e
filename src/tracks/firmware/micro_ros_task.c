#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

int main(void) {
    stdio_init_all();
    printf("micro_ros_task initializing...\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        printf("Error in support init: %d\n", rc);
        return -1;
    }

    rcl_node_t node;
    rc = rclc_node_init_default(&node, "micro_ros_hello_node", "", &support);
    if (rc != RCL_RET_OK) {
        printf("Error in node init: %d\n", rc);
        return -1;
    }

    rcl_publisher_t publisher;
    rc = rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "hello_topic");
    if (rc != RCL_RET_OK) {
        printf("Error initializing publisher: %d\n", rc);
        return -1;
    }

    std_msgs__msg__String msg;
    char hello_str[] = "Hello World";
    msg.data.data = hello_str;
    msg.data.size = strlen(hello_str);
    msg.data.capacity = msg.data.size + 1;

    printf("micro_ros_task started. Publishing 'Hello World' every second...\n");

    while (true) {
        rc = rcl_publish(&publisher, &msg, NULL);
        if (rc != RCL_RET_OK) {
            printf("Error publishing: %d\n", rc);
        }
        sleep_ms(1000);
    }

    // Cleanup (unreachable)
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return 0;
}
