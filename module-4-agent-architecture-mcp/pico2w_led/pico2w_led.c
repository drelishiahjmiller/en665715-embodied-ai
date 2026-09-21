#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/bool.h"

#include "picow_udp_transport.h"
#include "wifi_config.h"

static rcl_publisher_t led_state_publisher;
static std_msgs__msg__Bool led_command;
static std_msgs__msg__Bool led_state;

static void led_command_callback(const void *incoming) {
    const std_msgs__msg__Bool *command = incoming;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, command->data);
    led_state.data = command->data;
    printf("LED command received: %s\n", command->data ? "ON" : "OFF");
    if (rcl_publish(&led_state_publisher, &led_state, NULL) != RCL_RET_OK) {
        printf("Failed to publish LED state.\n");
    }
}

static bool check_rcl(rcl_ret_t result, const char *operation) {
    if (result == RCL_RET_OK) {
        return true;
    }
    printf("%s failed: %d\n", operation, (int) result);
    return false;
}

int main(void) {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed.\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to %s...\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000)) {
        printf("Wi-Fi connection failed.\n");
        return 1;
    }

    uint8_t *ip_address = (uint8_t *) &cyw43_state.netif[0].ip_addr.addr;
    printf("Connected. IP address: %d.%d.%d.%d\n",
           ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

    static picow_udp_transport_context_t transport_context = {
        .agent_ip = ROS_AGENT_IP,
        .agent_port = ROS_AGENT_PORT,
        .pcb = NULL,
        .receive_head = 0,
        .receive_tail = 0,
        .receive_count = 0,
    };
    if (rmw_uros_set_custom_transport(
            false,
            &transport_context,
            picow_udp_transport_open,
            picow_udp_transport_close,
            picow_udp_transport_write,
            picow_udp_transport_read) != RMW_RET_OK) {
        printf("Could not configure the micro-ROS UDP transport.\n");
        return 1;
    }

    printf("Pinging micro-ROS Agent at %s:%u...\n",
           ROS_AGENT_IP,
           (unsigned int) ROS_AGENT_PORT);
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        printf("Waiting for micro-ROS Agent...\n");
        sleep_ms(1000);
    }
    printf("micro-ROS Agent connected.\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_subscription_t led_command_subscription =
        rcl_get_zero_initialized_subscription();
    led_state_publisher = rcl_get_zero_initialized_publisher();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    if (!check_rcl(
            rclc_support_init(&support, 0, NULL, &allocator),
            "rclc_support_init") ||
        !check_rcl(
            rclc_node_init_default(&node, "pico2w_led", "", &support),
            "rclc_node_init_default") ||
        !check_rcl(
            rclc_publisher_init_default(
                &led_state_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                "/pico/led/state"),
            "rclc_publisher_init_default") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &led_command_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                "/pico/led/set"),
            "rclc_subscription_init_best_effort") ||
        !check_rcl(
            rclc_executor_init(&executor, &support.context, 1, &allocator),
            "rclc_executor_init") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &led_command_subscription,
                &led_command,
                led_command_callback,
                ON_NEW_DATA),
            "rclc_executor_add_subscription")) {
        return 1;
    }

    printf("micro-ROS node ready.\n");
    absolute_time_t next_state_publish = make_timeout_time_ms(1000);
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        if (time_reached(next_state_publish)) {
            if (rcl_publish(&led_state_publisher, &led_state, NULL) !=
                RCL_RET_OK) {
                printf("Failed to publish LED heartbeat.\n");
            }
            next_state_publish = make_timeout_time_ms(5000);
        }
    }
}
