#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "geometry_msgs/msg/twist.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/int32.h"

#include "bno055.h"
#include "drive_control.h"
#include "picow_udp_transport.h"
#include "tb6612.h"
#include "wifi_config.h"

static rcl_publisher_t pitch_publisher;
static rcl_publisher_t heading_publisher;
static rcl_publisher_t drive_status_publisher;
static geometry_msgs__msg__Twist cmd_vel_message;
static std_msgs__msg__Float32 pitch_message;
static std_msgs__msg__Float32 heading_message;
static std_msgs__msg__Int32 drive_status_message;
static uint32_t commands_received;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void publish_drive_status(int32_t status) {
    drive_status_message.data = status;
    if (rcl_publish(&drive_status_publisher, &drive_status_message, NULL) !=
        RCL_RET_OK) {
        printf("Failed to publish drive status.\n");
        rcl_reset_error();
    }
}

static void cmd_vel_callback(const void *incoming) {
    const geometry_msgs__msg__Twist *command = incoming;
    drive_control_command(
        (float) command->linear.x, (float) command->angular.z, now_ms());
    commands_received++;
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
    sleep_ms(1500);
    tb6612_init();
    drive_control_init();

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

    // The robot can still drive without the IMU, but it will not publish
    // pitch or heading.
    bool imu_ready = bno055_init();
    if (imu_ready) {
        printf("BNO055 ready. Heading starts at 0 degrees.\n");
    } else {
        printf("BNO055 not found; IMU topics will not be published.\n");
    }

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
    rcl_subscription_t cmd_vel_subscription =
        rcl_get_zero_initialized_subscription();
    pitch_publisher = rcl_get_zero_initialized_publisher();
    heading_publisher = rcl_get_zero_initialized_publisher();
    drive_status_publisher = rcl_get_zero_initialized_publisher();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    // The micro-ROS library from Module 5 allows up to 3 publishers and
    // 2 subscriptions, so no library rebuild is needed.
    if (!check_rcl(
            rclc_support_init(&support, 0, NULL, &allocator),
            "rclc_support_init") ||
        !check_rcl(
            rclc_node_init_default(&node, "pico2w_drive", "", &support),
            "rclc_node_init_default") ||
        !check_rcl(
            rclc_publisher_init_default(
                &pitch_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/pico/imu/pitch"),
            "IMU pitch publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &heading_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/pico/imu/heading"),
            "IMU heading publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &drive_status_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "/pico/drive/status"),
            "drive status publisher") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &cmd_vel_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/pico/cmd_vel"),
            "cmd_vel subscription") ||
        !check_rcl(
            rclc_executor_init(&executor, &support.context, 1, &allocator),
            "rclc_executor_init") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &cmd_vel_subscription,
                &cmd_vel_message,
                cmd_vel_callback,
                ON_NEW_DATA),
            "cmd_vel executor subscription")) {
        drive_control_stop();
        return 1;
    }

    printf("micro-ROS node ready. Listening on /pico/cmd_vel.\n");
    int32_t last_status = DRIVE_STATUS_STOPPED;
    absolute_time_t next_imu_publish = make_timeout_time_ms(100);
    absolute_time_t next_status_publish = make_timeout_time_ms(1000);
    absolute_time_t next_log = make_timeout_time_ms(1000);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        int32_t status = drive_control_update(now_ms());
        if (status != last_status || time_reached(next_status_publish)) {
            if (status == DRIVE_STATUS_TIMEOUT_STOP &&
                last_status != DRIVE_STATUS_TIMEOUT_STOP) {
                printf("No /pico/cmd_vel for 500 ms; wheels stopped.\n");
            }
            publish_drive_status(status);
            last_status = status;
            next_status_publish = make_timeout_time_ms(1000);
        }

        if (imu_ready && time_reached(next_imu_publish)) {
            if (bno055_read_pitch(&pitch_message.data) &&
                rcl_publish(&pitch_publisher, &pitch_message, NULL) !=
                    RCL_RET_OK) {
                printf("Failed to publish IMU pitch.\n");
            }
            if (bno055_read_heading(&heading_message.data) &&
                rcl_publish(&heading_publisher, &heading_message, NULL) !=
                    RCL_RET_OK) {
                printf("Failed to publish IMU heading.\n");
            }
            next_imu_publish = make_timeout_time_ms(100);
        }

        if (time_reached(next_log)) {
            printf("Drive status %ld, heading %.1f deg, %lu commands received\n",
                   (long) status,
                   imu_ready ? heading_message.data : 0.0f,
                   (unsigned long) commands_received);
            next_log = make_timeout_time_ms(1000);
        }
    }
}
