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
#include "std_msgs/msg/float32_multi_array.h"
#include "std_msgs/msg/int32_multi_array.h"

#include "bno055.h"
#include "encoders.h"
#include "picow_udp_transport.h"
#include "tb6612.h"
#include "wheel_control.h"
#include "wifi_config.h"

static rcl_publisher_t wheel_ticks_publisher;
static rcl_publisher_t telemetry_publisher;
static rcl_publisher_t heading_publisher;
static geometry_msgs__msg__Twist cmd_vel_message;
static std_msgs__msg__Int32MultiArray wheel_ticks_message;
static int32_t wheel_ticks_buffer[2];
static std_msgs__msg__Float32MultiArray telemetry_message;
static float telemetry_buffer[TEL_COUNT];
static std_msgs__msg__Float32MultiArray gains_message;
static float gains_buffer[5];
static std_msgs__msg__Float32 heading_message;
static uint32_t commands_received;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void cmd_vel_callback(const void *incoming) {
    const geometry_msgs__msg__Twist *command = incoming;
    wheel_control_command(
        (float) command->linear.x, (float) command->angular.z, now_ms());
    commands_received++;
}

// /pico/pid/gains: [kp, ki, kd, feed_forward, ramp]   (1 = on, 0 = off)
// The last two are optional and default to on.
static void gains_callback(const void *incoming) {
    const std_msgs__msg__Float32MultiArray *msg = incoming;
    if (msg->data.size < 3) {
        printf("Ignoring /pico/pid/gains: need at least [kp, ki, kd].\n");
        return;
    }
    bool feed_forward = msg->data.size < 4 || msg->data.data[3] > 0.5f;
    bool ramp = msg->data.size < 5 || msg->data.data[4] > 0.5f;
    wheel_control_set_gains(msg->data.data[0], msg->data.data[1], msg->data.data[2],
                            feed_forward, ramp);
    printf("New gains: Kp %.3f  Ki %.3f  Kd %.3f  feed-forward %s  ramp %s\n",
           msg->data.data[0], msg->data.data[1], msg->data.data[2],
           feed_forward ? "on" : "off", ramp ? "on" : "off");
}

static void setup_float_array(std_msgs__msg__Float32MultiArray *msg, float *buffer,
                              size_t size, size_t capacity) {
    msg->data.data = buffer;
    msg->data.size = size;
    msg->data.capacity = capacity;
    msg->layout.dim.data = NULL;
    msg->layout.dim.size = 0;
    msg->layout.dim.capacity = 0;
    msg->layout.data_offset = 0;
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
    wheel_control_init();
    encoders_init();
    printf("Encoders ready on GP%d/GP%d (left) and GP%d/GP%d (right).\n",
           ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

    // /pico/wheel_ticks carries two numbers: [left, right] total encoder counts.
    // micro-ROS needs the array memory set up in advance.
    wheel_ticks_message.data.data = wheel_ticks_buffer;
    wheel_ticks_message.data.size = 2;
    wheel_ticks_message.data.capacity = 2;
    wheel_ticks_message.layout.dim.data = NULL;
    wheel_ticks_message.layout.dim.size = 0;
    wheel_ticks_message.layout.dim.capacity = 0;
    wheel_ticks_message.layout.data_offset = 0;
    setup_float_array(&telemetry_message, telemetry_buffer, TEL_COUNT, TEL_COUNT);
    setup_float_array(&gains_message, gains_buffer, 0, 5);

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

    // The robot can still drive without the IMU, but it will not publish heading.
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
    rcl_subscription_t gains_subscription = rcl_get_zero_initialized_subscription();
    wheel_ticks_publisher = rcl_get_zero_initialized_publisher();
    telemetry_publisher = rcl_get_zero_initialized_publisher();
    heading_publisher = rcl_get_zero_initialized_publisher();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    // The micro-ROS library from Module 5 allows up to 3 publishers and
    // 2 subscriptions, so no library rebuild is needed.
    if (!check_rcl(
            rclc_support_init(&support, 0, NULL, &allocator),
            "rclc_support_init") ||
        !check_rcl(
            rclc_node_init_default(&node, "pico_pid", "", &support),
            "rclc_node_init_default") ||
        !check_rcl(
            // Best effort: if one message is lost, the next total count still
            // contains every edge, so odometry loses nothing.
            rclc_publisher_init_best_effort(
                &wheel_ticks_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                "/pico/wheel_ticks"),
            "wheel ticks publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &heading_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/pico/imu/heading"),
            "IMU heading publisher") ||
        !check_rcl(
            rclc_publisher_init_best_effort(
                &telemetry_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                "/pico/pid/telemetry"),
            "PID telemetry publisher") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &cmd_vel_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/pico/cmd_vel"),
            "cmd_vel subscription") ||
        !check_rcl(
            rclc_subscription_init_default(
                &gains_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                "/pico/pid/gains"),
            "PID gains subscription") ||
        !check_rcl(
            rclc_executor_init(&executor, &support.context, 2, &allocator),
            "rclc_executor_init") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &cmd_vel_subscription,
                &cmd_vel_message,
                cmd_vel_callback,
                ON_NEW_DATA),
            "cmd_vel executor subscription") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &gains_subscription,
                &gains_message,
                gains_callback,
                ON_NEW_DATA),
            "gains executor subscription")) {
        wheel_control_stop();
        return 1;
    }

    printf("micro-ROS node ready. PID speed control on /pico/cmd_vel.\n");
    int32_t status = WHEEL_STATUS_STOPPED;
    absolute_time_t next_control = make_timeout_time_ms(WHEEL_CONTROL_PERIOD_MS);
    absolute_time_t next_imu_publish = make_timeout_time_ms(100);
    absolute_time_t next_log = make_timeout_time_ms(1000);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

        // 50 Hz: read the encoders, run the PID, publish counts and telemetry
        if (time_reached(next_control)) {
            encoders_read(&wheel_ticks_buffer[0], &wheel_ticks_buffer[1]);
            status = wheel_control_update(now_ms(), wheel_ticks_buffer[0], wheel_ticks_buffer[1]);
            wheel_control_telemetry(telemetry_buffer);
            if (rcl_publish(&wheel_ticks_publisher, &wheel_ticks_message, NULL) != RCL_RET_OK) {
                rcl_reset_error();
            }
            if (rcl_publish(&telemetry_publisher, &telemetry_message, NULL) != RCL_RET_OK) {
                rcl_reset_error();
            }
            next_control = delayed_by_ms(next_control, WHEEL_CONTROL_PERIOD_MS);
            if (time_reached(next_control)) {
                next_control = make_timeout_time_ms(WHEEL_CONTROL_PERIOD_MS);
            }
        }

        if (imu_ready && time_reached(next_imu_publish)) {
            if (bno055_read_heading(&heading_message.data) &&
                rcl_publish(&heading_publisher, &heading_message, NULL) != RCL_RET_OK) {
                printf("Failed to publish IMU heading.\n");
            }
            next_imu_publish = make_timeout_time_ms(100);
        }

        if (time_reached(next_log)) {
            printf("Status %ld | L set %.3f meas %.3f | R set %.3f meas %.3f m/s | "
                   "ticks %ld %ld | %lu commands\n",
                   (long) status, telemetry_buffer[TEL_LEFT_SETPOINT],
                   telemetry_buffer[TEL_LEFT_MEASURED], telemetry_buffer[TEL_RIGHT_SETPOINT],
                   telemetry_buffer[TEL_RIGHT_MEASURED], (long) wheel_ticks_buffer[0],
                   (long) wheel_ticks_buffer[1], (unsigned long) commands_received);
            next_log = make_timeout_time_ms(1000);
        }
    }
}
