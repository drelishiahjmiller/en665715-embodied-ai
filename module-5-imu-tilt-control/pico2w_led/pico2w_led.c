#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/int32.h"

#include "bno055.h"
#include "picow_udp_transport.h"
#include "tb6612.h"
#include "tilt_control.h"
#include "wifi_config.h"

#define FORWARD_TILT_SIGN 1.0f

static rcl_publisher_t led_state_publisher;
static rcl_publisher_t pitch_publisher;
static rcl_publisher_t tilt_status_publisher;
static std_msgs__msg__Bool led_command;
static std_msgs__msg__Bool led_state;
static std_msgs__msg__Float32 pitch_message;
static std_msgs__msg__Int32 tilt_request_message;
static std_msgs__msg__Int32 tilt_status_message;

static void publish_tilt_status(int32_t status) {
    tilt_status_message.data = status;
    rcl_ret_t result =
        rcl_publish(&tilt_status_publisher, &tilt_status_message, NULL);
    if (result != RCL_RET_OK) {
        rcl_error_string_t error = rcl_get_error_string();
        printf("Could not publish tilt status %ld: rcl_publish=%d: %s\n",
               (long) status,
               (int) result,
               error.str);
        rcl_reset_error();
    }
}

static void led_command_callback(const void *incoming) {
    const std_msgs__msg__Bool *command = incoming;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, command->data);
    led_state.data = command->data;
    printf("LED command received: %s\n", command->data ? "ON" : "OFF");
    if (rcl_publish(&led_state_publisher, &led_state, NULL) != RCL_RET_OK) {
        printf("Failed to publish LED state.\n");
    }
}

static void tilt_request_callback(const void *incoming) {
    const std_msgs__msg__Int32 *request = incoming;
    publish_tilt_status(tilt_control_arm(request->data));
}

static bool check_rcl(rcl_ret_t result, const char *operation) {
    if (result == RCL_RET_OK) {
        return true;
    }
    printf("%s failed: %d\n", operation, (int) result);
    return false;
}

static bool calibrate_neutral_pitch(float *neutral_pitch) {
    float sum = 0.0f;
    for (int sample = 0; sample < 10; sample++) {
        float pitch = 0.0f;
        if (!bno055_read_pitch(&pitch)) {
            return false;
        }
        sum += pitch;
        sleep_ms(50);
    }
    *neutral_pitch = sum / 10.0f;
    return true;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    tb6612_init();
    tb6612_stop();

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

    if (!bno055_init()) {
        while (true) {
            printf("BNO055 initialization failed; motors remain stopped.\n");
            sleep_ms(1000);
        }
    }

    float neutral_pitch = 0.0f;
    if (!calibrate_neutral_pitch(&neutral_pitch)) {
        while (true) {
            printf("Could not calibrate BNO055; motors remain stopped.\n");
            sleep_ms(1000);
        }
    }
    tilt_control_init(neutral_pitch, FORWARD_TILT_SIGN);
    printf("Neutral pitch: %.2f degrees\n", neutral_pitch);

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
    rcl_subscription_t tilt_request_subscription =
        rcl_get_zero_initialized_subscription();
    led_state_publisher = rcl_get_zero_initialized_publisher();
    pitch_publisher = rcl_get_zero_initialized_publisher();
    tilt_status_publisher = rcl_get_zero_initialized_publisher();
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
            "LED state publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &pitch_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                "/pico/imu/pitch"),
            "IMU pitch publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &tilt_status_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "/pico/tilt/status"),
            "tilt status publisher") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &led_command_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                "/pico/led/set"),
            "LED command subscription") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &tilt_request_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "/pico/tilt/request"),
            "tilt request subscription") ||
        !check_rcl(
            rclc_executor_init(&executor, &support.context, 2, &allocator),
            "rclc_executor_init") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &led_command_subscription,
                &led_command,
                led_command_callback,
                ON_NEW_DATA),
            "LED executor subscription") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &tilt_request_subscription,
                &tilt_request_message,
                tilt_request_callback,
                ON_NEW_DATA),
            "tilt request executor subscription")) {
        tilt_control_stop();
        return 1;
    }

    printf("micro-ROS node ready.\n");
    absolute_time_t next_sensor_read = make_timeout_time_ms(50);
    absolute_time_t next_pitch_publish = make_timeout_time_ms(100);
    absolute_time_t next_pitch_log = make_timeout_time_ms(1000);
    absolute_time_t next_led_heartbeat = make_timeout_time_ms(1000);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        if (time_reached(next_sensor_read)) {
            float pitch = 0.0f;
            bool pitch_valid = bno055_read_pitch(&pitch);
            if (pitch_valid && time_reached(next_pitch_publish)) {
                pitch_message.data = pitch;
                if (rcl_publish(&pitch_publisher, &pitch_message, NULL) !=
                    RCL_RET_OK) {
                    printf("Failed to publish IMU pitch.\n");
                }
                next_pitch_publish = make_timeout_time_ms(100);
            }
            if (time_reached(next_pitch_log)) {
                if (pitch_valid) {
                    printf("BNO055 pitch: %.2f degrees\n", pitch);
                } else {
                    printf("BNO055 pitch read failed.\n");
                }
                next_pitch_log = make_timeout_time_ms(1000);
            }
            int32_t status = tilt_control_update(pitch_valid, pitch);
            if (status != TILT_STATUS_NONE) {
                publish_tilt_status(status);
                if (status == TILT_STATUS_ERROR) {
                    printf("BNO055 read failed; armed movement cancelled.\n");
                }
            }
            next_sensor_read = make_timeout_time_ms(50);
        }

        if (time_reached(next_led_heartbeat)) {
            if (rcl_publish(&led_state_publisher, &led_state, NULL) !=
                RCL_RET_OK) {
                printf("Failed to publish LED heartbeat.\n");
            }
            next_led_heartbeat = make_timeout_time_ms(5000);
        }
    }
}
