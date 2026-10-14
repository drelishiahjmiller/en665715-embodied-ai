#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

enum {
    WHEEL_STATUS_STOPPED = 0,
    WHEEL_STATUS_DRIVING = 1,
    WHEEL_STATUS_TIMEOUT_STOP = 2,
};

// Control loop period
#define WHEEL_CONTROL_PERIOD_MS 20

// Values sent on /pico/pid/telemetry, in this order
enum {
    TEL_TIME_S = 0,
    TEL_LEFT_SETPOINT,
    TEL_LEFT_MEASURED,
    TEL_LEFT_EFFORT,
    TEL_RIGHT_SETPOINT,
    TEL_RIGHT_MEASURED,
    TEL_RIGHT_EFFORT,
    TEL_STATUS,
    TEL_COUNT,
};

void wheel_control_init(void);

// New robot command: v in m/s (forward +), w in rad/s (left turn +).
void wheel_control_command(float v, float w, uint32_t now_ms);

// New PID gains for both wheels. feed_forward: add target / max speed.
// ramp: limit setpoint changes to MAX_ACCEL (turn off for pure step tests).
void wheel_control_set_gains(float kp, float ki, float kd, bool feed_forward, bool ramp);

// Run one control step. Call every WHEEL_CONTROL_PERIOD_MS with the total
// encoder counts. Returns the status.
int32_t wheel_control_update(uint32_t now_ms, int32_t left_counts, int32_t right_counts);

// Copy the latest telemetry values (TEL_COUNT floats).
void wheel_control_telemetry(float *out);

void wheel_control_stop(void);

#endif
