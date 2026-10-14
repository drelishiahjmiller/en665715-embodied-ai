#include "wheel_control.h"

#include <math.h>

#include "pid.h"
#include "tb6612.h"

// ===== Course robot =====
#define WHEEL_SEPARATION_M 0.127f            // 5 in, center to center
#define WHEEL_DIAMETER_M 0.0635f             // 2.5 in
#define COUNTS_PER_REV 4320.0f               // x4 counts per wheel turn
#define M_PER_COUNT (3.14159265f * WHEEL_DIAMETER_M / COUNTS_PER_REV)

// ===== Calibrate these for your robot (PID guide, Step 6) =====
// Wheel speed at full PWM (feed-forward scale). TT 1:90 motors: about 0.38 m/s
// with no load.
#define MAX_WHEEL_SPEED_MPS 0.40f
// Smallest PWM fraction that turns a wheel. Effort above 0 starts here.
#define MIN_DUTY 0.40f

// ===== Limits =====
#define MAX_LINEAR_MPS 0.30f
#define MAX_ANGULAR_RPS 1.50f
#define MAX_ACCEL_MPS2 0.50f                 // setpoint ramp (smooth starts and stops)
#define COMMAND_TIMEOUT_MS 500               // stop if /pico/cmd_vel goes quiet
#define SPEED_FILTER 0.5f                    // 0..1, higher = smoother but slower
#define STOPPED_SPEED_MPS 0.01f

// ===== Default gains (tune in the PID guide) =====
#define DEFAULT_KP 1.0f
#define DEFAULT_KI 5.0f
#define DEFAULT_KD 0.0f

typedef struct {
    wheel_pid_t pid;
    float target;        // from inverse kinematics (m/s)
    float setpoint;      // target after the acceleration ramp (m/s)
    float measured;      // filtered speed from the encoder (m/s)
    float effort;        // PID output, -1..1
    int32_t last_counts;
} wheel_t;

static wheel_t left_wheel, right_wheel;
static bool feed_forward_on = true;
static bool ramp_on = true;
static bool have_counts = false;
static uint32_t last_command_ms = 0;
static uint32_t last_update_ms = 0;
static int32_t status = WHEEL_STATUS_STOPPED;
static float telemetry[TEL_COUNT];

static float clampf(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
}

// PID effort (-1..1) -> motor duty, skipping the dead zone where the motor
// does not turn: any effort above 0.01 starts at MIN_DUTY.
static float effort_to_duty(float effort) {
    if (fabsf(effort) < 0.01f) {
        return 0.0f;
    }
    float duty = MIN_DUTY + (1.0f - MIN_DUTY) * fabsf(effort);
    return effort > 0.0f ? duty : -duty;
}

static void reset_wheel(wheel_t *wheel) {
    wheel->target = 0.0f;
    wheel->setpoint = 0.0f;
    wheel->effort = 0.0f;
    pid_reset(&wheel->pid);
}

void wheel_control_init(void) {
    pid_init(&left_wheel.pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    pid_init(&right_wheel.pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    reset_wheel(&left_wheel);
    reset_wheel(&right_wheel);
    left_wheel.measured = right_wheel.measured = 0.0f;
    tb6612_stop();
}

void wheel_control_command(float v, float w, uint32_t now_ms) {
    v = clampf(v, MAX_LINEAR_MPS);
    w = clampf(w, MAX_ANGULAR_RPS);
    // Inverse kinematics: robot command (v, w) -> wheel speeds
    left_wheel.target = v - w * WHEEL_SEPARATION_M / 2.0f;
    right_wheel.target = v + w * WHEEL_SEPARATION_M / 2.0f;
    last_command_ms = now_ms;
}

void wheel_control_set_gains(float kp, float ki, float kd, bool feed_forward, bool ramp) {
    pid_init(&left_wheel.pid, kp, ki, kd);
    pid_init(&right_wheel.pid, kp, ki, kd);
    feed_forward_on = feed_forward;
    ramp_on = ramp;
}

static void control_wheel(wheel_t *wheel, int32_t counts, float dt) {
    // Measure: encoder counts since the last step -> speed, lightly filtered
    float raw = (float) (counts - wheel->last_counts) * M_PER_COUNT / dt;
    wheel->last_counts = counts;
    wheel->measured = SPEED_FILTER * wheel->measured + (1.0f - SPEED_FILTER) * raw;

    // Ramp the setpoint toward the target: at most MAX_ACCEL_MPS2 * dt per step
    if (ramp_on) {
        wheel->setpoint += clampf(wheel->target - wheel->setpoint, MAX_ACCEL_MPS2 * dt);
    } else {
        wheel->setpoint = wheel->target;
    }

    // Fully stopped: motors off and PID reset, so nothing creeps or winds up
    if (wheel->setpoint == 0.0f && fabsf(wheel->measured) < STOPPED_SPEED_MPS) {
        wheel->effort = 0.0f;
        pid_reset(&wheel->pid);
        return;
    }
    float u_ff = feed_forward_on ? wheel->setpoint / MAX_WHEEL_SPEED_MPS : 0.0f;
    wheel->effort = pid_step(&wheel->pid, wheel->setpoint, wheel->measured, u_ff, dt);
}

int32_t wheel_control_update(uint32_t now_ms, int32_t left_counts, int32_t right_counts) {
    if (!have_counts) {
        left_wheel.last_counts = left_counts;
        right_wheel.last_counts = right_counts;
        last_update_ms = now_ms;
        have_counts = true;
        return status;
    }
    float dt = (float) (now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;
    if (dt <= 0.0f) {
        return status;
    }

    // Watchdog: no fresh command -> target speed 0 (the ramp brings the robot to a stop)
    bool timed_out = now_ms - last_command_ms > COMMAND_TIMEOUT_MS;
    if (timed_out) {
        left_wheel.target = right_wheel.target = 0.0f;
    }

    control_wheel(&left_wheel, left_counts, dt);
    control_wheel(&right_wheel, right_counts, dt);

    float left_duty = effort_to_duty(left_wheel.effort);
    float right_duty = effort_to_duty(right_wheel.effort);
    tb6612_set(left_duty, right_duty);

    bool moving = left_duty != 0.0f || right_duty != 0.0f;
    status = moving ? WHEEL_STATUS_DRIVING
                    : (timed_out && last_command_ms != 0 ? WHEEL_STATUS_TIMEOUT_STOP
                                                         : WHEEL_STATUS_STOPPED);

    telemetry[TEL_TIME_S] = (float) now_ms / 1000.0f;
    telemetry[TEL_LEFT_SETPOINT] = left_wheel.setpoint;
    telemetry[TEL_LEFT_MEASURED] = left_wheel.measured;
    telemetry[TEL_LEFT_EFFORT] = left_wheel.effort;
    telemetry[TEL_RIGHT_SETPOINT] = right_wheel.setpoint;
    telemetry[TEL_RIGHT_MEASURED] = right_wheel.measured;
    telemetry[TEL_RIGHT_EFFORT] = right_wheel.effort;
    telemetry[TEL_STATUS] = (float) status;
    return status;
}

void wheel_control_telemetry(float *out) {
    for (int i = 0; i < TEL_COUNT; i++) {
        out[i] = telemetry[i];
    }
}

void wheel_control_stop(void) {
    reset_wheel(&left_wheel);
    reset_wheel(&right_wheel);
    tb6612_stop();
    status = WHEEL_STATUS_STOPPED;
}
