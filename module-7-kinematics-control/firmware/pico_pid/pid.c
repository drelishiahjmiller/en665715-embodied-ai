#include "pid.h"

void pid_init(wheel_pid_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid_reset(pid);
}

void pid_reset(wheel_pid_t *pid) {
    pid->integral = 0.0f;
    pid->last_measured = 0.0f;
    pid->has_last = false;
}

float pid_step(wheel_pid_t *pid, float setpoint, float measured, float u_ff, float dt) {
    // Step 1: error = how far we are from the target (positive = too slow)
    float error = setpoint - measured;

    // Step 2: candidate integral (only kept if the output is not saturated)
    float integral = pid->integral + error * dt;

    // Step 3: derivative on the MEASUREMENT, so a setpoint jump causes no kick
    float derivative = 0.0f;
    if (pid->has_last && dt > 0.0f) {
        derivative = -(measured - pid->last_measured) / dt;
    }
    pid->last_measured = measured;
    pid->has_last = true;

    // Step 4: feed-forward + P + I + D
    float u = u_ff + pid->kp * error + pid->ki * integral + pid->kd * derivative;

    // Step 5: clamp to the motor's range. Anti-windup: if the output is
    // saturated, do not let the integral keep growing.
    if (u > 1.0f) {
        u = 1.0f;
    } else if (u < -1.0f) {
        u = -1.0f;
    } else {
        pid->integral = integral;
    }
    return u;
}
