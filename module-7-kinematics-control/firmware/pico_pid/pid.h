#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;             // proportional gain  (PWM effort per m/s of error)
    float ki;             // integral gain      (per m/s per second)
    float kd;             // derivative gain    (per m/s^2)
    float integral;       // running sum of error * dt
    float last_measured;  // for the derivative term
    bool has_last;
} wheel_pid_t;

void pid_init(wheel_pid_t *pid, float kp, float ki, float kd);
void pid_reset(wheel_pid_t *pid);

// One PID step. Returns the motor effort from -1.0 (full reverse) to 1.0
// (full forward). u_ff is the feed-forward guess added before the PID terms.
float pid_step(wheel_pid_t *pid, float setpoint, float measured, float u_ff, float dt);

#endif
