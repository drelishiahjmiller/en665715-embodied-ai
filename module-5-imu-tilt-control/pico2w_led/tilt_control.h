#ifndef TILT_CONTROL_H
#define TILT_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

enum {
    TILT_REQUEST_BACKWARD = -1,
    TILT_REQUEST_FORWARD = 1,
};

enum {
    TILT_STATUS_NONE = 0,
    TILT_STATUS_READY_FORWARD = 1,
    TILT_STATUS_READY_BACKWARD = 2,
    TILT_STATUS_RUNNING_FORWARD = 3,
    TILT_STATUS_RUNNING_BACKWARD = 4,
    TILT_STATUS_STOPPED_FORWARD = 5,
    TILT_STATUS_STOPPED_BACKWARD = 6,
    TILT_STATUS_ERROR = 7,
    TILT_STATUS_BUSY = 8,
};

void tilt_control_init(float neutral_pitch, float forward_tilt_sign);
int32_t tilt_control_arm(int32_t direction);
int32_t tilt_control_update(bool pitch_valid, float pitch_degrees);
void tilt_control_stop(void);

#endif
