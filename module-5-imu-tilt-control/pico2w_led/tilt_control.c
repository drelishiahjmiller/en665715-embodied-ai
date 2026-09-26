#include "tilt_control.h"

#include "pico/stdlib.h"

#include "tb6612.h"

#define TILT_THRESHOLD_DEG 15.0f
#define MOVE_DURATION_MS 5000

typedef enum {
    STATE_IDLE,
    STATE_ARMED_FORWARD,
    STATE_ARMED_BACKWARD,
    STATE_RUNNING_FORWARD,
    STATE_RUNNING_BACKWARD,
} tilt_state_t;

static tilt_state_t state = STATE_IDLE;
static absolute_time_t stop_deadline;
static float neutral_pitch;
static float forward_tilt_sign = 1.0f;

void tilt_control_init(float level_pitch, float tilt_sign) {
    neutral_pitch = level_pitch;
    forward_tilt_sign = tilt_sign >= 0.0f ? 1.0f : -1.0f;
    state = STATE_IDLE;
    tb6612_stop();
}

int32_t tilt_control_arm(int32_t direction) {
    if (state != STATE_IDLE) {
        return TILT_STATUS_BUSY;
    }
    if (direction == TILT_REQUEST_FORWARD) {
        state = STATE_ARMED_FORWARD;
        return TILT_STATUS_READY_FORWARD;
    }
    if (direction == TILT_REQUEST_BACKWARD) {
        state = STATE_ARMED_BACKWARD;
        return TILT_STATUS_READY_BACKWARD;
    }
    return TILT_STATUS_ERROR;
}

int32_t tilt_control_update(bool pitch_valid, float pitch_degrees) {
    if ((state == STATE_ARMED_FORWARD || state == STATE_ARMED_BACKWARD) &&
        !pitch_valid) {
        state = STATE_IDLE;
        tb6612_stop();
        return TILT_STATUS_ERROR;
    }

    if (state == STATE_ARMED_FORWARD || state == STATE_ARMED_BACKWARD) {
        float tilt = (pitch_degrees - neutral_pitch) * forward_tilt_sign;
        if (state == STATE_ARMED_FORWARD && tilt >= TILT_THRESHOLD_DEG) {
            tb6612_drive(true);
            state = STATE_RUNNING_FORWARD;
            stop_deadline = make_timeout_time_ms(MOVE_DURATION_MS);
            return TILT_STATUS_RUNNING_FORWARD;
        }
        if (state == STATE_ARMED_BACKWARD && tilt <= -TILT_THRESHOLD_DEG) {
            tb6612_drive(false);
            state = STATE_RUNNING_BACKWARD;
            stop_deadline = make_timeout_time_ms(MOVE_DURATION_MS);
            return TILT_STATUS_RUNNING_BACKWARD;
        }
    }

    if ((state == STATE_RUNNING_FORWARD || state == STATE_RUNNING_BACKWARD) &&
        time_reached(stop_deadline)) {
        bool was_forward = state == STATE_RUNNING_FORWARD;
        tb6612_stop();
        state = STATE_IDLE;
        return was_forward ? TILT_STATUS_STOPPED_FORWARD
                           : TILT_STATUS_STOPPED_BACKWARD;
    }

    return TILT_STATUS_NONE;
}

void tilt_control_stop(void) {
    tb6612_stop();
    state = STATE_IDLE;
}
