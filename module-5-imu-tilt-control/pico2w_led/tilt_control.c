#include "tilt_control.h"

#include "pico/stdlib.h"

#include "tb6612.h"

// A tilt this far from the position recorded at arming starts the wheels.
#define TILT_TRIGGER_DEG 2.0f
#define TILT_HOLD_MS 100
#define MOVE_DURATION_MS 10000

typedef enum {
    STATE_IDLE,
    STATE_ARMED_FORWARD,
    STATE_ARMED_BACKWARD,
    STATE_RUNNING_FORWARD,
    STATE_RUNNING_BACKWARD,
} tilt_state_t;

static tilt_state_t state = STATE_IDLE;
static bool tilt_hold_active;
static absolute_time_t tilt_hold_deadline;
static absolute_time_t stop_deadline;
static float latest_pitch;
static float reference_pitch;

void tilt_control_init(float initial_pitch) {
    latest_pitch = initial_pitch;
    reference_pitch = initial_pitch;
    state = STATE_IDLE;
    tilt_hold_active = false;
    tb6612_stop();
}

int32_t tilt_control_arm(int32_t direction) {
    if (state != STATE_IDLE) {
        return TILT_STATUS_BUSY;
    }
    if (direction != TILT_REQUEST_FORWARD &&
        direction != TILT_REQUEST_BACKWARD) {
        return TILT_STATUS_ERROR;
    }

    reference_pitch = latest_pitch;
    tilt_hold_active = false;
    if (direction == TILT_REQUEST_FORWARD) {
        state = STATE_ARMED_FORWARD;
        return TILT_STATUS_READY_FORWARD;
    }
    state = STATE_ARMED_BACKWARD;
    return TILT_STATUS_READY_BACKWARD;
}

int32_t tilt_control_update(bool pitch_valid, float pitch_degrees) {
    if (pitch_valid) {
        latest_pitch = pitch_degrees;
    }

    if ((state == STATE_ARMED_FORWARD || state == STATE_ARMED_BACKWARD) &&
        !pitch_valid) {
        state = STATE_IDLE;
        tilt_hold_active = false;
        tb6612_stop();
        return TILT_STATUS_ERROR;
    }

    if (state == STATE_ARMED_FORWARD || state == STATE_ARMED_BACKWARD) {
        bool forward = state == STATE_ARMED_FORWARD;
        float change = pitch_degrees - reference_pitch;
        bool tilted = change >= TILT_TRIGGER_DEG || change <= -TILT_TRIGGER_DEG;

        if (!tilted) {
            tilt_hold_active = false;
        } else if (!tilt_hold_active) {
            tilt_hold_deadline = make_timeout_time_ms(TILT_HOLD_MS);
            tilt_hold_active = true;
        } else if (time_reached(tilt_hold_deadline)) {
            tb6612_drive(forward);
            state = forward ? STATE_RUNNING_FORWARD : STATE_RUNNING_BACKWARD;
            tilt_hold_active = false;
            stop_deadline = make_timeout_time_ms(MOVE_DURATION_MS);
            return forward ? TILT_STATUS_RUNNING_FORWARD
                           : TILT_STATUS_RUNNING_BACKWARD;
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
    tilt_hold_active = false;
}
