#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>

bool bno055_init(void);
bool bno055_read_pitch(float *degrees);

// Heading in degrees from -180 to 180, counterclockwise (left turn) positive.
// It is relative to the direction the robot faced when the Pico started.
bool bno055_read_heading(float *degrees);

#endif
