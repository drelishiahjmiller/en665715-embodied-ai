#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>

bool bno055_init(void);
bool bno055_read_pitch(float *degrees);

#endif
