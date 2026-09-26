#ifndef TB6612_H
#define TB6612_H

#include <stdbool.h>

void tb6612_init(void);
void tb6612_stop(void);
void tb6612_drive(bool forward);

#endif
