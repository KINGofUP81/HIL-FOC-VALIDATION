#ifndef HIL_TX_H
#define HIL_TX_H

#include <stdint.h>

void HIL_SendFloats(const float *vars, uint8_t count);

#endif
