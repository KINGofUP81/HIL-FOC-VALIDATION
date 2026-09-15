#ifndef CDC_RX_H
#define CDC_RX_H

#include <stdint.h>

/* -------- received currents structure -------- */
typedef struct {
    float ia;
    float ib;
    float ic;
    float theta;
} Currents_t;

/* -------- shared RX state (DECLARATIONS ONLY) -------- */
extern volatile uint8_t frame_ready;
extern Currents_t rx_currents;

#endif /* CDC_RX_H */
