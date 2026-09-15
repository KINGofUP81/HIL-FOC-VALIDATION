/*
 * hil_tx.c
 *
 *  Created on: Jan 1, 2026
 *      Author: Afraaz Khan
 */
#include "hil_tx.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define SOF 0xAA
#define EOF 0x55
#define MAX_FLOATS 6   // safe limit for now

void HIL_SendFloats(const float *vars, uint8_t count)
{
    if (count == 0 || count > MAX_FLOATS)
        return;

    uint8_t frame_len = 1 + (count * 4) + 1;
    uint8_t frame[32];

    frame[0] = SOF;
    frame[frame_len - 1] = EOF;

    for (uint8_t i = 0; i < count; i++)
    {
        memcpy(&frame[1 + i*4], &vars[i], 4);
    }

    while (CDC_Transmit_FS(frame, frame_len) == USBD_BUSY);
}



