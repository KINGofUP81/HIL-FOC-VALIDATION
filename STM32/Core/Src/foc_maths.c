/*
 * foc_maths.c
 *
 *  Created on: Jan 1, 2026
 *      Author: Afraaz Khan
 */

#include "foc_maths.h"
#include "cdc_rx.h"
#include <math.h>

/*
 * Clarke transform (3-phase → αβ)
 *
 * Assumes:
 *   ia + ib + ic = 0   (balanced system)
 *
 * Standard power-invariant Clarke:
 *
 * i_alpha = ia
 * i_beta  = (ia + 2*ib) / sqrt(3)
 */

void clarke_transform(float ia, float ib, float ic, IAlphaBeta_t *out)
{
    (void)ic;  // unused in standard Clarke (but kept for clarity)

    out->i_alpha = ia;
    out->i_beta  = (ib - ic) * 0.577350269f;
}

void Park(float ialpha, float ibeta, float theta,
          float *id, float *iq)
{
    float s = sinf(theta);
    float c = cosf(theta);

    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}



