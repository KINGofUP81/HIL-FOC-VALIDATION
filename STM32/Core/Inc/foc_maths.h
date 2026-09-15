/*
 * foc_maths.h
 *
 *  Created on: Jan 1, 2026
 *      Author: Afraaz Khan
 */
#ifndef FOC_MATHS_H
#define FOC_MATHS_H

typedef struct
{
    float i_alpha;
    float i_beta;
} IAlphaBeta_t;

void clarke_transform(float ia, float ib, float ic, IAlphaBeta_t *out);
void Park(float ialpha, float ibeta, float theta,
          float *id, float *iq);

#endif
