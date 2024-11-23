#ifndef __ESC_H
#define __ESC_H

#include "main.h"

#define HOBBYWING

typedef const struct
{
    unsigned int CURR_MAX, ACCELERATOR_us_MIN, ACCELERATOR_us_MAX;
    unsigned char RATIO;
} ESC_prop_t;

#ifdef HOBBYWING
static ESC_prop_t HOBBYWING_PLATINUM_120A_V4 =
    {
        .CURR_MAX = 120,
        .ACCELERATOR_us_MIN = 1100,
        .ACCELERATOR_us_MAX = 1940,
        .RATIO = 7};
#endif

void ESC_SetCURR(volatile unsigned int *TIM_CCR, ESC_prop_t *ESC, float curr);

#endif