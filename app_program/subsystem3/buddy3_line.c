#include "buddy3_line.h"

#include <stddef.h>

bool s3_normalize(uint16_t raw, uint16_t white, uint16_t black,
                  uint16_t raw_max, uint16_t *p_darkness)
{
    int32_t span = (int32_t)black - (int32_t)white;
    int32_t value;

    if (p_darkness == NULL)
    {
        return false;
    }
    *p_darkness = 0U;
    if ((span == 0) || (raw_max == 0U) || (raw > raw_max) ||
        (white > raw_max) || (black > raw_max))
    {
        return false;
    }
    value = (((int32_t)raw - (int32_t)white) * 1000) / span;
    if (value < 0)
    {
        value = 0;
    }
    if (value > 1000)
    {
        value = 1000;
    }
    *p_darkness = (uint16_t)value;
    return true;
}
