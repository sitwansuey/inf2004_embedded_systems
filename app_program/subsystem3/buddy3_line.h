#ifndef BUDDY3_LINE_H
#define BUDDY3_LINE_H

#include <stdbool.h>
#include <stdint.h>

/* Darkness: white=0, black=1000; either electrical polarity is supported. */
bool s3_normalize(uint16_t raw, uint16_t white, uint16_t black,
                  uint16_t raw_max, uint16_t *p_darkness);

#endif
