/* Private S5 bound-search context; main uses only subsystem5.h. */
#ifndef S5_SCAN_H
#define S5_SCAN_H
#include "subsystem5.h"
#include "s5_width.h"
typedef struct
{
    INT left_clear, right_clear;
    UINT acquisition_samples;
    UINT echo_failures;
    BOOL averaging_bound;
    UINT bound_samples;
    UW bound_sum_mm;
    INT origin_left, origin_right;
    uint8_t position_recorded : 1;
    uint8_t plan_pending : 1;
} s5_scan_context_t;
IMPORT INT s5_scan_step(s5_status_t *next, s5_scan_context_t *scan);
#endif
