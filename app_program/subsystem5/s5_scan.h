/* Private S5 bound-search context; main uses only subsystem5.h. */
#ifndef S5_SCAN_H
#define S5_SCAN_H
#include "subsystem5.h"
#include "s5_width.h"
typedef struct
{
    INT left_clear, right_clear;
    UINT acquisition_samples;
    UINT motion_restarts;
    UINT echo_failures;
    INT echo_angle;
    BOOL echo_angle_valid;
    INT fine_hit, fine_clear;
    UINT fine_checks;
    BOOL averaging_bound;
    UINT bound_samples;
    UW bound_sum_mm;
    INT origin_left, origin_right;
    uint8_t position_recorded : 1;
    uint8_t plan_pending : 1;
} s5_scan_context_t;
/* Unsigned subtraction preserves encoder wrap; compare before abs(INT_MIN). */
static inline BOOL s5_encoder_displaced(INT a, INT b)
{
    int32_t delta=(int32_t)((uint32_t)a-(uint32_t)b);
    return delta>S5_SCAN_MOTION_TOLERANCE_COUNTS || delta< -S5_SCAN_MOTION_TOLERANCE_COUNTS;
}
IMPORT INT s5_scan_step(s5_status_t *next, s5_scan_context_t *scan);
#endif
