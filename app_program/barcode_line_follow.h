/* Gentle outer line loop during GP28 capture. The wheel PI remains the inner
 * loop. Positive correction bends right; neither wheel stops or reverses. */
#ifndef CSC02_BARCODE_LINE_FOLLOW_H
#define CSC02_BARCODE_LINE_FOLLOW_H
#include <stdbool.h>
#include <stdint.h>
#define BARCODE_SCAN_MAX_STEERING 100 /* permille; per-wheel balance applied afterward */
typedef struct {
    int32_t filtered_error;
    int16_t correction;
    uint32_t sample_ms;
    bool primed;
} barcode_line_follow_t;
static inline int16_t barcode_line_follow_step(barcode_line_follow_t *s,
    uint32_t sample_ms,bool active,uint16_t left_dark,uint16_t right_dark,unsigned mask)
{
    if(!s)return 0;
    if(!active || left_dark>1000U || right_dark>1000U || mask>3U || mask==3U) {
        *s=(barcode_line_follow_t){0};return 0;
    }
    if(s->primed) {
        uint32_t dt=sample_ms-s->sample_ms;
        if(!dt)return s->correction; /* One filter update per real sample. */
        if(dt>=30U){*s=(barcode_line_follow_t){0};return 0;}
    }
    int32_t error=(int32_t)right_dark-(int32_t)left_dark;
    /* Ignore small white-floor differences; use calibrated analog contrast
     * before the hard black threshold is crossed. No integral wind-up. */
    error=error>80?error-80:error< -80?error+80:0;
    s->filtered_error=s->primed?(3*s->filtered_error+error)/4:error;
    s->sample_ms=sample_ms;s->primed=true;
    int32_t correction=s->filtered_error*120/1000;
    if(correction>BARCODE_SCAN_MAX_STEERING)correction=BARCODE_SCAN_MAX_STEERING;
    if(correction< -BARCODE_SCAN_MAX_STEERING)correction= -BARCODE_SCAN_MAX_STEERING;
    s->correction=(int16_t)correction;return s->correction;
}
#endif
