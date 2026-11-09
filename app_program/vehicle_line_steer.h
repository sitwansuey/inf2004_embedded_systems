#ifndef VEHICLE_LINE_STEER_H
#define VEHICLE_LINE_STEER_H
#include <stdint.h>
/* Straddle line following: proportional-in-time differential steering.
 *
 * A side sensor touching the line used to command a full pivot (one wheel
 * stopped). Corrections lasted 100-300 ms, so every touch was a hard
 * brake/kick, swung the robot until the other sensor touched, and reset the
 * wheel-speed PID on every FORWARD<->PIVOT switch. Instead keep FORWARD and
 * slow the inner wheel: start gently and ramp up while the same sensor stays
 * on black. Only a sustained touch (a tight bend) escalates to the pivot.
 *
 * S2 steering units: positive raises the left wheel (turns right/CW), 400 is
 * the maximum (inner wheel at 60% of the base demand before the ceiling). */
#define LINE_STEER_PIVOT 0x7fffffff /* returned when the bend needs a pivot */

static inline int32_t line_steer_amount(uint32_t held_ms,int32_t minimum,int32_t maximum,
    uint32_t ramp_ms,uint32_t pivot_after_ms)
{
    if(pivot_after_ms!=0U && held_ms>=pivot_after_ms) return LINE_STEER_PIVOT;
    if(maximum<minimum) maximum=minimum;
    if(ramp_ms==0U || held_ms>=ramp_ms) return maximum;
    return minimum+(int32_t)((int64_t)(maximum-minimum)*held_ms/ramp_ms);
}
#endif
