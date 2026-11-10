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
/* While a barcode is read the course (cross/corner/pivot logic) is off, but
 * the line must still be followed: a single straddle sensor on black (mask 1
 * right, 2 left) steers gently toward it, capped at maximum and never a pivot;
 * both or neither on black (bars, centred) is straight. maximum 0: off. */
static inline int32_t line_marker_steer(unsigned mask,uint32_t held_ms,int32_t minimum,
    int32_t maximum,uint32_t ramp_ms)
{
    if(maximum<=0 || (mask!=1U && mask!=2U)) return 0;
    if(minimum>maximum) minimum=maximum;
    int32_t steer=line_steer_amount(held_ms,minimum,maximum,ramp_ms,0U);
    return mask==1U?steer:-steer;
}
#endif
