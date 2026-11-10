#include <assert.h>
#include <stdio.h>
/* robot_config.h first, as subsystem2.c does, so wheel_steering.h sees the
 * robot's slew rate. */
#include "../../app_program/robot_config.h"
#include "../../app_program/vehicle_line_steer.h"
#include "../../app_program/subsystem2/wheel_steering.h"
/* The robot_config.h defaults; a change there must update this test. */
_Static_assert(BOARD_LINE_STEER_MIN==250 && BOARD_LINE_STEER_MAX==400 &&
               BOARD_LINE_STEER_RAMP_MS==150U && BOARD_LINE_PIVOT_AFTER_MS==450U &&
               BOARD_BARCODE_READ_LINE_STEER==250 && S2_STEER_SLEW_PER_MS==4U,
               "line steering defaults changed");
static int32_t amount(uint32_t held) {
    return line_steer_amount(held,BOARD_LINE_STEER_MIN,BOARD_LINE_STEER_MAX,
                             BOARD_LINE_STEER_RAMP_MS,BOARD_LINE_PIVOT_AFTER_MS);
}

int main(void) {
    /* Firm first correction, quick linear ramp, saturation, then pivot. */
    assert(amount(0U)==250);
    assert(amount(75U)==325);
    assert(amount(150U)==400 && amount(449U)==400);
    assert(amount(450U)==LINE_STEER_PIVOT && amount(5000U)==LINE_STEER_PIVOT);
    /* S2 applies the steer at 4 units/ms: full 400 within 100 ms of a touch. */
    assert(s2_steering_step(0,400,50U)==200 && s2_steering_step(0,400,100U)==400);
    assert(s2_steering_step(400,-400,100U)==0 && s2_steering_step(-10,-30,10U)==-30);
    /* Pivot disabled, no ramp, inverted bounds. */
    assert(line_steer_amount(100000U,150,400,300U,0U)==400);
    assert(line_steer_amount(0U,150,400,0U,700U)==400);
    assert(line_steer_amount(0U,300,100,300U,700U)==300);
    /* Within S2's accepted range, and the inner wheel keeps driving (no pivot
     * stop): at 30% demand under the 35% ceiling. */
    for(uint32_t t=0U;t<BOARD_LINE_PIVOT_AFTER_MS;t+=10U) {
        int32_t s=amount(t);
        assert(s>=250 && s<=400);
        unsigned outer=s2_steered_demand_milli(30U,s,0U,35U);
        unsigned inner=s2_steered_demand_milli(30U,s,1U,35U);
        assert(outer<=35000U && inner>=14000U && inner<outer);
    }
    /* Barcode reads: a single sensor on black steers toward it (mask 1 right
     * = positive, 2 left = negative), capped and never a pivot; bars under
     * both sensors or none are straight; maximum 0 disables it. */
    assert(line_marker_steer(1U,0U,250,250,150U)==250 && line_marker_steer(2U,0U,250,250,150U)==-250);
    assert(line_marker_steer(1U,100000U,250,250,150U)==250); /* no pivot however long */
    assert(line_marker_steer(1U,0U,100,300,200U)==100 && line_marker_steer(1U,100U,100,300,200U)==200);
    assert(line_marker_steer(2U,500U,100,300,200U)==-300);
    assert(line_marker_steer(1U,0U,400,250,150U)==250); /* minimum above the cap */
    assert(!line_marker_steer(0U,0U,250,250,150U) && !line_marker_steer(3U,0U,250,250,150U));
    assert(!line_marker_steer(1U,0U,250,0,150U) && !line_marker_steer(2U,999U,250,-5,150U));
    printf("line_steer: ok\n");
    return 0;
}
