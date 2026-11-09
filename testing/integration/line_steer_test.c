#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_line_steer.h"
#include "../../app_program/subsystem2/wheel_steering.h"

/* Same defaults as robot_config.h BOARD_LINE_STEER_* / PIVOT_AFTER_MS. */
static int32_t amount(uint32_t held) { return line_steer_amount(held,150,400,300U,700U); }

int main(void) {
    /* Gentle first correction, linear ramp, saturation, then pivot. */
    assert(amount(0U)==150);
    assert(amount(150U)==275);
    assert(amount(300U)==400 && amount(699U)==400);
    assert(amount(700U)==LINE_STEER_PIVOT && amount(5000U)==LINE_STEER_PIVOT);
    /* Pivot disabled, no ramp, inverted bounds. */
    assert(line_steer_amount(100000U,150,400,300U,0U)==400);
    assert(line_steer_amount(0U,150,400,0U,700U)==400);
    assert(line_steer_amount(0U,300,100,300U,700U)==300);
    /* Within S2's accepted range, and the inner wheel keeps driving (no pivot
     * stop): at 30% demand under the 35% ceiling. */
    for(uint32_t t=0U;t<700U;t+=10U) {
        int32_t s=amount(t);
        assert(s>=150 && s<=400);
        unsigned outer=s2_steered_demand_milli(30U,s,0U,35U);
        unsigned inner=s2_steered_demand_milli(30U,s,1U,35U);
        assert(outer<=35000U && inner>=14000U && inner<outer);
    }
    printf("line_steer: ok\n");
    return 0;
}
