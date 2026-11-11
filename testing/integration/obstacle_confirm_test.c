#include "../../app_program/vehicle_obstacle_confirm.h"
#include <assert.h>
#include <stdio.h>

static bool step(vehicle_obstacle_confirm_t *c, uint32_t ms, bool near)
{
    return vehicle_obstacle_confirm_step(c, true, ms, near, 3U);
}

int main(void)
{
    vehicle_obstacle_confirm_t c = {0};
    /* Three distinct near samples confirm, on the third and not before; a
     * sample re-read by later vehicle ticks counts once. */
    assert(!step(&c, 0U, true));  /* time 0 is a real first sample */
    assert(!step(&c, 0U, true) && !step(&c, 0U, true));
    assert(!step(&c, 65U, true));
    assert(step(&c, 130U, true));
    assert(step(&c, 130U, true)); /* holds while that sample repeats */
    assert(step(&c, 195U, true)); /* and while near samples continue */
    /* Any other new sample breaks the streak: far/open/no echo/off-axis. */
    assert(!step(&c, 260U, false));
    assert(!step(&c, 325U, true) && !step(&c, 390U, true));
    assert(!step(&c, 455U, false)); /* two near, then a dropout */
    assert(!step(&c, 520U, true) && !step(&c, 585U, true) && step(&c, 650U, true));
    /* A failed status read neither counts nor resets. */
    c = (vehicle_obstacle_confirm_t){0};
    assert(!step(&c, 10U, true) && !step(&c, 75U, true));
    assert(!vehicle_obstacle_confirm_step(&c, false, 140U, true, 3U));
    assert(!vehicle_obstacle_confirm_step(&c, false, 140U, false, 3U));
    assert(step(&c, 140U, true));
    /* Sample times wrap; the count saturates; required 0 never confirms. */
    c = (vehicle_obstacle_confirm_t){0};
    assert(!step(&c, UINT32_MAX - 64U, true) && !step(&c, UINT32_MAX, true) && step(&c, 64U, true));
    for (uint32_t n = 0U; n < 300U; ++n) assert(step(&c, 1000U + n * 65U, true));
    assert(c.streak == 255U);
    assert(!vehicle_obstacle_confirm_step(&c, true, 99999U, true, 0U));
    puts("Obstacle confirmation: 3 consecutive new near samples, re-reads count once, any other sample resets PASS");
    return 0;
}
