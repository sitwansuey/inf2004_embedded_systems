#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
static void
no_return (s5_status_t * s, s5_scan_context_t * c, s5_echo_result_t echo)
{
    s->sample_angle_deg = s->angle_deg;
    s->valid            = 0;
    s->distance_mm      = 0;
    s->echo_result      = echo;
    (void) s5_scan_step(s, c);
}
/* An object within +-20 deg, open space (no echo) beyond on the open sides.
 * Coarse search 20/40/60 deg left; the left's best clear angle mirrored on the
 * right; a blocked left searches 20/40/60 deg right instead. */
static void
route (int left, int right, s5_echo_result_t echo)
{
    s5_status_t       s  = {.state = S5_MONITOR, .encoder_valid = 1};
    s5_scan_context_t c  = {0};
    unsigned          lm = 0, rm = 0, steps = 0;
    for (; steps < 600 && s.state != S5_READY && s.state != S5_BLOCKED; ++steps)
    {
        int a              = s.angle_deg;
        s.sample_angle_deg = a;
        s.valid            = 1;
        s.distance_mm      = 150;
        s.echo_result      = S5_ECHO_OK;
        s.time_ms += S5_SAMPLE_MS;
        if ((a > 20 && left) || (a < -20 && right))
        {
            s.valid       = 0;
            s.distance_mm = 0;
            s.echo_result = echo;
        }
        if (s.state == S5_SCAN_LEFT)
        {
            lm |= 1U << (unsigned) (a / S5_SCAN_STEP_DEG - 1);
        }
        if (s.state == S5_SCAN_RIGHT && c.right_coarse)
        {
            rm |= 1U << (unsigned) (-a / S5_SCAN_STEP_DEG - 1);
        }
        s.angle_deg = s5_scan_step(&s, &c);
    }
    assert(steps < 600);
    /* Left clear at 40 deg; blocked through 60 deg: the right is searched
     * coarsely. */
    assert(lm == (left ? 3U : 7U) && rm == (left ? 0U : right ? 3U : 7U));
    if (!left && !right)
    {
        assert(s.state == S5_BLOCKED && !s.approach_plan_valid);
        return;
    }
    assert(s.state == S5_READY && s.approach_plan_valid && !s.valid &&
           !s.distance_mm);
    assert(s5_sample_is_open(&s) && s5_direction_clear(&s, 200));
    assert(s5_assess_path(&s, 1, s.time_ms).state == S5_PATH_READY);
    /* Both edges are known: measured, or the one measured edge mirrored. */
    assert(s.left_bound_valid && s.right_bound_valid &&
           s.left_bound_deg == 20 && s.right_bound_deg == -20);
    assert(s.object_width_valid && s.object_left_mm == 55 &&
           s.object_right_mm == 55);
    /* Only a measured side is planned; with both, the tie goes LEFT. */
    assert(left ? s.planned_rotate_deg > 0 : s.planned_rotate_deg < 0);
}
int
main (void)
{
    for (unsigned e = S5_ECHO_NO_RISE; e <= S5_ECHO_NO_FALL; ++e)
    {
        route(1, 0, (s5_echo_result_t) e);
        route(0, 1, (s5_echo_result_t) e);
        route(1, 1, (s5_echo_result_t) e);
        route(0, 0, (s5_echo_result_t) e);
        s5_status_t       s = {.state = S5_MONITOR};
        s5_scan_context_t c = {0};
        no_return(&s, &c, (s5_echo_result_t) e);
        assert(!s5_sample_is_open(&s));
        no_return(&s, &c, (s5_echo_result_t) e);
        assert(!s5_sample_is_open(&s));
        no_return(&s, &c, (s5_echo_result_t) e);
        assert(s5_sample_is_open(&s) && s.state == S5_MONITOR);
        assert(s5_assess_path(&s, 1, 0).state == S5_PATH_FORWARD);
        no_return(&s, &c, (s5_echo_result_t) e);
        assert(s5_sample_is_open(&s));
        s.angle_deg = s.sample_angle_deg = 30;
        no_return(&s, &c, (s5_echo_result_t) e);
        assert(!s5_sample_is_open(&s));
        s = (s5_status_t){.state = S5_APPROACH};
        c = (s5_scan_context_t){0};
        for (unsigned n = 0; n < 3; ++n)
        {
            no_return(&s, &c, (s5_echo_result_t) e);
        }
        assert(s5_sample_is_open(&s) &&
               s5_assess_path(&s, 1, 0).state == S5_PATH_GUIDE_FORWARD);
        s.sample_angle_deg = 30;
        assert(!s5_direction_clear(&s, 50));
    }
    for (unsigned e = S5_ECHO_STUCK_HIGH; e <= S5_ECHO_TOO_SHORT; e += 3)
    {
        s5_status_t       s = {.state = S5_MONITOR};
        s5_scan_context_t c = {0};
        for (unsigned n = 0; n < 4; ++n)
        {
            no_return(&s, &c, (s5_echo_result_t) e);
        }
        assert(!s5_sample_is_open(&s) && s.state == S5_FAULT);
    }
    puts("PASS: no-rise/no-fall clear after 3 settled samples; open left or "
         "right plans that side (one measured edge mirrored), neither blocks; "
         "READY verification; forward bypass guidance; angle reset; hard "
         "faults remain closed.");
}

/*** end of file ***/
