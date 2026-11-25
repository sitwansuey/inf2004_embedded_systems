#include <assert.h>
#include <math.h>
/* M_PI is not ISO C: glibc hides it under -std=c11 (no _DEFAULT_SOURCE),
 * and _USE_MATH_DEFINES only matters to Windows/MinGW headers. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include "../../app_program/vehicle_reverse_track.h"
#include "../../app_program/subsystem2/wheel_steering.h"

/* Same defaults as robot_config.h BOARD_BARCODE_RETRY_REVERSE_*. */
enum
{
    YAW_GAIN  = 40,
    LINE_GAIN = 200,
    LIMIT     = 300
};

static int32_t
steer (int32_t yaw, int32_t ref, unsigned mask, bool fresh)
{
    return reverse_track_steering(yaw, ref, mask, fresh, YAW_GAIN, LINE_GAIN,
                                  LIMIT);
}

static void
unit_checks (void)
{
    assert(reverse_track_wrap_mdeg(0) == 0);
    assert(reverse_track_wrap_mdeg(190000) == -170000);
    assert(reverse_track_wrap_mdeg(-190000) == 170000);
    assert(reverse_track_wrap_mdeg(359000) == -1000);
    /* Short way round across the odometry wrap. */
    assert(steer(-359000, 359000, 0, true) == -80);
    /* Heading term opposes the error; line term is inverted vs forward. */
    assert(steer(5000, 0, 0, true) == -200);
    assert(steer(-5000, 0, 0, true) == 200);
    assert(steer(0, 0, 1U, true) == LINE_GAIN);
    assert(steer(0, 0, 2U, true) == -LINE_GAIN);
    assert(steer(0, 0, 3U, true) == 0 && steer(0, 0, 0U, true) == 0);
    assert(steer(0, 0, 1U, false) == 0);
    assert(steer(20000, 0, 2U, true) == -LIMIT &&
           steer(-20000, 0, 1U, true) == LIMIT);
    /* EMA seeds, converges and averages across the wrap. */
    int32_t ref = reverse_track_yaw_ref(0, 10000, false, 50);
    assert(ref == 10000);
    for (int n = 0; n < 500; ++n)
    {
        ref = reverse_track_yaw_ref(ref, 20000, true, 50);
    }
    assert(ref > 19900 && ref <= 20000);
    ref = reverse_track_yaw_ref(179000, -179000, true, 2);
    assert(ref == -180000 || ref == 180000);
}

/* Closed loop on a straight line: reverse at 80 mm/s for 600 mm with the
 * firmware's integer law and S2's 2/ms steering slew. Sensors 60 mm ahead
 * of the axle at +/-12 mm, line 18 mm wide, track 113 mm. */
static double
run (double ref_error_deg, double y0_mm, double theta0_deg, double * max_theta)
{
    const double v = 80.0, track = 113.0, ahead = 60.0, half = 12.0,
                 line_half = 9.0, dt = 0.010;
    double       x = 0, y = y0_mm, th = theta0_deg * M_PI / 180.0, worst = 0;
    int32_t      filtered = 0;
    int32_t      ref      = (int32_t) lround(ref_error_deg * 1000.0);
    *max_theta            = 0;
    while (x > -600.0)
    {
        double   sy = y + ahead * sin(th), c = half * cos(th);
        unsigned mask = (fabs(sy - c) < line_half ? 1U : 0U) |
                        (fabs(sy + c) < line_half ? 2U : 0U);
        int32_t  yaw =
            reverse_track_wrap_mdeg((int32_t) lround(th * 180000.0 / M_PI));
        filtered = s2_steering_step(filtered, steer(yaw, ref, mask, true), 10U);
        double vl  = -v * (1000.0 + filtered) / 1000.0,
               vr  = -v * (1000.0 - filtered) / 1000.0;
        double fwd = (vl + vr) / 2.0, w = (vr - vl) / track;
        x += fwd * cos(th) * dt;
        y += fwd * sin(th) * dt;
        th += w * dt;
        double lateral = fabs(y + ahead * sin(th));
        if (lateral > worst)
        {
            worst = lateral;
        }
        if (fabs(th) * 180.0 / M_PI > *max_theta)
        {
            *max_theta = fabs(th) * 180.0 / M_PI;
        }
    }
    return worst;
}

int
main (void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    unit_checks();
    const double refs[] = {-5, -2, 0, 2, 5}, offsets[] = {-8, 0, 8},
                 headings[] = {-3, 0, 3};
    double       worst = 0, worst_theta = 0;
    unsigned     failures = 0;
    for (unsigned a = 0; a < 5; ++a)
    {
        for (unsigned b = 0; b < 3; ++b)
        {
            for (unsigned c = 0; c < 3; ++c)
            {
                double th;
                double lat = run(refs[a], offsets[b], headings[c], &th);
                if (lat > worst)
                {
                    worst = lat;
                }
                if (th > worst_theta)
                {
                    worst_theta = th;
                }
                if (lat >= 20.0 || th >= 15.0)
                {
                    ++failures;
                    printf("FAIL ref %+.0f deg, y0 %+.0f mm, heading %+.0f "
                           "deg: offset %.1f mm, heading %.1f deg\n",
                           refs[a], offsets[b], headings[c], lat, th);
                }
            }
        }
    }
    assert(failures == 0);
    printf("reverse_track: ok (worst sensor offset %.1f mm, worst heading %.1f "
           "deg)\n",
           worst, worst_theta);
    return 0;
}

/*** end of file ***/
