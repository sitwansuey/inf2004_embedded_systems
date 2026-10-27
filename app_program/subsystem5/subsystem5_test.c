#include "subsystem5.h"
#if APP_TEST
EXPORT void subsystem5_test(void)
{
    const INT angles[] = {0, -90, 90, 0};
    const char *names[] = {"origin", "right 90 deg", "left 90 deg (180 deg sweep)", "return origin"};
    const char *echo[] = {"ok", "stuck-high", "no-rise", "no-fall", "too-short"};
    tm_printf((UB *)"[S5 TEST] sequence start; +left/-right; 1000 ms pause at each position\n");
    for (UINT i = 0; i < 4; ++i)
    {
        s5_status_t sample;
        BOOL take_sample = i == 1 || i == 2;
        tm_printf((UB *)"[S5 TEST] move=%s angle_deg=%d wait_ms=1000\n", names[i], angles[i]);
        ER er = s5_test_position(angles[i], take_sample, &sample);
        if (er < E_OK)
        {
            CHECK(er, "S5 test position");
            CHECK(s5_test_position(0, FALSE, &sample), "S5 test return origin");
            return;
        }
        if (take_sample)
        {
            tm_printf((UB *)"[SENSOR] source=S5 sample_t=%u angle_deg=%d "
                            "distance_mm=%u echo_us=%u valid=%d echo=%s\n",
                      sample.time_ms,
                      sample.angle_deg, sample.distance_mm, sample.echo_us, sample.valid, echo[sample.echo_result]);
        }
    }
    tm_printf((UB *)"[S5 TEST] sequence finished; servo at origin, motors stopped\n");
}
#endif
