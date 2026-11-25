/*
 *    subsystem5_test.c
 *    On-target tests for subsystem 5.  Built only with: make TEST=1
 *    The integrated test image (make TEST=1 NET=lwip MQTT=1) runs them.
 *
 *    Passive: the path checks call the pure s5_assess_path() policy on
 *    local records; the live checks only copy the status the scan task
 *    publishes (5 ms lock wait) and round-trip that copy through the
 *    92-byte status codec in local buffers.  Nothing here moves the servo
 *    or triggers the sonar; the scan task alone owns the hardware.  The
 *    explicit S5 hardware diagnostics (APP_S5_SERVO_TEST and friends) are
 *    separate images and are not run from here.
 *
 *    Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *    console output to testing/subsystem5/logs/ and record the result
 *    in testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U16 to S5-U20 (new
 *    IDs: S5-U01 to S5-U15 have all been used before, see the TEST_LOG's
 *    ID-change notes).
 */
#include "subsystem5.h"
#include "s5_data.h"
#include "../test_report.h"

#if APP_TEST
/* The scan task centres the servo for S5_STARTUP_CENTER_MS before its
 * first sample, and only runs once the initial task blocks. */
#define S5_TEST_READY_MS (S5_STARTUP_CENTER_MS + 1500U)
#define S5_TEST_POLL_MS 50U

#if APP_S5_ENABLED && !APP_S5_DIAGNOSTICS
static uint8_t s5_test_wire[S5_DATA_SIZE];

/* The fields a consumer acts on survive encode then decode unchanged. */
static BOOL
s5_test_same (const s5_status_t * a, const s5_status_t * b)
{
    return a->state == b->state && a->angle_deg == b->angle_deg &&
           a->sample_angle_deg == b->sample_angle_deg &&
           a->scan_id == b->scan_id && a->distance_mm == b->distance_mm &&
           a->echo_us == b->echo_us && a->time_ms == b->time_ms &&
           a->valid == b->valid && a->echo_result == b->echo_result &&
           a->object_width_valid == b->object_width_valid &&
           a->object_width_mm == b->object_width_mm;
}
#endif

void
subsystem5_test (void)
{
    TEST_INIT();

    /* Path policy on fixed records (no hardware): the inclusive 200 mm
     * stop threshold and the 500 ms staleness limit. */
    s5_status_t p = {.state       = S5_MONITOR,
                     .valid       = TRUE,
                     .distance_mm = S5_OBSTACLE_MM + 1U,
                     .time_ms     = 1};
    TEST_CHECK("S5-U16", s5_assess_path(&p, TRUE, 1).state == S5_PATH_FORWARD,
               "fresh clear path allows forward");
    p.distance_mm = S5_OBSTACLE_MM;
    TEST_CHECK("S5-U17", s5_assess_path(&p, TRUE, 1).state == S5_PATH_STOP,
               "inclusive stop threshold");
    TEST_CHECK("S5-U18",
               s5_assess_path(&p, TRUE, 2U + S5_STALE_MS).state ==
                   S5_PATH_UNAVAILABLE,
               "stale reading is unavailable");

#if APP_S5_ENABLED && !APP_S5_DIAGNOSTICS
    {
        s5_status_t first   = {0};
        s5_status_t later   = {0};
        s5_status_t decoded = {0};
        ER          er;
        UINT        waited = 0U;

        /* S5-U19: the scan task publishes new samples (bounded wait past
         * the startup centring), whether or not an echo comes back */
        er    = s5_get_status(&first);
        later = first;
        while ((er == E_OK) && (later.time_ms == first.time_ms) &&
               (waited < S5_TEST_READY_MS))
        {
            tk_dly_tsk(S5_TEST_POLL_MS);
            waited += S5_TEST_POLL_MS;
            er = s5_get_status(&later);
        }
        TEST_CHECK("S5-U19",
                   (er == E_OK) && (later.time_ms != first.time_ms) &&
                       (later.state != S5_FAULT),
                   "scan task publishes fresh samples, no fault");

        /* S5-U20: the live status copy round-trips through the 92-byte
         * status codec on the target compiler */
        TEST_CHECK(
            "S5-U20",
            (er == E_OK) &&
                s5_data_encode(&later, s5_test_wire, sizeof(s5_test_wire)) &&
                s5_data_decode(s5_test_wire, sizeof(s5_test_wire), &decoded) &&
                s5_test_same(&later, &decoded),
            "live status round-trips through the status codec");
    }
#elif !APP_S5_ENABLED
    tm_printf((UB *) "[TEST] S5-U19 SKIP  S5 disabled in this profile "
                     "(GP0/GP1 used by the UART console)\n");
    tm_printf((UB *) "[TEST] S5-U20 SKIP  S5 disabled in this profile\n");
#else
    tm_printf((
        UB *) "[TEST] S5-U19 SKIP  explicit S5 diagnostic owns the hardware\n");
    tm_printf((
        UB *) "[TEST] S5-U20 SKIP  explicit S5 diagnostic owns the hardware\n");
#endif

    TEST_SUMMARY("subsystem5");
}
#endif

/*** end of file ***/
