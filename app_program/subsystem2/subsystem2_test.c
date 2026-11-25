/*
 * subsystem2_test.c
 * Buddy 2 -- On-target passive checks. Student ID: 2500613.
 * Built only with: make TEST=1.  The integrated test image
 * (make TEST=1 NET=lwip MQTT=1) and the S2_ONLY=1 TEST=1 bench image run
 * them from usermain() once the subsystems have started.
 *
 * PASSIVE: nothing here drives a motor.  The checks only read the motion
 * status and the wheel encoder counts.  The one send (S2-U33) is a STOP
 * from a task that is not the bound controller, which subsystem2.c must
 * reject before it touches anything.  The old raised-wheel motor test
 * (S2-U01 to S2-U14, removed 2026-11-20) is not restored: it moved the
 * wheels at power-up.
 *
 * Each check prints one "[TEST] S2-Uxx PASS|FAIL" line (../test_report.h).
 * Save the console output to testing/subsystem2/logs/ and record the result
 * in testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U29 to S2-U33 (new IDs:
 * S2-U01 to S2-U28 have all been used before, see the TEST_LOG's ID-change
 * notes).
 */
#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST
/* Bounded waits: the motor task (priority 14) only runs once the initial
 * task (priority 1) blocks, so the encoders may not be ready yet. */
#define S2_TEST_READY_MS 500U
#define S2_TEST_POLL_MS 20U
#define S2_TEST_REST_MS 300U

#if S2_ENCODER_ENABLED
static UW s2_test_wheel_total(const s2_encoder_status_t *p_status, BOOL motor1)
{
    return motor1 ?
        p_status->motor1_positive_pulses + p_status->motor1_negative_pulses :
        p_status->positive_pulses + p_status->negative_pulses;
}
#endif

EXPORT void subsystem2_test(void)
{
    s2_motion_status_t status = {0};
    ER er;

    TEST_INIT();

    /* S2-U29: the status API rejects a NULL output pointer */
    TEST_CHECK("S2-U29", s2_motion_get_status(NULL) == E_PAR,
               "motion status rejects NULL");

    /* S2-U30: at power-up (DISARMED, no motion command) the motor owner
     * reports STOP at 0% and the STOPPED state */
    er = s2_motion_get_status(&status);
    TEST_CHECK("S2-U30", (er == E_OK) &&
               (status.state == S2_MOTION_STOPPED) &&
               (status.command == S2_MOVE_STOP) &&
               (status.power_percent == 0U),
               "motor output is STOPPED at 0% after boot");

#if S2_ENCODER_ENABLED
    {
        s2_encoder_status_t first = {0};
        s2_encoder_status_t second = {0};
        UINT waited = 0U;

        /* S2-U31: the encoder snapshot becomes valid (bounded wait) */
        er = s2_encoder_get_status(&first);
        while ((er == E_OK) && (first.valid == 0U) &&
               (waited < S2_TEST_READY_MS))
        {
            tk_dly_tsk(S2_TEST_POLL_MS);
            waited += S2_TEST_POLL_MS;
            er = s2_encoder_get_status(&first);
        }
        TEST_CHECK("S2-U31", (er == E_OK) && (first.valid != 0U),
                   "wheel encoder snapshot is valid");

        /* S2-U32: with the motors stopped, neither wheel's pulse count
         * changes over 300 ms (robot at rest at power-up) */
        if ((er == E_OK) && (first.valid != 0U))
        {
            tk_dly_tsk(S2_TEST_REST_MS);
            er = s2_encoder_get_status(&second);
        }
        TEST_CHECK("S2-U32", (er == E_OK) && (second.valid != 0U) &&
                   (s2_test_wheel_total(&second, TRUE) ==
                    s2_test_wheel_total(&first, TRUE)) &&
                   (s2_test_wheel_total(&second, FALSE) ==
                    s2_test_wheel_total(&first, FALSE)),
                   "encoder counts stay still at rest");
    }
#else
    {
        s2_encoder_status_t encoder;

        TEST_CHECK("S2-U31", s2_encoder_get_status(&encoder) == E_NOSPT,
                   "encoders disabled: snapshot reports E_NOSPT");
        tm_printf((UB *)"[TEST] S2-U32 SKIP  encoders disabled "
                  "(S2_ENCODER_ENABLED=0)\n");
    }
#endif

#if !APP_S2_ONLY
    /* S2-U33: only the bound controller (the vehicle task) may command
     * the motors; a STOP from this task is refused with E_OACV */
    TEST_CHECK("S2-U33", s2_movement_send(S2_MOVE_STOP, 0U) == E_OACV,
               "motion command from a non-controller task is refused");
#else
    tm_printf((UB *)"[TEST] S2-U33 SKIP  S2_ONLY=1 has no bound controller\n");
#endif

    TEST_SUMMARY("subsystem2");
}
#endif
