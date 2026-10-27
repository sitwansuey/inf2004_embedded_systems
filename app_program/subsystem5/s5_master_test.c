/* Master suite: acquisition feeds its measured plan into approach. */
#include "subsystem5.h"
#include "s5_hardware.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST && APP_S5_MASTER_TEST
/* Approach is open-loop: acquisition has already fixed the decisions. */
static BOOL approach_motion(s2_move_command_t command, UW duration)
{
    if (duration > S5_NAVIGATION_TIMEOUT_MS) return FALSE;
    if (s2_movement_send(command, 30) < E_OK) return FALSE;
    tk_dly_tsk(duration);
    if (s2_movement_send(S2_MOVE_STOP, 0) < E_OK) return FALSE;
    tk_dly_tsk(S5_SAMPLE_MS);
    return TRUE;
}

EXPORT void s5_master_test(void)
{
    s5_status_t result;
    tm_printf((UB *)"[S5 MASTER TEST] 1: acquisition\n");
    if (!acquisition_test_run(&result)) {
        tm_printf((UB *)"[S5 MASTER TEST] FAIL: acquisition; approach skipped\n");
        return;
    }
    /* Acquisition compares tangent headings from both averaged edge ranges
     * and passes the smaller feasible signed rotation into approach. */
    tm_printf((UB *)"[S5 MASTER TEST] 2: approach; clearing=%d rotate=%d cruise_mm=%u\n",
              result.avoidance_deg, result.planned_rotate_deg, result.planned_approach_mm);
    s5_hw_servo(0);
    INT travel = result.angle_deg < 0 ? -result.angle_deg : result.angle_deg;
    tk_dly_tsk((UW)travel * S5_SERVO_MS_PER_DEG + S5_SETTLE_MS + S5_SAMPLE_MS);
    s2_move_command_t rotate = result.planned_rotate_deg > 0 ? S2_MOVE_LEFT : S2_MOVE_RIGHT;
    s2_move_command_t reverse_rotate = result.planned_rotate_deg > 0 ? S2_MOVE_RIGHT : S2_MOVE_LEFT;
    UW cruise_ms = (result.planned_approach_mm * 1000U + S5_CRUISE_MM_PER_SECOND - 1U) / S5_CRUISE_MM_PER_SECOND;
    if (!approach_motion(rotate, result.planned_rotate_ms) ||
        !approach_motion(S2_MOVE_FORWARD, cruise_ms) ||
        !approach_motion(reverse_rotate, result.planned_rotate_ms)) {
        CHECK(s2_movement_send(S2_MOVE_STOP, 0), "master test stop");
        tm_printf((UB *)"[S5 MASTER TEST] FAIL: approach stopped\n");
        return;
    }
    tm_printf((UB *)"[S5 MASTER TEST] PASS: acquisition and approach; facing initial heading, stopped\n");
}
#endif
