/* Forward to 15 cm, find both bounds; beep once for left and twice for right. */
#include "subsystem5.h"
#include "s5_hardware.h"
#include "s5_scan.h"
#include "s5_policy.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST && (APP_ACQUISITION_TEST || APP_S5_MASTER_TEST)
#include "../board_pins.h"
static void left_test_beep(void)
{
    if (pwm_set_pin(BOARD_BUZZER_GPIO) < E_OK)
        return;
    pwm_set_enabled(BOARD_BUZZER_GPIO, FALSE);
    /* Slice 3: 1 MHz clock, 2 kHz tone, 50% duty for half a second. */
    out_w(PWM_BASE + PWM_CHx_DIV + 3U * 0x14U, 125U << 4);
    pwm_set_wrap(BOARD_BUZZER_GPIO, 499);
    pwm_set_cc(BOARD_BUZZER_GPIO, 250);
    pwm_set_enabled(BOARD_BUZZER_GPIO, TRUE);
    tk_dly_tsk(500);
    pwm_set_cc(BOARD_BUZZER_GPIO, 0);
    pwm_set_enabled(BOARD_BUZZER_GPIO, FALSE);
}

EXPORT BOOL acquisition_test_run(s5_status_t *result)
{
    s5_status_t sample = {.state = S5_MONITOR};
    s5_scan_context_t scan = {0};
    s2_move_command_t movement = S2_MOVE_STOP;
    UINT confirmations = 0;
    BOOL left_announced = FALSE, right_announced = FALSE;
    SYSTIM start, now;
    s2_controller_bind();
    if (s2_movement_send(S2_MOVE_STOP, 0) < E_OK)
        return FALSE;
    s5_hw_servo(0);
    tk_dly_tsk(S5_STARTUP_CENTER_MS);
    tk_get_otm(&start);
    tm_printf((UB *)"[ACQUISITION TEST] forward to 15 cm, then acquire left and right bounds\n");
    for (;;)
    {
        tk_get_otm(&now);
        if (now.lo - start.lo >= 30000U) {
            tm_printf((UB *)"[ACQUISITION TEST] timeout: left_valid=%d right_valid=%d average_samples=%u\n",
                      sample.left_bound_valid, sample.right_bound_valid, scan.bound_samples);
            break;
        }
        sample.distance_mm = 0;
        s5_hw_sample(&sample);
        sample.valid = sample.echo_result == S5_ECHO_OK && sample.distance_mm > 0 && sample.distance_mm <= 5000;
        sample.sample_angle_deg = sample.angle_deg;
        tm_printf((UB *)"[ACQUISITION TEST] angle=%d distance_mm=%u echo_us=%u valid=%d echo=%d\n",
                  sample.angle_deg, sample.distance_mm, sample.echo_us, sample.valid, sample.echo_result);
        tm_printf((UB *)"[ACQUISITION ENCODER] start_ms=%u end_ms=%u valid=%d "
                  "motor1_start=%d motor1_end=%d motor2_start=%d motor2_end=%d\n",
                  sample.encoder_start_ms, sample.encoder_end_ms, sample.encoder_valid,
                  sample.motor1_counts_start, sample.motor1_counts_end,
                  sample.motor2_counts_start, sample.motor2_counts_end);
        INT target = sample.angle_deg;
        if (sample.state == S5_MONITOR)
        {
            if (!sample.valid) break;
            BOOL close = sample.distance_mm <= 150U;
            s2_move_command_t next = close ? S2_MOVE_STOP : S2_MOVE_FORWARD;
            if (next != movement && s2_movement_send(next, close ? 0 : 30) < E_OK) break;
            movement = next;
            confirmations = close ? confirmations + 1 : 0;
            if (confirmations >= S5_ACQUISITION_SAMPLES) {
                sample.state = S5_SCAN_LEFT;
                sample.obstacle_distance_mm = sample.distance_mm;
                target = S5_SCAN_STEP_DEG;
            }
        }
        else
        {
            target = s5_scan_step(&sample, &scan);
            if (sample.acquisition_clear && !sample.valid)
                tm_printf((UB *)"[ACQUISITION TEST] no echo assumed clear; next_angle=%d\n", target);
            if (sample.state == S5_FAULT || sample.state == S5_BLOCKED ||
                target < -S5_MAX_ANGLE || target > S5_MAX_ANGLE) {
                tm_printf((UB *)"[ACQUISITION TEST] stopped: state=%d echo=%d angle=%d left_valid=%d right_valid=%d\n",
                          sample.state, sample.echo_result, sample.angle_deg,
                          sample.left_bound_valid, sample.right_bound_valid);
                break;
            }
            if (sample.left_bound_valid && !left_announced)
            {
                tm_printf((UB *)"[ACQUISITION TEST] left_bound_deg=%d average_mm=%u; beep once\n", sample.left_bound_deg, sample.left_bound_distance_mm);
                left_test_beep();
                left_announced = TRUE;
                target = -sample.left_bound_deg; /* Start right at the mirrored left bound. */
            }
            if (sample.right_bound_valid && !right_announced)
            {
                if (s2_movement_send(S2_MOVE_STOP, 0) < E_OK)
                    return FALSE;
                tm_printf((UB *)"[ACQUISITION TEST] PASS: left=%d right=%d left_average_mm=%u right_average_mm=%u width_mm=%u; beep twice\n",
                          sample.left_bound_deg, sample.right_bound_deg, sample.left_bound_distance_mm,
                          sample.right_bound_distance_mm, sample.object_width_mm);
                left_test_beep();
                tk_dly_tsk(200);
                left_test_beep();
                right_announced = TRUE;
                if (!result) return TRUE;
                /* Both bounds and averages are complete; hand off directly. */
                if (!s5_acquisition_plan(&sample)) {
                    tm_printf((UB *)"[ACQUISITION TEST] invalid approach geometry: clearing=%d depth_mm=%u\n",
                              sample.avoidance_deg, sample.obstacle_distance_mm);
                    return FALSE;
                }
                sample.state = S5_READY;
                *result = sample;
                return TRUE;
            }
            if (sample.state == S5_READY) {
                *result = sample;
                return TRUE;
            }
        }
        if (target != sample.angle_deg)
        {
            INT travelled = target - sample.angle_deg;
            if (travelled < 0)
                travelled = -travelled;
            sample.angle_deg = target;
            s5_hw_servo(target);
            UW settle = (UW)travelled * S5_SERVO_MS_PER_DEG;
            tk_dly_tsk(settle < S5_SETTLE_MS ? S5_SETTLE_MS : settle);
            tk_dly_tsk(S5_SAMPLE_MS); /* Extra measurement-cycle quiet pause before measuring. */
        }
        else tk_dly_tsk(S5_SAMPLE_MS);
    }
    CHECK(s2_movement_send(S2_MOVE_STOP, 0), "left test stop");
    tm_printf((UB *)"[ACQUISITION TEST] FAIL: invalid reading, missing bound, or timeout\n");
    return FALSE;
}

EXPORT void aqcuisition_test(void)
{
    acquisition_test_run(NULL);
}
#endif
