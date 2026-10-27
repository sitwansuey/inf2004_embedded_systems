/* Forward to 15 cm, find only the left bound, then beep and stay stopped. */
#include "subsystem5.h"
#include "s5_hardware.h"
#include "s5_scan.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST && APP_ACQUISITION_LEFT_TEST
#include "../board_pins.h"
static void left_test_beep(void)
{
    if (pwm_set_pin(BOARD_BUZZER_GPIO) < E_OK) return;
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

EXPORT void acquisition_left_test(void)
{
    s5_status_t sample = {.state = S5_MONITOR};
    s5_scan_context_t scan = {0};
    s2_move_command_t movement = S2_MOVE_STOP;
    UINT confirmations = 0;
    SYSTIM start, now;
    s2_controller_bind();
    if (s2_movement_send(S2_MOVE_STOP, 0) < E_OK) return;
    s5_hw_servo(0);
    tk_dly_tsk(S5_STARTUP_CENTER_MS);
    tk_get_otm(&start);
    tm_printf((UB *)"[LEFT TEST] forward to 15 cm, then scan left up to 50 deg\n");
    for (;;) {
        tk_get_otm(&now);
        if (now.lo - start.lo >= 30000U) break;
        sample.distance_mm = 0;
        s5_hw_sample(&sample);
        sample.valid = sample.echo_result == S5_ECHO_OK && sample.distance_mm > 0 && sample.distance_mm <= 5000;
        sample.sample_angle_deg = sample.angle_deg;
        tm_printf((UB *)"[LEFT TEST] angle=%d distance_mm=%u echo_us=%u valid=%d echo=%d\n",
                  sample.angle_deg, sample.distance_mm, sample.echo_us, sample.valid, sample.echo_result);
        INT target = sample.angle_deg;
        if (sample.state == S5_MONITOR) {
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
        } else {
            target = s5_scan_step(&sample, &scan);
            if (sample.left_bound_valid) {
                if (s2_movement_send(S2_MOVE_STOP, 0) < E_OK) return;
                tm_printf((UB *)"[LEFT TEST] PASS: left_bound_deg=%d average_mm=%u\n", sample.left_bound_deg, sample.left_bound_distance_mm);
                left_test_beep();
                return; /* Stop here, before the right-bound search. */
            }
            if (sample.state == S5_FAULT || sample.state == S5_BLOCKED) break;
        }
        if (target != sample.angle_deg) {
            sample.angle_deg = target;
            s5_hw_servo(target);
            tk_dly_tsk(S5_SETTLE_MS); /* Every left scan move is at most 10 deg. */
        } else tk_dly_tsk(S5_SAMPLE_MS);
    }
    CHECK(s2_movement_send(S2_MOVE_STOP, 0), "left test stop");
    tm_printf((UB *)"[LEFT TEST] FAIL: invalid reading, no left bound, or timeout\n");
}
#endif
