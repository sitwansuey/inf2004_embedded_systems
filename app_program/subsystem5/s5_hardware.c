#include "../robot_config.h"
#if APP_S5_ENABLED
/* Ultrasonic timing and servo PWM; no scan or motor decisions. */
#include "s5_hardware.h"
#include <tk/syslib.h>
#include "../subsystem2/subsystem2.h"
#if !BOARD_UART0_FREE
#error "S5 uses GP0/GP1: build with CONSOLE=usb_cdc CRASH_REPORT=0"
#endif
/* Timer is a free-running 1 MHz counter; unsigned subtraction handles wrap. */
static UW micros(void) { return in_w(TIMER_TIMERAWL); }
static void wait_us(UW us)
{
    UW start = micros();
    while (micros() - start < us)
    {
    }
}
static BOOL wait_echo(UINT level, UW start)
{
    while (gpio_get_val(BOARD_ULTRASONIC_ECHO_GPIO) != level)
    {
        if (micros() - start >= 25000U)
            return FALSE;
    }
    return TRUE;
}
EXPORT s5_echo_result_t s5_hw_measure_raw(UW *mm, UW *echo_us)
{
    *mm = 0;
    if (echo_us) *echo_us = 0;
    if (gpio_get_val(BOARD_ULTRASONIC_ECHO_GPIO))
        return S5_ECHO_STUCK_HIGH;
    gpio_set_val(BOARD_ULTRASONIC_TRIG_GPIO, 1);
    wait_us(10);
    gpio_set_val(BOARD_ULTRASONIC_TRIG_GPIO, 0);
    UW start = micros();
    if (!wait_echo(1, start))
        return S5_ECHO_NO_RISE;
    UW rise = micros();
    if (!wait_echo(0, start)) {
        if (echo_us) *echo_us = micros() - rise; /* Partial pulse: timed out high. */
        return S5_ECHO_NO_FALL;
    }
    UW pulse = micros() - rise;
    if (echo_us) *echo_us = pulse;
    if (pulse < 100U)
        return S5_ECHO_TOO_SHORT;
    *mm = pulse * 10U / 58U; /* datasheet: echo microseconds / 58 = cm */
    return S5_ECHO_OK;
}

EXPORT s5_echo_result_t s5_hw_measure(UW *mm)
{
    return s5_hw_measure_raw(mm, NULL);
}

/* This kernel uses SysTick, so it never starts the RP2040 hardware timer.
 * Start its watchdog tick source at 1 MHz (12 MHz clk_ref / 12).
 * This enables the tick generator only, not watchdog resets. */
static ER sensor_timer_init(void)
{
    const UW watchdog_tick = 0x4005802cU;
    out_w(watchdog_tick, (1U << 9) | XOSC_MHz);
    clr_w(RESETS_RESET, RESETS_RESET_TIMER);
    UW attempts = 10000;
    while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
    {
        if (--attempts == 0)
            return E_SYS;
    }
    UW start = micros();
    for (attempts = 0; attempts < 10000; ++attempts)
    {
        if (micros() != start)
            return E_OK;
    }
    return E_SYS; /* Fail startup instead of hanging in wait_us(). */
}
EXPORT void s5_hw_servo(INT left_deg)
{
    /* 125 MHz / 125 = 1 MHz; 20 ms frame, 750..2250 us commanded pulse range.
     * Slice 6 belongs exclusively to S5; GP13 must not use this slice. */
    if (left_deg < -90) left_deg=-90;
    if (left_deg > 90) left_deg=90;
    pwm_set_cc(BOARD_SERVO_GPIO, 1500 + (750 * left_deg) / 90);
}

EXPORT ER s5_hw_timer_init(void)
{
    return sensor_timer_init();
}

EXPORT void s5_hw_io_init(void)
{
    gpio_set_val(BOARD_ULTRASONIC_TRIG_GPIO, 0);
    gpio_set_pin(BOARD_ULTRASONIC_TRIG_GPIO, GPIO_MODE_OUT);
    gpio_set_pin(BOARD_ULTRASONIC_ECHO_GPIO, GPIO_MODE_IN);
    pwm_set_enabled(BOARD_SERVO_GPIO, FALSE);
    out_w(PWM_BASE + PWM_CHx_DIV + ((BOARD_SERVO_GPIO >> 1) & 7) * 0x14, 125U << 4);
    pwm_set_wrap(BOARD_SERVO_GPIO, 19999);
    s5_hw_servo(0);
    pwm_set_pin(BOARD_SERVO_GPIO);
    pwm_set_enabled(BOARD_SERVO_GPIO, TRUE);
}


/* Pair the echo with wheel snapshots to expose movement during acquisition. */
EXPORT void s5_hw_sample(s5_status_t *sample)
{
    s2_encoder_status_t before = {0}, after = {0};
    SYSTIM start, end;
    ER first = s2_encoder_get_status(&before);
    tk_get_otm(&start);
    UW distance=0, pulse=0;
    sample->echo_result = s5_hw_measure_raw(&distance, &pulse);
    sample->distance_mm = distance <= 5000U ? (uint16_t)distance : 0;
    sample->echo_us = pulse <= 25000U ? (uint16_t)pulse : 25000U;
    ER last = s2_encoder_get_status(&after);
    tk_get_otm(&end);
    sample->motor1_counts_start = before.motor1_forward_counts;
    sample->motor2_counts_start = before.motor2_forward_counts;
    sample->motor1_counts_end = after.motor1_forward_counts;
    sample->motor2_counts_end = after.motor2_forward_counts;
    sample->encoder_start_ms = start.lo;
    sample->encoder_end_ms = end.lo;
    sample->encoder_valid = first == E_OK && last == E_OK && before.valid && after.valid;
    sample->time_ms = end.lo;
    sample->sample_angle_deg = sample->angle_deg;
    sample->valid = sample->echo_result == S5_ECHO_OK && sample->distance_mm > 0 && sample->distance_mm <= 5000;
}

#endif
