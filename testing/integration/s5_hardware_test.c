/* Physical signal roles come from Buddy 5's original Grove wiring:
 * GP1 drives TRIG; GP0 receives ECHO. Exercise the production driver. */
#include "../../app_program/subsystem5/s5_hardware.h"
#include "../../app_program/subsystem2/subsystem2.h"
#include <stdio.h>
#include <stdlib.h>
#define EXPECT(c) do { if (!(c)) { \
    fprintf(stderr, "S5 hardware failed line %d: %s\n", __LINE__, #c); \
    exit(2); } } while (0)

enum { NORMAL, NO_RISE, STUCK_HIGH, NO_FALL, TOO_SHORT };
static UW clock_us, trigger_start, fired_at, accesses;
static UINT scenario, trigger_high, fired, echo_configured, trig_configured;
static BOOL tick_running, allow_tick = TRUE;
_Static_assert(BOARD_MOTORS_DISABLED == 1, "Sensor tests retain drive lock");

UW in_w(UW address)
{
    EXPECT(++accesses < 100000U);
    if (address == TIMER_TIMERAWL) return tick_running ? clock_us++ : clock_us;
    EXPECT(address == RESETS_RESET_DONE);
    return RESETS_RESET_TIMER;
}
void out_w(UW address, UW value)
{
    if (address == 0x4005802cU) {
        EXPECT(value == ((1U << 9) | 12U));
        tick_running = allow_tick;
    } else EXPECT(address == PWM_BASE + PWM_CHx_DIV + 6U * 0x14U);
}
void clr_w(UW address, UW value)
{
    EXPECT(address == RESETS_RESET && value == RESETS_RESET_TIMER);
}
ER gpio_set_pin(UINT pin, UINT mode)
{
    EXPECT((pin == 1U && mode == GPIO_MODE_OUT) ||
           (pin == 0U && mode == GPIO_MODE_IN));
    if (pin == 0U) echo_configured = 1;
    else trig_configured = 1;
    return E_OK;
}
ER gpio_set_val(UINT pin, UINT value)
{
    EXPECT(pin == 1U && value <= 1U);
    if (value) { EXPECT(!trigger_high); trigger_high = 1; trigger_start = clock_us; }
    else if (trigger_high) {
        EXPECT(clock_us - trigger_start >= 10U);
        trigger_high = 0; fired = 1; fired_at = clock_us;
    }
    return E_OK;
}
UINT gpio_get_val(UINT pin)
{
    EXPECT(pin == 0U && echo_configured && trig_configured);
    EXPECT(++accesses < 100000U);
    if (tick_running) ++clock_us;
    if (scenario == STUCK_HIGH) return 1;
    if (!fired || scenario == NO_RISE) return 0;
    UW elapsed = clock_us - fired_at;
    UW width = scenario == TOO_SHORT ? 50U : 580U;
    return elapsed >= 50U && (scenario == NO_FALL || elapsed < 50U + width);
}
ER pwm_set_cc(UINT pin, UW value)
{
    EXPECT(pin == 12U && value >= 750U && value <= 2250U); return E_OK;
}
ER pwm_set_wrap(UINT pin, UW value)
{
    EXPECT(pin == 12U && value == 19999U); return E_OK;
}
ER pwm_set_enabled(UINT pin, BOOL enabled)
{
    EXPECT(pin == 12U && (enabled == TRUE || enabled == FALSE)); return E_OK;
}
ER pwm_set_pin(UINT pin) { EXPECT(pin == 12U); return E_OK; }
ER s2_encoder_get_status(s2_encoder_status_t *status)
{
    *status = (s2_encoder_status_t){.valid = 1, .motor1_forward_counts = 10,
                                  .motor2_forward_counts = 20};
    return E_OK;
}
ER tk_get_otm(SYSTIM *time)
{
    time->hi = 0; time->lo = clock_us / 1000U; return E_OK;
}
static void reset(UINT mode, UW start)
{
    scenario = mode; clock_us = start; accesses = 0;
    trigger_high = fired = 0;
}
int main(void)
{
    EXPECT(s5_hw_timer_init() == E_OK);
    s5_hw_io_init();
    EXPECT(echo_configured && trig_configured && !trigger_high);
    UW mm, pulse;
    for (UINT wrap = 0; wrap < 2; ++wrap) {
        reset(NORMAL, wrap ? UINT32_MAX - 40U : 100U);
        EXPECT(s5_hw_measure_raw(&mm, &pulse) == S5_ECHO_OK);
        EXPECT(fired && !trigger_high && mm >= 98U && mm <= 102U);
        EXPECT(pulse >= 575U && pulse <= 585U);
        reset(NO_RISE, clock_us); mm = pulse = 123U;
        EXPECT(s5_hw_measure_raw(&mm, &pulse) == S5_ECHO_NO_RISE);
        EXPECT(mm == 0 && pulse == 0);
        reset(STUCK_HIGH, clock_us);
        EXPECT(s5_hw_measure_raw(&mm, &pulse) == S5_ECHO_STUCK_HIGH && !fired);
        reset(NO_FALL, clock_us);
        EXPECT(s5_hw_measure_raw(&mm, &pulse) == S5_ECHO_NO_FALL);
        EXPECT(mm == 0 && pulse > 24000U);
        reset(TOO_SHORT, clock_us);
        EXPECT(s5_hw_measure_raw(&mm, &pulse) == S5_ECHO_TOO_SHORT && mm == 0);
    }
    reset(NORMAL, 100U);
    s5_status_t status = {.state = S5_MONITOR};
    s5_hw_sample(&status);
    EXPECT(status.valid && status.encoder_valid && status.echo_result == S5_ECHO_OK);
    EXPECT(status.motor1_counts_start == 10 && status.motor1_counts_end == 10);
    EXPECT(status.distance_mm >= 98U && status.distance_mm <= 102U);
    reset(NO_RISE, clock_us);
    s5_hw_sample(&status);
    EXPECT(!status.valid && status.distance_mm == 0 && status.echo_result == S5_ECHO_NO_RISE);
    allow_tick = FALSE; tick_running = FALSE; accesses = 0;
    EXPECT(s5_hw_timer_init() == E_SYS);
    puts("INT-CR1-08: GP1 trigger/GP0 echo, valid pulse, faults, timer wrap/start failure, narrow sample PASS");
    return 0;
}
