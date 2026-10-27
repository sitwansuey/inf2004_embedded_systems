/* Native register/GPIO boundary for the actual S5 hardware driver. */
#ifndef S5_HARDWARE_HOST_H
#define S5_HARDWARE_HOST_H
#include "host.h"
INT tm_printf(UB *format, ...);
typedef struct { void *next; } T_MSG;
#define GPIO_MODE_OUT 1U
#define TIMER_TIMERAWL 0x40054028U
#define RESETS_RESET 0x4000c000U
#define RESETS_RESET_DONE 0x4000c008U
#define RESETS_RESET_TIMER (1U << 21)
#define PWM_BASE 0x40050000U
#define PWM_CHx_DIV 4U
#define XOSC_MHz 12U
UW in_w(UW address);
void out_w(UW address, UW value);
void clr_w(UW address, UW value);
ER gpio_set_val(UINT pin, UINT value);
ER pwm_set_cc(UINT pin, UW value);
ER pwm_set_wrap(UINT pin, UW value);
ER pwm_set_enabled(UINT pin, BOOL enabled);
ER pwm_set_pin(UINT pin);
#endif
