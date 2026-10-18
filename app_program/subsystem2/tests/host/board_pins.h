/* Simulated pins/register addresses; not a board configuration. */
#ifndef S2_HOST_BOARD_PINS_H
#define S2_HOST_BOARD_PINS_H

#include "app.h"

#define BOARD_MOTOR1_A_GPIO 8U
#define BOARD_MOTOR1_B_GPIO 9U
#define BOARD_MOTOR2_A_GPIO 10U
#define BOARD_MOTOR2_B_GPIO 11U
#ifndef BOARD_MOTOR1_ENCODER_A_GPIO
#define BOARD_MOTOR1_ENCODER_A_GPIO 16U
#endif
#ifndef BOARD_MOTOR1_ENCODER_B_GPIO
#define BOARD_MOTOR1_ENCODER_B_GPIO 17U
#endif
#ifndef BOARD_MOTOR2_ENCODER_A_GPIO
#define BOARD_MOTOR2_ENCODER_A_GPIO 4U
#endif
#ifndef BOARD_MOTOR2_ENCODER_B_GPIO
#define BOARD_MOTOR2_ENCODER_B_GPIO 5U
#endif
#define IO_BANK0_BASE 0x1000U
#define RESETS_RESET 0x2000U
#define RESETS_RESET_DONE 0x2004U
#define RESETS_RESET_PWM 1U
#define GPIO_IN 0x3000U
#define GPIO(pin) (0x4000U + (pin) * 4U)
#define GPIO_PUE 1U
#define GPIO_PDE 2U
#define GPIO_SHEMITT 4U
#define GPIO_MODE_IN 0U
#define GPIO_MODE_OUT 1U

UW in_w(UW address);
void out_w(UW address, UW value);
void set_w(UW address, UW value);
void clr_w(UW address, UW value);
ER gpio_set_pin(UINT pin, UINT mode);
void gpio_set_val(UINT pin, UINT value);
UINT gpio_get_val(UINT pin);
void pwm_set_cc(UINT pin, UW duty);
void pwm_set_enabled(UINT pin, BOOL enabled);
void pwm_set_wrap(UINT pin, UW wrap);
void pwm_set_pin(UINT pin);
void ClearInt(UINT intno);
void EnableInt(UINT intno, UINT priority);

#endif
