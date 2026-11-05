/* This robot's open-loop barcode demonstration profile. Not PID calibration.
 * GP28 now uses DO/D0 as digital input; the old analog endpoint calibration
 * does not apply. Verify physical white/black polarity before a powered run.
 * Wheel geometry is carried from the previous trial profile; physical
 * 90/180-degree turn accuracy must still be checked on the robot. */
#ifndef CSC02_BARCODE_DEMO30_H
#define CSC02_BARCODE_DEMO30_H
#define BOARD_MOTORS_DISABLED 0
#define BOARD_IR_MODE 3 /* GP26/27 ADC line sensors; GP28 digital barcode. */
#define BOARD_IR_ADC_CALIBRATED 1
#ifndef BOARD_BARCODE_DIGITAL_BLACK_LEVEL
#define BOARD_BARCODE_DIGITAL_BLACK_LEVEL 1U /* Verify on the physical white/black targets. */
#endif
#if BOARD_BARCODE_DIGITAL_BLACK_LEVEL == 1
#define BOARD_IR_ADC_WHITE {187U,189U,0U}
#define BOARD_IR_ADC_BLACK {3214U,3177U,1U}
#elif BOARD_BARCODE_DIGITAL_BLACK_LEVEL == 0
#define BOARD_IR_ADC_WHITE {187U,189U,1U}
#define BOARD_IR_ADC_BLACK {3214U,3177U,0U}
#else
#error Digital barcode black level must be zero or one
#endif
#define BOARD_IR_CENTER_WHITE 1
#define BOARD_BARCODE_MIN_RUN_MS 0U /* Temporarily disable the pulse-duration floor. */
#define BOARD_BARCODE_RETRY 1 /* Discard invalid candidates and keep tracing. */
#define BOARD_BARCODE_DEFAULT_FORWARD 1 /* Missing/unmapped instruction: straight. */
#ifndef BOARD_BARCODE_WHEEL_PI
#define BOARD_BARCODE_WHEEL_PI 0
#endif
#if BOARD_BARCODE_WHEEL_PI
#define BOARD_PID_ENABLED 1
#define BOARD_BARCODE_SCAN_STEERING 1
#include "wheel_pi_floor.h"
#else
#define BOARD_PID_ENABLED 0
#define BOARD_BARCODE_SCAN_STEERING 0
#define S2_MOTOR1_TRIM_PERCENT 100U
#define S2_MOTOR2_TRIM_PERCENT 100U
#endif
#define BOARD_RIGHT_PWM_PERMILLE 1000U
#define BOARD_DRIVE_POWER_CAP 30U
#define BOARD_TICKS_PER_REV 631
#define BOARD_WHEEL_CIRCUMFERENCE_MM 220
#define BOARD_TRACK_WIDTH_MM 113
#define APP_S5_ENABLED 0
#ifndef BOARD_BARCODE_AUTO_START
#define BOARD_BARCODE_AUTO_START 1
#endif
#endif
