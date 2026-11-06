/* Groupmate's wireless floor calibration snapshot, supplied 2026-11-06.
 * Motor 1 LEFT, motor 2 RIGHT. No motor unlock, start or network settings.
 * Command reference scale, not measured maximum speed. Course acceptance
 * remains pending; fractional-output PI is required. Dashboard Start=20.
 */
#ifndef CSC02_MOTOR_SPEED_CALIBRATED_20261106_H
#define CSC02_MOTOR_SPEED_CALIBRATED_20261106_H
#define BOARD_PID_ENABLED 1
#define S2_ENCODER_ENABLED 1
#define S2_COMMAND_TIMEOUT_MS 150
#define BOARD_DRIVE_POWER_CAP 30U
#define BOARD_PID_MAX_POWER 30
#define BOARD_PID_MAX_CPS 1667
#define BOARD_PID_KP_MILLI 20
#define BOARD_PID_KI_MILLI 10
#define BOARD_PID_KD_MILLI 0
#define BOARD_PID_CALIBRATED 0
#define S2_MOTOR1_TRIM_PERCENT 74U
#define S2_MOTOR2_TRIM_PERCENT 78U
#define BOARD_PID_LEFT_FF_OFFSET 3U
#define BOARD_PID_RIGHT_FF_OFFSET 4U
#define BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE 1000U
#define BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE 970U
#define BOARD_RIGHT_PWM_PERMILLE 1000U
#define BOARD_TICKS_PER_REV 631
#define BOARD_WHEEL_CIRCUMFERENCE_MM 220
#define BOARD_TRACK_WIDTH_MM 113
#endif
