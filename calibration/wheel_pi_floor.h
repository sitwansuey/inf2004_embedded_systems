/* Initial wheel PI settings measured on this robot, 2026-11-05.
 * Straight floor pulses validated; line steering and mapped turns need tests.
 * 1667 is a command scale (30 ->500cps), not measured 100%-PWM speed.
 * FF(PWM)=round(command*trim/100)+offset, then feedback, capped at30.
 * Physical evidence: workspace calibration/pid_20261105/PI_FLOOR_ANALYSIS.json. */
#ifndef CSC02_WHEEL_PI_FLOOR_H
#define CSC02_WHEEL_PI_FLOOR_H
#define BOARD_PID_MAX_CPS 1667
#define BOARD_PID_KP_MILLI 20
#define BOARD_PID_KI_MILLI 10
#define BOARD_PID_KD_MILLI 0
#define BOARD_PID_MAX_POWER 30
#define BOARD_PID_CALIBRATED 0 /* Not a complete course/turn validation. */
#define BOARD_PID_LEFT_FF_OFFSET 3U
#define BOARD_PID_RIGHT_FF_OFFSET 4U
#define BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE 1000U
/* Wireless 3s floor check at970: counts1576/1583; user confirmed much straighter.
 * This changes the PI reference. Full line/barcode/turn validation still pending. */
#define BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE 970U
#define S2_MOTOR1_TRIM_PERCENT 74U
#define S2_MOTOR2_TRIM_PERCENT 78U
#endif
