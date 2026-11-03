/* Copy into app_program only for this RoboPico's USB motor and line trials. */
#ifndef CSC02_BENCH_CALIBRATION_H
#define CSC02_BENCH_CALIBRATION_H
#if defined(BOARD_IR_MOTOR_SCAN_BENCH) && BOARD_IR_MOTOR_SCAN_BENCH && !BOARD_IR_LINE_ONLY_BENCH
#define BOARD_TICKS_PER_REV 631
#define BOARD_WHEEL_CIRCUMFERENCE_MM 220
#define BOARD_TRACK_WIDTH_MM 113
#define BOARD_PID_ENABLED 1
#define BOARD_PID_CALIBRATED 0
#define BOARD_PID_MAX_CPS 2500 /* Normalized trial scale: 15% requests 375 counts/s. */
#define BOARD_PID_KP_MILLI 30
#define BOARD_PID_KI_MILLI 10
#define BOARD_PID_KD_MILLI 0
#define BOARD_PID_MAX_POWER 20
#define S2_MOTOR1_TRIM_PERCENT 73U
#define S2_MOTOR2_TRIM_PERCENT 98U
#define BOARD_RIGHT_PWM_PERMILLE 990U /* Provisional; line-following trial only. */
#define BOARD_BENCH_CLEARANCE_MM 200U /* Strictly greater than 20 cm and a fresh forward echo. */
#endif
#endif
