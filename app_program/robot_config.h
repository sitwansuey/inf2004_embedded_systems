/* Bench defaults deliberately require measured wiring/calibration before arming. */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#if __has_include("robot_config.local.h")
#include "robot_config.local.h"
#endif
/* S5 hardware diagnostics are explicit builds, separate from passive TEST=1. */
#ifndef APP_S5_MASTER_TEST
#define APP_S5_MASTER_TEST 0
#endif
#ifndef APP_ACQUISITION_TEST
#define APP_ACQUISITION_TEST 0
#endif
#ifndef APP_ACQUISITION_LEFT_TEST
#define APP_ACQUISITION_LEFT_TEST 0
#endif
#ifndef APP_S5_SERVO_TEST
#define APP_S5_SERVO_TEST 0
#endif
#define APP_S5_DIAGNOSTICS (APP_S5_MASTER_TEST || APP_ACQUISITION_TEST || APP_ACQUISITION_LEFT_TEST || APP_S5_SERVO_TEST)
#if APP_S5_DIAGNOSTICS && !APP_TEST
#error S5 diagnostics require APP_TEST=1
#endif
#ifndef BOARD_MOTORS_DISABLED
#define BOARD_MOTORS_DISABLED 1 /* USB bench lock; requires a deliberate rebuild to enable drive. */
#endif
#if BOARD_MOTORS_DISABLED != 0 && BOARD_MOTORS_DISABLED != 1
#error BOARD_MOTORS_DISABLED must be 0 or 1
#endif
#ifndef BOARD_UART0_FREE
#define BOARD_UART0_FREE 0
#endif
#ifndef APP_S5_ENABLED
#define APP_S5_ENABLED BOARD_UART0_FREE
#endif
#ifndef BOARD_IR_MODE
#define BOARD_IR_MODE 0 /* 0 disabled, 1 digital, 2 ADC; confirm AO/DO first. */
#endif
#ifndef BOARD_IR_RAW_BENCH
#define BOARD_IR_RAW_BENCH 0 /* Input-only bring-up; never calibrated steering. */
#endif
#if BOARD_IR_RAW_BENCH != 0 && BOARD_IR_RAW_BENCH != 1
#error BOARD_IR_RAW_BENCH must be 0 or 1
#endif
#if BOARD_IR_RAW_BENCH && (!BOARD_MOTORS_DISABLED || BOARD_IR_MODE == 0)
#error IR raw bench requires the drive lock and a selected input mode
#endif
#ifndef BOARD_IR_MOTOR_SCAN_BENCH
#define BOARD_IR_MOTOR_SCAN_BENCH 0
#endif
#if BOARD_IR_MOTOR_SCAN_BENCH != 0 && BOARD_IR_MOTOR_SCAN_BENCH != 1
#error BOARD_IR_MOTOR_SCAN_BENCH must be 0 or 1
#endif
#ifndef BOARD_IR_LINE_TRACE_BENCH
#define BOARD_IR_LINE_TRACE_BENCH 0
#endif
#if BOARD_IR_LINE_TRACE_BENCH != 0 && BOARD_IR_LINE_TRACE_BENCH != 1
#error BOARD_IR_LINE_TRACE_BENCH must be 0 or 1
#endif
#if BOARD_IR_LINE_TRACE_BENCH && !BOARD_IR_MOTOR_SCAN_BENCH
#error Line trace requires the bounded USB motor bench infrastructure
#endif
#ifndef BOARD_IR_LINE_ONLY_BENCH
#define BOARD_IR_LINE_ONLY_BENCH 0
#endif
#if BOARD_IR_LINE_ONLY_BENCH != 0 && BOARD_IR_LINE_ONLY_BENCH != 1
#error BOARD_IR_LINE_ONLY_BENCH must be 0 or 1
#endif
#ifndef BOARD_IR_CONTINUOUS_BENCH
#define BOARD_IR_CONTINUOUS_BENCH 0
#endif
#if BOARD_IR_CONTINUOUS_BENCH != 0 && BOARD_IR_CONTINUOUS_BENCH != 1
#error Continuous bench must be zero or one
#endif
#if BOARD_IR_CONTINUOUS_BENCH && !BOARD_IR_LINE_ONLY_BENCH
#error Continuous bench requires its separate line-only profile
#endif
#if BOARD_IR_LINE_ONLY_BENCH
#if BOARD_IR_CONTINUOUS_BENCH
#if !defined(IR_SCAN_MAX_POWER) || IR_SCAN_MAX_POWER != 30U
#error Continuous bench requires its separate 30 percent cap
#endif
#elif !defined(IR_SCAN_MAX_POWER) || IR_SCAN_MAX_POWER != 50U
#error Course bench requires its separate 50 percent cap
#endif
#if !BOARD_IR_LINE_TRACE_BENCH || APP_S5_ENABLED || S2_ENCODER_ENABLED != 1
#error Line-only bench requires line trace, disabled sonar and both encoder channels
#endif
#if !defined(IR_SCAN_MAX_MS) || IR_SCAN_MAX_MS != 20000U
#error Line-only bench requires its separate 20000ms cap
#endif
#else
#if defined(IR_SCAN_MAX_MS) && IR_SCAN_MAX_MS != 6000U
#error Other motor benches retain their 6000ms cap
#endif
#if defined(IR_SCAN_MAX_POWER) && IR_SCAN_MAX_POWER != 20U
#error Other motor benches retain their 20 percent power cap
#endif
#endif
#ifndef BOARD_IR_CALIBRATED_BENCH
#define BOARD_IR_CALIBRATED_BENCH 0
#endif
#if BOARD_IR_CALIBRATED_BENCH != 0 && BOARD_IR_CALIBRATED_BENCH != 1
#error BOARD_IR_CALIBRATED_BENCH must be 0 or 1
#endif
#if BOARD_IR_CALIBRATED_BENCH
#if !BOARD_MOTORS_DISABLED || BOARD_IR_MODE != 2 || BOARD_IR_RAW_BENCH
#error Calibrated IR bench requires locked motors, ADC and no raw profile
#endif
#endif
#if BOARD_IR_CALIBRATED_BENCH || BOARD_IR_MOTOR_SCAN_BENCH
#if defined(BOARD_IR_ADC_CALIBRATED) || defined(BOARD_IR_ADC_WHITE) || defined(BOARD_IR_ADC_BLACK)
#error Calibrated IR bench must use its reviewed measured endpoints
#endif
#include "robot_ir_calibration.h"
#endif
#ifndef BOARD_IR_BARCODE_CAPTURE
#define BOARD_IR_BARCODE_CAPTURE 0
#endif
#if BOARD_IR_BARCODE_CAPTURE != 0 && BOARD_IR_BARCODE_CAPTURE != 1
#error BOARD_IR_BARCODE_CAPTURE must be 0 or 1
#endif
#if BOARD_IR_MOTOR_SCAN_BENCH
#if BOARD_MOTORS_DISABLED || BOARD_IR_MODE != 2 || BOARD_IR_RAW_BENCH || BOARD_IR_CALIBRATED_BENCH || !BOARD_IR_BARCODE_CAPTURE
#error Motor scan requires its separate enabled-motor ADC capture profile
#endif
#if !defined(S2_COMMAND_TIMEOUT_MS) || S2_COMMAND_TIMEOUT_MS > 150 || S2_COMMAND_TIMEOUT_MS == 0
#error Motor scan requires the independent motor watchdog at most150ms
#endif
#if !BOARD_UART0_FREE || TM_MQTT || APP_TEST || APP_S2_ONLY || APP_S4_ONLY
#error Motor scan requires USB, no MQTT, no startup tests and all subsystem owners
#endif
#endif
#if BOARD_IR_BARCODE_CAPTURE && !BOARD_IR_CALIBRATED_BENCH && !BOARD_IR_MOTOR_SCAN_BENCH
#error Barcode capture requires a reviewed IR bench profile
#endif
#ifndef BOARD_IR_BLACK_LEVEL
#define BOARD_IR_BLACK_LEVEL 1
#endif
#ifndef BOARD_IR_LINE_MASK
#define BOARD_IR_LINE_MASK 3 /* GP26 right + GP27 left; GP28 is barcode-only. */
#endif
#ifndef BOARD_BARCODE_SENSOR
#define BOARD_BARCODE_SENSOR 2 /* S3 index 2 = GP28; excluded from line mask. */
#endif
#ifndef BOARD_IR_POSITIONS
#define BOARD_IR_POSITIONS {1000,-1000,0} /* Logical right/left; not millimetres. */
#endif
#ifndef BOARD_MAP_A
#define BOARD_MAP_A 1 /* Code 39 A=left, B=right, C=straight, D=U-turn. */
#endif
#ifndef BOARD_MAP_B
#define BOARD_MAP_B 2
#endif
#ifndef BOARD_MAP_C
#define BOARD_MAP_C 3
#endif
#ifndef BOARD_MAP_D
#define BOARD_MAP_D 4
#endif
#ifndef BOARD_MAP_Z
#define BOARD_MAP_Z 0
#endif
#ifndef BOARD_TICKS_PER_REV
#define BOARD_TICKS_PER_REV 0 /* measured x1 counts, not quadrature x4 */
#endif
#ifndef BOARD_WHEEL_CIRCUMFERENCE_MM
#define BOARD_WHEEL_CIRCUMFERENCE_MM 0
#endif
#ifndef BOARD_TRACK_WIDTH_MM
#define BOARD_TRACK_WIDTH_MM 0
#endif
#ifndef BOARD_TERRAIN_CALIBRATED
#define BOARD_TERRAIN_CALIBRATED 0 /* verified mounting and flat reference; not a scale-accuracy claim */
#endif
#ifndef BOARD_S4_MOUNTING
#define BOARD_S4_MOUNTING 0 /* 0: forward=X,left=Y,up=Z; 1: forward=-Y,left=X,up=Z */
#endif
#if BOARD_S4_MOUNTING < 0 || BOARD_S4_MOUNTING > 1
#error Unsupported Buddy 4 mounting profile
#endif
#ifndef BOARD_ACCEL_CALIBRATED
#define BOARD_ACCEL_CALIBRATED 0
#endif
#ifndef BOARD_ACCEL_BIAS_X_MG
#define BOARD_ACCEL_BIAS_X_MG 0
#endif
#ifndef BOARD_ACCEL_BIAS_Y_MG
#define BOARD_ACCEL_BIAS_Y_MG 0
#endif
#ifndef BOARD_ACCEL_BIAS_Z_MG
#define BOARD_ACCEL_BIAS_Z_MG 0
#endif
#ifndef BOARD_ACCEL_GAIN_X_MILLI
#define BOARD_ACCEL_GAIN_X_MILLI 1000
#endif
#ifndef BOARD_ACCEL_GAIN_Y_MILLI
#define BOARD_ACCEL_GAIN_Y_MILLI 1000
#endif
#ifndef BOARD_ACCEL_GAIN_Z_MILLI
#define BOARD_ACCEL_GAIN_Z_MILLI 1000
#endif
#if BOARD_ACCEL_CALIBRATED < 0 || BOARD_ACCEL_CALIBRATED > 1 || \
    BOARD_ACCEL_BIAS_X_MG < -500 || BOARD_ACCEL_BIAS_X_MG > 500 || \
    BOARD_ACCEL_BIAS_Y_MG < -500 || BOARD_ACCEL_BIAS_Y_MG > 500 || \
    BOARD_ACCEL_BIAS_Z_MG < -500 || BOARD_ACCEL_BIAS_Z_MG > 500 || \
    BOARD_ACCEL_GAIN_X_MILLI < 500 || BOARD_ACCEL_GAIN_X_MILLI > 1500 || \
    BOARD_ACCEL_GAIN_Y_MILLI < 500 || BOARD_ACCEL_GAIN_Y_MILLI > 1500 || \
    BOARD_ACCEL_GAIN_Z_MILLI < 500 || BOARD_ACCEL_GAIN_Z_MILLI > 1500
#error Invalid accelerometer calibration coefficients
#endif
#ifndef BOARD_PITCH_ZERO_CDEG
#define BOARD_PITCH_ZERO_CDEG 0
#endif
#ifndef BOARD_TERRAIN_ONE_SIDED
#define BOARD_TERRAIN_ONE_SIDED 1
#endif
#ifndef BOARD_IMU_GEOMETRY_CALIBRATED
#define BOARD_IMU_GEOMETRY_CALIBRATED 0
#endif
#ifndef BOARD_IMU_FORWARD_MM
#define BOARD_IMU_FORWARD_MM 0
#endif
#ifndef BOARD_IMU_LEFT_MM
#define BOARD_IMU_LEFT_MM 0
#endif
#ifndef BOARD_IMU_UP_MM
#define BOARD_IMU_UP_MM 0
#endif
#if BOARD_TRACK_WIDTH_MM < 0 || BOARD_TRACK_WIDTH_MM > 1000 || \
    BOARD_IMU_FORWARD_MM < -500 || BOARD_IMU_FORWARD_MM > 500 || \
    BOARD_IMU_LEFT_MM < -500 || BOARD_IMU_LEFT_MM > 500 || \
    BOARD_IMU_UP_MM < -500 || BOARD_IMU_UP_MM > 500 || \
    BOARD_TERRAIN_ONE_SIDED < 0 || BOARD_TERRAIN_ONE_SIDED > 1 || \
    BOARD_IMU_GEOMETRY_CALIBRATED < 0 || BOARD_IMU_GEOMETRY_CALIBRATED > 1
#error Invalid Buddy 4 height geometry
#endif
#ifndef BOARD_ROLL_ZERO_CDEG
#define BOARD_ROLL_ZERO_CDEG 0
#endif
#ifndef BOARD_BYPASS_VALIDATED
#define BOARD_BYPASS_VALIDATED 0 /* enable only after raised-wheel/track tests */
#endif
#ifndef BOARD_BENCH_CLEARANCE_MM
#define BOARD_BENCH_CLEARANCE_MM 250U
#endif
#ifndef BOARD_PID_ENABLED
#define BOARD_PID_ENABLED 0
#endif
#ifndef BOARD_PID_CALIBRATED
#define BOARD_PID_CALIBRATED 0
#endif
#ifndef BOARD_PID_MAX_CPS
#define BOARD_PID_MAX_CPS 0 /* measured wheel counts/s at the reference full scale */
#endif
#ifndef BOARD_PID_KP_MILLI
#define BOARD_PID_KP_MILLI 20
#endif
#ifndef BOARD_PID_KI_MILLI
#define BOARD_PID_KI_MILLI 5
#endif
#ifndef BOARD_PID_KD_MILLI
#define BOARD_PID_KD_MILLI 0
#endif
#ifndef BOARD_PID_MAX_POWER
#define BOARD_PID_MAX_POWER 50
#endif
#if BOARD_PID_ENABLED != 0 && BOARD_PID_ENABLED != 1
#error BOARD_PID_ENABLED must be 0 or 1
#endif
#if BOARD_PID_ENABLED && (BOARD_PID_KP_MILLI < 0 || BOARD_PID_KP_MILLI > 10000 || BOARD_PID_KI_MILLI < 0 || BOARD_PID_KI_MILLI > 10000 || BOARD_PID_KD_MILLI < 0 || BOARD_PID_KD_MILLI > 10000 || (BOARD_PID_KP_MILLI + BOARD_PID_KI_MILLI + BOARD_PID_KD_MILLI) == 0)
#error PID gains must be in 0..10000 with at least one nonzero feedback gain
#endif
#if BOARD_PID_ENABLED && (BOARD_PID_MAX_CPS < 1 || BOARD_PID_MAX_CPS > 100000 || BOARD_PID_MAX_POWER < 1 || BOARD_PID_MAX_POWER > 50)
#error PID requires a measured speed scale and an output cap of 1..50 percent
#endif
/* Preserve Buddy 5's existing Grove wiring (subsystem5 branch, app.h).
 * The Week 6 document lists these roles in the opposite order. */
#define BOARD_ULTRASONIC_TRIG_GPIO 1U
#define BOARD_ULTRASONIC_ECHO_GPIO 0U
#define BOARD_SERVO_GPIO 12U
#endif
