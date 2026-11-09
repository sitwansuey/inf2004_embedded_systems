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
/* Barcode-only import: preserve all other board defaults and profiles. */
#ifndef BOARD_BARCODE_RETRY
#define BOARD_BARCODE_RETRY 1
#endif
/* Physical barcode re-read: miss at normal speed -> reverse at 30% speed
 * (read backwards); hit -> forward at normal speed without re-reading;
 * miss -> forward at 30% speed for a third read; third miss -> carry on.
 * A read ends when the robot has travelled BOARD_BARCODE_END_GAP_MM without
 * a GP28 edge (vehicle.c measures it by odometry, so speed and pauses do not
 * matter); BOARD_BARCODE_END_QUIET_MS is only the time backstop for a robot
 * that stops on the marker. */
#ifndef BOARD_BARCODE_END_GAP_MM
#define BOARD_BARCODE_END_GAP_MM 30U /* no GP28 change over 30 mm = end of barcode */
#endif
#ifndef BOARD_BARCODE_END_QUIET_MS
#define BOARD_BARCODE_END_QUIET_MS 4000U /* backstop: no GP28 change for 4 s */
#endif
#ifndef BOARD_BARCODE_RETRY_DRIVE
#define BOARD_BARCODE_RETRY_DRIVE 1
#endif
#ifndef BOARD_BARCODE_RETRY_BRAKE_MS
#define BOARD_BARCODE_RETRY_BRAKE_MS 200U /* settle before reversing direction */
#endif
#ifndef BOARD_BARCODE_RETRY_SPEED_PERCENT
#define BOARD_BARCODE_RETRY_SPEED_PERCENT 30U /* re-read speed, % of the normal demand */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_MAX_MM
#define BOARD_BARCODE_RETRY_REVERSE_MAX_MM 800U /* reverse bound if no read result */
#endif
#ifndef BOARD_BARCODE_RETRY_SLOW_EXTRA_MM
#define BOARD_BARCODE_RETRY_SLOW_EXTRA_MM 150U /* slow pass beyond the reverse distance */
#endif
#ifndef BOARD_BARCODE_RETRY_COOLDOWN_MM
#define BOARD_BARCODE_RETRY_COOLDOWN_MM 300U /* ignore late results of the same marker */
#endif
#ifndef BOARD_BARCODE_RETRY_MIN_POWER
#define BOARD_BARCODE_RETRY_MIN_POWER 5U /* PID speed target floor for the slow passes */
#endif
#ifndef BOARD_BARCODE_RETRY_STATE_MS
#define BOARD_BARCODE_RETRY_STATE_MS 20000U /* any retry step longer than this gives up */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_ELEMENTS
#define BOARD_BARCODE_RETRY_REVERSE_ELEMENTS 29U /* stop reversing once a whole *X* (15 bars + 14 spaces) is read */
#endif
#ifndef BOARD_BARCODE_RETRY_IDLE_MS
#define BOARD_BARCODE_RETRY_IDLE_MS 3000U /* no barcode state change while reversing/waiting = failed */
#endif
#ifndef BOARD_BARCODE_RETRY_RUNUP_MM
#define BOARD_BARCODE_RETRY_RUNUP_MM 50U /* reverse this far beyond the end gap past the first bar */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_MIN_MS
#define BOARD_BARCODE_RETRY_REVERSE_MIN_MS 5000U /* reverse at least this long before declaring failure */
#endif
/* First normal-speed pass: once GP28 starts a marker (S3 READING), read the
 * rest of it at the re-read speed above. Bars are measured by travelled
 * distance, so the change of speed does not distort them, and a ~4 mm narrow
 * bar gets ~8 samples instead of ~2 at full speed. */
#ifndef BOARD_BARCODE_SLOW_WHILE_READING
#define BOARD_BARCODE_SLOW_WHILE_READING 1
#endif
/* A decoded turn runs at the next junction (both straddle sensors and GP28 on
 * black): straight on by the sensor-to-wheel offset, so the wheels pivot on
 * the crossing. No junction within SEARCH_MM: it turns there. Straddle builds
 * only (BOARD_LINE_STRADDLE); edge-following builds still turn at once. */
#ifndef BOARD_NAV_JUNCTION_OFFSET_MM
#define BOARD_NAV_JUNCTION_OFFSET_MM 90U /* front IR sensors to the wheel axle */
#endif
#ifndef BOARD_NAV_JUNCTION_SEARCH_MM
#define BOARD_NAV_JUNCTION_SEARCH_MM 600U /* stop waiting for a junction */
#endif
#if BOARD_NAV_JUNCTION_OFFSET_MM > 1000 || BOARD_NAV_JUNCTION_SEARCH_MM > 10000
#error Junction navigation distances out of range
#endif
/* After a barcode U-turn, rotate on the spot (+/-30, +/-60 deg sweep) until a
 * line sensor is on black before tracking. 1: left/right turns do it too. */
#ifndef BOARD_NAV_UTURN_IGNORE_MM
#define BOARD_NAV_UTURN_IGNORE_MM 1200U /* after a U-turn, re-reading the same marker does not turn again */
#endif
#ifndef BOARD_NAV_SPOT_SCAN_ALL
#define BOARD_NAV_SPOT_SCAN_ALL 0
#endif
/* Steered reverse (PID builds): steer = -YAW_GAIN*theta_deg + LINE_GAIN*e,
 * where theta is yaw relative to the forward-tracking heading and e is +1/-1
 * when only the right/left straddle sensor is on black. */
#ifndef BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN
#define BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN 40 /* steering per degree of heading error */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN
/* LINE/YAW sets the heading authority (200/40 = 5 deg) the line term has
 * against a wrong heading reference; below ~5x the robot drifts off. */
#define BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN 200 /* steering when one sensor touches the line */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT
#define BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT 300 /* |steer| bound, S2 accepts up to 400 */
#endif
#ifndef BOARD_BARCODE_RETRY_YAW_REF_DIV
#define BOARD_BARCODE_RETRY_YAW_REF_DIV 50 /* heading EMA divisor: 50 ticks x 10 ms = 0.5 s */
#endif
/* Straddle line following (PID builds): a side touch steers smoothly instead
 * of pivoting; see vehicle_line_steer.h. Set BOARD_LINE_SMOOTH_STEER 0 for
 * the original pivot-per-touch behaviour. */
#ifndef BOARD_LINE_SMOOTH_STEER
#define BOARD_LINE_SMOOTH_STEER 1
#endif
#ifndef BOARD_LINE_STEER_MIN
#define BOARD_LINE_STEER_MIN 150 /* first correction: inner wheel ~74% of the outer */
#endif
#ifndef BOARD_LINE_STEER_MAX
#define BOARD_LINE_STEER_MAX 400 /* after the ramp: ~141 mm turn radius */
#endif
#ifndef BOARD_LINE_STEER_RAMP_MS
#define BOARD_LINE_STEER_RAMP_MS 300U /* MIN -> MAX while the same sensor stays on black */
#endif
#ifndef BOARD_LINE_PIVOT_AFTER_MS
#define BOARD_LINE_PIVOT_AFTER_MS 700U /* still touching: tight bend, pivot (0 = never) */
#endif
#ifndef BOARD_BARCODE_MIN_RUN_MS
#define BOARD_BARCODE_MIN_RUN_MS 20U
#endif
#ifndef BOARD_BARCODE_MIN_RUN_POSITION
#define BOARD_BARCODE_MIN_RUN_POSITION 4U /* half encoder counts (~0.7 mm): shorter AND briefer = noise */
#endif
#ifndef BOARD_BARCODE_DIGITAL_BLACK_LEVEL
#define BOARD_BARCODE_DIGITAL_BLACK_LEVEL 1U /* GP28 was measured HIGH on black. */
#endif
#if BOARD_BARCODE_DIGITAL_BLACK_LEVEL != 0 && BOARD_BARCODE_DIGITAL_BLACK_LEVEL != 1
#error Barcode digital black level must be zero or one
#endif
#ifndef BOARD_LINE_STRADDLE
#define BOARD_LINE_STRADDLE 0 /* 1: GP26/27 straddle the line; both WHITE = centred */
#endif
#ifndef BOARD_IR_MODE
#define BOARD_IR_MODE 0 /* 0 disabled, 1 digital, 2 ADC, 3 ADC line + GP28 digital barcode. */
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
#define BOARD_MAP_A 4 /* Codebook: A=U-turn, Z=right, F=left; B-D unmapped (read, no turn). 1 left 2 right 3 straight 4 U-turn */
#endif
#ifndef BOARD_MAP_B
#define BOARD_MAP_B 0
#endif
#ifndef BOARD_MAP_C
#define BOARD_MAP_C 0
#endif
#ifndef BOARD_MAP_D
#define BOARD_MAP_D 0
#endif
#ifndef BOARD_MAP_Z
#define BOARD_MAP_Z 2
#endif
#ifndef BOARD_MAP_F
#define BOARD_MAP_F 1 /* left */
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
#define BOARD_TERRAIN_CALIBRATED 0 /* sensor X forward, Y left, Z up */
#endif
#ifndef BOARD_PITCH_ZERO_CDEG
#define BOARD_PITCH_ZERO_CDEG 0
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
