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
/* Decoded turn (VC_NAV_TURN): spin at NAV_TURN_POWER by encoder yaw to
 * NAV_TURN_MIN_DEG, then on at NAV_TURN_SEEK_POWER until a straddle sensor
 * touches black (the new line); no touch by NAV_TURN_MAX_DEG ends it anyway.
 * A touch before MIN_DEG (old line, crossing) never ends it. U-turns use the
 * UTURN pair. Straddle builds only; MAX_DEG 0 turns by encoder yaw alone. */
#ifndef BOARD_NAV_TURN_POWER
#define BOARD_NAV_TURN_POWER 25U
#endif
#ifndef BOARD_NAV_TURN_SEEK_POWER
#define BOARD_NAV_TURN_SEEK_POWER 15U
#endif
#ifndef BOARD_NAV_TURN_MIN_DEG
#define BOARD_NAV_TURN_MIN_DEG 60U
#endif
#ifndef BOARD_NAV_TURN_MAX_DEG
#define BOARD_NAV_TURN_MAX_DEG 120U
#endif
#ifndef BOARD_NAV_UTURN_MIN_DEG
#define BOARD_NAV_UTURN_MIN_DEG 150U
#endif
#ifndef BOARD_NAV_UTURN_MAX_DEG
#define BOARD_NAV_UTURN_MAX_DEG 210U
#endif
#if BOARD_NAV_TURN_POWER < 1 || BOARD_NAV_TURN_POWER > 50 || BOARD_NAV_TURN_SEEK_POWER < 1 || BOARD_NAV_TURN_SEEK_POWER > 50
#error Navigation turn powers must be 1..50 percent
#endif
#if (BOARD_NAV_TURN_MAX_DEG && (BOARD_NAV_TURN_MIN_DEG > 90 || BOARD_NAV_TURN_MAX_DEG < 90 || BOARD_NAV_TURN_MAX_DEG > 180)) || \
    (BOARD_NAV_UTURN_MAX_DEG && (BOARD_NAV_UTURN_MIN_DEG > 180 || BOARD_NAV_UTURN_MAX_DEG < 180 || BOARD_NAV_UTURN_MAX_DEG > 255))
#error Navigation turn windows must be MIN <= 90 <= MAX <= 180 (U-turn: MIN <= 180 <= MAX <= 255)
#endif
/* After a barcode U-turn, rotate on the spot (+/-30, +/-60 deg sweep) until a
 * line sensor is on black before tracking; a turn that ended on a line touch
 * needs none. 1: left/right turns do it too. */
#ifndef BOARD_NAV_UTURN_IGNORE_MM
#define BOARD_NAV_UTURN_IGNORE_MM 1200U /* after a U-turn, re-reading the same marker does not turn again */
#endif
#ifndef BOARD_NAV_SPOT_SCAN_ALL
#define BOARD_NAV_SPOT_SCAN_ALL 1
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
/* 2026-11-10: 150 / 300 ms reacted too gently for tight bends (a sim of the
 * course lost every 90 deg bend of <=100 mm radius); 250 / 150 ms holds them. */
#ifndef BOARD_LINE_STEER_MIN
#define BOARD_LINE_STEER_MIN 250 /* first correction: inner wheel ~60% of the outer */
#endif
#ifndef BOARD_LINE_STEER_MAX
#define BOARD_LINE_STEER_MAX 400 /* after the ramp: ~141 mm turn radius */
#endif
#ifndef BOARD_LINE_STEER_RAMP_MS
#define BOARD_LINE_STEER_RAMP_MS 150U /* MIN -> MAX while the same sensor stays on black */
#endif
/* While a barcode is read (>= 3 elements) the course is off; still steer
 * gently toward a single straddle sensor on black, up to this (0 = drive
 * straight as before; reads on a bend used to lose the line). */
#ifndef BOARD_BARCODE_READ_LINE_STEER
#define BOARD_BARCODE_READ_LINE_STEER 250
#endif
#if BOARD_BARCODE_READ_LINE_STEER < 0 || BOARD_BARCODE_READ_LINE_STEER > 400
#error BOARD_BARCODE_READ_LINE_STEER must be 0..400
#endif
#ifndef BOARD_LINE_PIVOT_AFTER_MS
#define BOARD_LINE_PIVOT_AFTER_MS 450U /* still touching: tight bend, pivot (0 = never); 700 lost sharp bends */
#endif
/* S2 steering slew while driving (units/ms, subsystem2/wheel_steering.h).
 * At 2 the line-steer ramp above was mostly hidden (old and new settings both
 * reached only ~200 after 100 ms) and sharp bends were lost; 4 reaches the
 * full 400 in 100 ms. */
#ifndef S2_STEER_SLEW_PER_MS
#define S2_STEER_SLEW_PER_MS 4U
#endif
#if S2_STEER_SLEW_PER_MS < 1 || S2_STEER_SLEW_PER_MS > 20
#error S2_STEER_SLEW_PER_MS must be 1..20 steering units per ms
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
#define BOARD_TERRAIN_CALIBRATED 0 /* mounting axes and zeros measured */
#endif
/* GY-511 accelerometer mounting: the signed sensor axis that points to the
 * robot's front, its left and up (1=+X 2=+Y 3=+Z, negative = the minus
 * axis). Terrain works in that robot frame: pitch is positive nose-up,
 * roll positive left side up, and the zeros below are robot-frame angles
 * read standing still on level ground. */
#ifndef BOARD_IMU_FRONT_AXIS
#define BOARD_IMU_FRONT_AXIS 1
#endif
#ifndef BOARD_IMU_LEFT_AXIS
#define BOARD_IMU_LEFT_AXIS 2
#endif
#ifndef BOARD_IMU_UP_AXIS
#define BOARD_IMU_UP_AXIS 3
#endif
#define BOARD_IMU_ABS_(a) ((a)<0?-(a):(a))
#if BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS)<1 || BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS)>3 || \
    BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS)<1 || BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS)>3 || \
    BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS)<1 || BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS)>3
#error BOARD_IMU_*_AXIS must be +/-1, +/-2 or +/-3
#endif
/* Distinct axes forming a right-handed frame (front x left = up): the
 * permutation's parity times the three signs must be positive. */
#if (BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS)-BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS))* \
    (BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS)-BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS))* \
    (BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS)-BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS))* \
    (BOARD_IMU_FRONT_AXIS)*(BOARD_IMU_LEFT_AXIS)*(BOARD_IMU_UP_AXIS) <= 0
#error BOARD_IMU_FRONT/LEFT/UP_AXIS must be three different axes, right-handed
#endif
/* Ramp speed handling (phase 5), 0 = off. The terrain tilt is computed
 * from accelerometer samples smoothed over BOARD_IMU_SMOOTH_MS (samples
 * outside BOARD_IMU_SMOOTH_MIN/MAX_MG skipped). While tracking (the line is
 * followed throughout), a total tilt (pitch or roll) of at least
 * BOARD_RAMP_TILT_CDEG held BOARD_RAMP_HOLD_MS is the way up: power capped at
 * BOARD_RAMP_UP_POWER (0: no cap). Once the tilt falls
 * BOARD_RAMP_CREST_DROP_CDEG below its peak it is the way down: capped at
 * BOARD_RAMP_DOWN_POWER. Back at BOARD_RAMP_LEVEL_CDEG for BOARD_RAMP_LEVEL_MS
 * the ramp is over: normal speed resumes and the next ramp is watched for,
 * or with BOARD_RAMP_STOP_AFTER 1 (ramp bench test) the run stops (VC_OK). */
#ifndef BOARD_RAMP_SPEED
#define BOARD_RAMP_SPEED 0
#endif
#ifndef BOARD_RAMP_STOP_AFTER
#define BOARD_RAMP_STOP_AFTER 0
#endif
#ifndef BOARD_IMU_SMOOTH_MS
#define BOARD_IMU_SMOOTH_MS 400U
#endif
#ifndef BOARD_IMU_SMOOTH_MIN_MG
#define BOARD_IMU_SMOOTH_MIN_MG 700
#endif
#ifndef BOARD_IMU_SMOOTH_MAX_MG
#define BOARD_IMU_SMOOTH_MAX_MG 1200
#endif
#ifndef BOARD_RAMP_TILT_CDEG
#define BOARD_RAMP_TILT_CDEG 400
#endif
#ifndef BOARD_RAMP_HOLD_MS
#define BOARD_RAMP_HOLD_MS 400U /* rejects the ~0.25 s pitch bump at the ramp entry */
#endif
#ifndef BOARD_RAMP_CREST_DROP_CDEG
#define BOARD_RAMP_CREST_DROP_CDEG 150
#endif
#ifndef BOARD_RAMP_LEVEL_CDEG
#define BOARD_RAMP_LEVEL_CDEG 300
#endif
#ifndef BOARD_RAMP_LEVEL_MS
#define BOARD_RAMP_LEVEL_MS 400U
#endif
#ifndef BOARD_RAMP_UP_POWER
#define BOARD_RAMP_UP_POWER 0U
#endif
#ifndef BOARD_RAMP_DOWN_POWER
#define BOARD_RAMP_DOWN_POWER 10U
#endif
#if BOARD_RAMP_SPEED && (BOARD_RAMP_UP_POWER > 50 || BOARD_RAMP_DOWN_POWER > 50 || \
    BOARD_RAMP_LEVEL_CDEG >= BOARD_RAMP_TILT_CDEG || BOARD_IMU_SMOOTH_MS < 1 || BOARD_IMU_SMOOTH_MS > 5000)
#error Ramp test: powers 0..50 (0 no cap), level below the start tilt, smoothing 1..5000 ms
#endif
#ifndef BOARD_PITCH_ZERO_CDEG
#define BOARD_PITCH_ZERO_CDEG 0
#endif
#ifndef BOARD_ROLL_ZERO_CDEG
#define BOARD_ROLL_ZERO_CDEG 0
#endif
/* 1: one accelerometer sample above 1.8 g (S4 shock) stops the run with
 * VC_SHOCK. 0: shock is still detected and reported (dashboard impact,
 * terrain SHOCK) but does not stop the run; the robot's own jolts (reverse
 * to forward, pivot starts) exceed 1.8 g. A real collision still stalls the
 * wheels (VC_MOTOR, PID builds only: S2's no-progress check runs inside the
 * PID) and a sustained tilt loss still faults VC_SENSOR. Default: off with
 * PID, on without it (no stall backstop there). */
#ifndef BOARD_SHOCK_STOP
#define BOARD_SHOCK_STOP (!BOARD_PID_ENABLED)
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
/* In-place spins (PID builds, subsystem2/turn_drive.h; navigation turns and
 * the spot scan): tyre scrub needs more torque than straight driving. A spin
 * may use up to TURN_MAX_POWER % PWM, holds each wheel at >= KICK_POWER %
 * until it has moved KICK_COUNTS (at most KICK_MS), kicks a wheel again after
 * REKICK_MS without progress, and faults only after STALL_MS without progress
 * (straight and the line course's pivots: 500 ms, unchanged). */
#ifndef BOARD_PID_TURN_MAX_POWER
#define BOARD_PID_TURN_MAX_POWER 55
#endif
#ifndef BOARD_TURN_KICK_MS
#define BOARD_TURN_KICK_MS 150U /* 0: no kick */
#endif
#ifndef BOARD_TURN_KICK_POWER
#define BOARD_TURN_KICK_POWER 45
#endif
#ifndef BOARD_TURN_KICK_COUNTS
#define BOARD_TURN_KICK_COUNTS 5 /* encoder counts of real motion that end a spin kick */
#endif
#ifndef BOARD_TURN_REKICK_MS
#define BOARD_TURN_REKICK_MS 100U /* a spinning wheel this long without progress is kicked again */
#endif
#ifndef BOARD_TURN_STALL_MS
#define BOARD_TURN_STALL_MS 1200U
#endif
#if BOARD_PID_ENABLED && (BOARD_PID_TURN_MAX_POWER < BOARD_PID_MAX_POWER || BOARD_PID_TURN_MAX_POWER > 100 || \
    BOARD_TURN_KICK_POWER < 0 || BOARD_TURN_KICK_POWER > BOARD_PID_TURN_MAX_POWER || BOARD_TURN_KICK_MS > 1000 || \
    BOARD_TURN_STALL_MS < 500 || BOARD_TURN_STALL_MS > 5000)
#error Turn drive needs PID_MAX_POWER <= TURN_MAX_POWER <= 100, KICK_POWER <= TURN_MAX_POWER, KICK_MS <= 1000, STALL_MS 500..5000
#endif
/* Preserve Buddy 5's existing Grove wiring (subsystem5 branch, app.h).
 * The Week 6 document lists these roles in the opposite order. */
#define BOARD_ULTRASONIC_TRIG_GPIO 1U
#define BOARD_ULTRASONIC_ECHO_GPIO 0U
#define BOARD_SERVO_GPIO 12U
/* Servo pulse that aims the ranger straight ahead (S5 angle 0). Measured
 * 2026-11-10 with an object centred in front of the robot: at 1500 us its
 * edges read +40/-27 deg (and every logged run leaned the same way), so
 * 1500 us aims ~6.5 deg right; 6.5 deg * 750 us / 90 deg = 54 us. */
#ifndef BOARD_SERVO_CENTER_US
#define BOARD_SERVO_CENTER_US 1554
#endif
#if BOARD_SERVO_CENTER_US < 1300 || BOARD_SERVO_CENTER_US > 1700
#error BOARD_SERVO_CENTER_US is the straight-ahead pulse: 1300..1700 us
#endif
#endif
