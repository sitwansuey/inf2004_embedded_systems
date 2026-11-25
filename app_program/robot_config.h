/* Defaults = this robot's measured calibration and the full mission profile
 * (the flashed robot_config.local.h, made the defaults on 2026-11-16). The
 * firmware still boots DISARMED and drives only after RESET then START
 * (dashboard or USB 'g'); STOP, watchdogs and faults always apply.
 * A git-ignored app_program/robot_config.local.h, if present, is included
 * first and overrides any value below (e.g. BOARD_MOTORS_DISABLED 1 for
 * USB/sensor bench work); see robot_config.local.example.h. Host tests
 * define ROBOT_CONFIG_NO_LOCAL so a local file never leaks into them. */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#if __has_include("robot_config.local.h") && !defined(ROBOT_CONFIG_NO_LOCAL)
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
#define APP_S5_DIAGNOSTICS                                                     \
    (APP_S5_MASTER_TEST || APP_ACQUISITION_TEST ||                             \
     APP_ACQUISITION_LEFT_TEST || APP_S5_SERVO_TEST)
#if APP_S5_DIAGNOSTICS && !APP_TEST
#error S5 diagnostics require APP_TEST=1
#endif
#ifndef BOARD_MOTORS_DISABLED
#define BOARD_MOTORS_DISABLED                                                  \
    0 /* 1: USB bench lock, the motors never drive                             \
       */
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
/* Loop timing (shared/task_timing.h): the vehicle controller, motor and
 * network loops record their longest run and late releases; the controller
 * prints them on the USB console every 10 s ("[TIMING] ..."). 0: no hooks. */
#ifndef BOARD_TIMING_STATS
#define BOARD_TIMING_STATS 1
#endif
/* Hardware watchdog (app_watchdog.c, include/tm/robot_health.h). A 100 ms
 * cyclic handler feeds the RP2040 watchdog only while every monitored loop
 * (VC, motor, network on core 0; telemetry, IMU on core 1) has advanced in
 * the last WINDOW_MS. One stale loop: the crash record names it and the
 * watchdog reboots the board TIMEOUT_MS after the last feed (motors off,
 * boot DISARMED). Armed only once boot has finished (every loop seen, the
 * network task past cyw43 init and its first join) AND GRACE_MS has passed.
 * 0: no watchdog (the crash record and the boot report stay). */
#ifndef BOARD_WATCHDOG
#define BOARD_WATCHDOG 1
#endif
#ifndef BOARD_WATCHDOG_TIMEOUT_MS
#define BOARD_WATCHDOG_TIMEOUT_MS 1500U
#endif
#ifndef BOARD_WATCHDOG_WINDOW_MS
#define BOARD_WATCHDOG_WINDOW_MS 1000U
#endif
#ifndef BOARD_WATCHDOG_GRACE_MS
#define BOARD_WATCHDOG_GRACE_MS 20000U
#endif
#if BOARD_WATCHDOG_TIMEOUT_MS < 500 || BOARD_WATCHDOG_TIMEOUT_MS > 8000 ||     \
    BOARD_WATCHDOG_WINDOW_MS < 300 || BOARD_WATCHDOG_WINDOW_MS > 5000
#error Watchdog timeout/window outside supported range
#endif
/* CYW43 radio power save (lib/libwifi/.../cyw43_utk.c). 0: off
 * (CYW43_NONE_PM) after init and after every join; 1: driver default
 * CYW43_PERFORMANCE_PM, the behaviour before checkpoint 8c. */
#ifndef BOARD_WIFI_POWER_SAVE
#define BOARD_WIFI_POWER_SAVE 0
#endif
/* S3 bounded barcode resync (s3_config_t.barcode_retry). */
#ifndef BOARD_BARCODE_RETRY
#define BOARD_BARCODE_RETRY 1
#endif
/* A marker capture of 3 or more elements cut short by an S3 sampler hiccup
 * (a latched STALE/SENSOR fault) counts as a failed read
 * (s3_config_t.barcode_fault_fails): EV_BARCODE and the re-read follow, as
 * for any miss. 0: such a marker is skipped silently (8b behaviour). */
#ifndef BOARD_BARCODE_FAULT_FAILS
#define BOARD_BARCODE_FAULT_FAILS 1
#endif
/* Physical barcode re-read: miss at normal speed -> reverse at 30% speed
 * (read backwards); hit -> forward at normal speed without re-reading;
 * miss -> forward at 30% speed for a third read; third miss -> carry on.
 * A read ends when the robot has travelled BOARD_BARCODE_END_GAP_MM without
 * a GP28 edge (vehicle.c measures it by odometry, so speed and pauses do not
 * matter); BOARD_BARCODE_END_QUIET_MS is only the time backstop for a robot
 * that stops on the marker. */
#ifndef BOARD_BARCODE_END_GAP_MM
#define BOARD_BARCODE_END_GAP_MM                                               \
    30U /* no GP28 change over 30 mm = end of barcode */
#endif
#ifndef BOARD_BARCODE_END_QUIET_MS
#define BOARD_BARCODE_END_QUIET_MS                                             \
    4000U /* backstop: no GP28 change for 4 s                                  \
           */
#endif
#ifndef BOARD_BARCODE_RETRY_DRIVE
#define BOARD_BARCODE_RETRY_DRIVE 1
#endif
#ifndef BOARD_BARCODE_RETRY_BRAKE_MS
#define BOARD_BARCODE_RETRY_BRAKE_MS                                           \
    200U /* settle before reversing direction */
#endif
#ifndef BOARD_BARCODE_RETRY_SPEED_PERCENT
#define BOARD_BARCODE_RETRY_SPEED_PERCENT                                      \
    50U /* re-read speed, % of the normal demand (8f: 30,                      \
i.e. 6% at a 20% demand, where the wheels stalled and lurched mid-marker; 25   \
Nov) */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_MAX_MM
#define BOARD_BARCODE_RETRY_REVERSE_MAX_MM                                     \
    800U /* reverse bound if no read result */
#endif
#ifndef BOARD_BARCODE_RETRY_SLOW_EXTRA_MM
#define BOARD_BARCODE_RETRY_SLOW_EXTRA_MM                                      \
    150U /* slow pass beyond the reverse distance */
#endif
#ifndef BOARD_BARCODE_RETRY_COOLDOWN_MM
#define BOARD_BARCODE_RETRY_COOLDOWN_MM                                        \
    300U /* ignore late results of the same marker */
#endif
#ifndef BOARD_BARCODE_RETRY_MIN_POWER
#define BOARD_BARCODE_RETRY_MIN_POWER                                          \
    5U /* PID speed target floor for the slow passes */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_MIN_POWER
#define BOARD_BARCODE_RETRY_REVERSE_MIN_POWER                                  \
    10U /* reverse re-read floor: 6% stalled the wheels in reverse */
#endif
#ifndef BOARD_BARCODE_RETRY_STATE_MS
#define BOARD_BARCODE_RETRY_STATE_MS                                           \
    20000U /* any retry step longer than this gives up */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_ELEMENTS
#define BOARD_BARCODE_RETRY_REVERSE_ELEMENTS                                   \
    29U /* a whole *X* (15 bars + 14 spaces) counted: the reverse may end      \
           before REVERSE_MIN_MS, still END_GAP+RUNUP past the read */
#endif
#ifndef BOARD_BARCODE_RETRY_IDLE_MS
#define BOARD_BARCODE_RETRY_IDLE_MS                                            \
    3000U /* no barcode state change while reversing/waiting = failed */
#endif
#ifndef BOARD_BARCODE_RETRY_RUNUP_MM
#define BOARD_BARCODE_RETRY_RUNUP_MM                                           \
    50U /* reverse this far beyond the end gap past the first bar */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_MIN_MS
#define BOARD_BARCODE_RETRY_REVERSE_MIN_MS                                     \
    5000U /* reverse at least this long before declaring failure */
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
 * the crossing. No junction within SEARCH_MM: it turns there. */
#ifndef BOARD_NAV_JUNCTION_OFFSET_MM
#define BOARD_NAV_JUNCTION_OFFSET_MM                                           \
    90U /* front IR sensors to the wheel axle */
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
 * UTURN pair. MAX_DEG 0 turns by encoder yaw alone. */
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
#if BOARD_NAV_TURN_POWER < 1 || BOARD_NAV_TURN_POWER > 50 ||                   \
    BOARD_NAV_TURN_SEEK_POWER < 1 || BOARD_NAV_TURN_SEEK_POWER > 50
#error Navigation turn powers must be 1..50 percent
#endif
#if (BOARD_NAV_TURN_MAX_DEG &&                                                 \
     (BOARD_NAV_TURN_MIN_DEG > 90 || BOARD_NAV_TURN_MAX_DEG < 90 ||            \
      BOARD_NAV_TURN_MAX_DEG > 180)) ||                                        \
    (BOARD_NAV_UTURN_MAX_DEG &&                                                \
     (BOARD_NAV_UTURN_MIN_DEG > 180 || BOARD_NAV_UTURN_MAX_DEG < 180 ||        \
      BOARD_NAV_UTURN_MAX_DEG > 255))
#error Navigation turn windows are invalid
#endif
#ifndef BOARD_NAV_UTURN_IGNORE_MM
#define BOARD_NAV_UTURN_IGNORE_MM                                              \
    1200U /* after a U-turn, re-reading the same marker does not turn again */
#endif
/* After a barcode U-turn, rotate on the spot (+/-30, +/-60 deg sweep) until a
 * line sensor is on black before tracking; a turn that ended on a line touch
 * needs none. 1: left/right turns do it too. */
#ifndef BOARD_NAV_SPOT_SCAN_ALL
#define BOARD_NAV_SPOT_SCAN_ALL 1
#endif
/* Steered reverse (PID builds): steer = -YAW_GAIN*theta_deg + LINE_GAIN*e,
 * where theta is yaw relative to the forward-tracking heading and e is +1/-1
 * when only the right/left straddle sensor is on black. The obstacle reverse
 * (VC_BACKUP) holds its heading with the same gains, line term off. */
#ifndef BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN
#define BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN                                   \
    40 /* steering per degree of heading error */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN
/* LINE/YAW sets the heading authority (200/40 = 5 deg) the line term has
 * against a wrong heading reference; below ~5x the robot drifts off. */
#define BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN                                  \
    200 /* steering when one sensor touches the line */
#endif
#ifndef BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT
#define BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT                                \
    300 /* |steer| bound, S2 accepts up to 400 */
#endif
#ifndef BOARD_BARCODE_RETRY_YAW_REF_DIV
#define BOARD_BARCODE_RETRY_YAW_REF_DIV                                        \
    50 /* heading EMA divisor: 50 VC ticks x 20 ms = 1.0 s */
#endif
/* Barcode-read line steering (vehicle_line_steer.h line_marker_steer): starts
 * at BOARD_LINE_STEER_MIN and ramps over BOARD_LINE_STEER_RAMP_MS to
 * BOARD_BARCODE_READ_LINE_STEER while the same straddle sensor stays on black.
 */
#ifndef BOARD_LINE_STEER_MIN
#define BOARD_LINE_STEER_MIN                                                   \
    200 /* first correction; 150 was too weak for bends, 250 wobbled on        \
           straights */
#endif
#ifndef BOARD_LINE_STEER_RAMP_MS
#define BOARD_LINE_STEER_RAMP_MS                                               \
    150U /* MIN -> read steer while the same sensor stays on black */
#endif
/* While a barcode is read (BOARD_BARCODE_READ_MIN_ELEMENTS or more) the
 * follower is off and the robot reads on at the slow speed; it may steer
 * gently toward a single straddle sensor on black, up to this (0: straight on).
 * 25 Nov: with 250, and the follower on until the 3rd bar, pivots and steering
 * under the sensor distorted the frame bars of most reads (the bars are
 * measured by wheel travel, which a pivot misstates). 0 now: a marker on a bend
 * may lose the line; markers belong on straights. */
#ifndef BOARD_BARCODE_READ_LINE_STEER
#define BOARD_BARCODE_READ_LINE_STEER 0
#endif
/* Completed bars/spaces before the follower switches off and the slow speed
 * starts: 0 = from the first black edge of a marker (8f: 3, so the first bars
 * were read at full speed with the follower still pivoting). A single crossing
 * line now also gets a short slow, straight stretch (until 30 mm of white). */
#ifndef BOARD_BARCODE_READ_MIN_ELEMENTS
#define BOARD_BARCODE_READ_MIN_ELEMENTS 0U
#endif
#if BOARD_BARCODE_READ_MIN_ELEMENTS > 29
#error BOARD_BARCODE_READ_MIN_ELEMENTS must be 0..29
#endif
#if BOARD_BARCODE_READ_LINE_STEER < 0 || BOARD_BARCODE_READ_LINE_STEER > 400
#error BOARD_BARCODE_READ_LINE_STEER must be 0..400
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
#define BOARD_BARCODE_MIN_RUN_POSITION                                         \
    4U /* half encoder counts (~0.7 mm): shorter AND briefer = noise */
#endif
#ifndef BOARD_BARCODE_DIGITAL_BLACK_LEVEL
#define BOARD_BARCODE_DIGITAL_BLACK_LEVEL                                      \
    1U /* GP28 was measured HIGH on black. */
#endif
#if BOARD_BARCODE_DIGITAL_BLACK_LEVEL != 0 &&                                  \
    BOARD_BARCODE_DIGITAL_BLACK_LEVEL != 1
#error Barcode digital black level must be zero or one
#endif
#ifndef BOARD_IR_MODE
#define BOARD_IR_MODE                                                          \
    3 /* 0 disabled, 1 digital, 2 ADC, 3 ADC line + GP28 digital barcode. */
#endif
/* Measured ADC endpoints (robot_ir_calibration.h) unless an override defines
 * BOARD_IR_ADC_CALIBRATED. Mode 2: GP26/27/28 all on the ADC. Mode 3 (since
 * 2026-11-09): GP26/GP27 ADC line sensors; GP28 barcode is a DIGITAL input
 * (module DO pin), HIGH = black (BOARD_BARCODE_DIGITAL_BLACK_LEVEL). */
#if (BOARD_IR_MODE == 2 || BOARD_IR_MODE == 3) &&                              \
    !defined(BOARD_IR_ADC_CALIBRATED)
#include "robot_ir_calibration.h"
#endif
#ifndef BOARD_IR_BLACK_LEVEL
#define BOARD_IR_BLACK_LEVEL 1
#endif
/* S3 latches a STALE fault (a line fault while tracking) when line samples
 * (every 10 ms) arrive further apart than this. Kept at S3's own 30 ms: a
 * longer gap inside a barcode would misjudge its bar widths. (The vehicle
 * controller holds still through its own short line-data gaps.) */
#ifndef BOARD_IR_MAX_SAMPLE_GAP_MS
#define BOARD_IR_MAX_SAMPLE_GAP_MS 30U
#endif
#ifndef BOARD_IR_LINE_MASK
#define BOARD_IR_LINE_MASK                                                     \
    3 /* GP26 right + GP27 left; GP28 is barcode-only.                         \
       */
#endif
#ifndef BOARD_BARCODE_SENSOR
#define BOARD_BARCODE_SENSOR                                                   \
    2 /* S3 index 2 = GP28; excluded from line mask.                           \
       */
#endif
#ifndef BOARD_IR_POSITIONS
#define BOARD_IR_POSITIONS                                                     \
    {1000, -1000, 0} /* Logical right/left; not millimetres. */
#endif
#ifndef BOARD_MAP_A
#define BOARD_MAP_A                                                            \
    4 /* Codebook: A=U-turn, Z=right, F=left; B-D unmapped (read, no turn). 1  \
         left 2 right 3 straight 4 U-turn */
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
/* Wheel geometry measured 2026-11-03 (calibration/README.md). Terrain tilt,
 * height and START all require valid odometry (0 = unmeasured). */
#ifndef BOARD_TICKS_PER_REV
#define BOARD_TICKS_PER_REV 631 /* measured x1 counts, not quadrature x4 */
#endif
#ifndef BOARD_WHEEL_CIRCUMFERENCE_MM
#define BOARD_WHEEL_CIRCUMFERENCE_MM 220
#endif
#ifndef BOARD_TRACK_WIDTH_MM
#define BOARD_TRACK_WIDTH_MM 113
#endif
#ifndef BOARD_TERRAIN_CALIBRATED
#define BOARD_TERRAIN_CALIBRATED 1 /* mounting axes and zeros measured */
#endif
/* GY-511 accelerometer mounting: the signed sensor axis that points to the
 * robot's front, its left and up (1=+X 2=+Y 3=+Z, negative = the minus
 * axis). Terrain works in that robot frame: pitch is positive nose-up,
 * roll positive left side up, and the zeros below are robot-frame angles
 * read standing still on level ground.
 * This robot: the sensor's -Y axis points to the front (forward acceleration
 * drives a_y negative), +X to its left, +Z up, and it sits ~15 deg nose-up. */
#ifndef BOARD_IMU_FRONT_AXIS
#define BOARD_IMU_FRONT_AXIS (-2)
#endif
#ifndef BOARD_IMU_LEFT_AXIS
#define BOARD_IMU_LEFT_AXIS 1
#endif
#ifndef BOARD_IMU_UP_AXIS
#define BOARD_IMU_UP_AXIS 3
#endif
#define BOARD_IMU_ABS_(a) ((a) < 0 ? -(a) : (a))
#if BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS) < 1 ||                                \
    BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS) > 3 ||                                \
    BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS) < 1 ||                                 \
    BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS) > 3 ||                                 \
    BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS) < 1 ||                                   \
    BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS) > 3
#error BOARD_IMU_*_AXIS must be +/-1, +/-2 or +/-3
#endif
/* Distinct axes forming a right-handed frame (front x left = up): the
 * permutation's parity times the three signs must be positive. */
#if (BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS) -                                     \
     BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS)) *                                   \
        (BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS) -                                   \
         BOARD_IMU_ABS_(BOARD_IMU_FRONT_AXIS)) *                               \
        (BOARD_IMU_ABS_(BOARD_IMU_UP_AXIS) -                                   \
         BOARD_IMU_ABS_(BOARD_IMU_LEFT_AXIS)) *                                \
        (BOARD_IMU_FRONT_AXIS) * (BOARD_IMU_LEFT_AXIS) *                       \
        (BOARD_IMU_UP_AXIS) <=                                                 \
    0
#error BOARD_IMU_FRONT/LEFT/UP_AXIS must be three different axes, right-handed
#endif
/* Straddle follower black/white decision, per mille of each sensor's
 * calibrated white..black range, with hysteresis: black at or above
 * BOARD_LINE_BLACK_PERMILLE, white again at or below BOARD_LINE_WHITE_PERMILLE.
 * S3's own line/barcode logic keeps its defaults. 2026-11-11: the ramp surface
 * reads 45-63% dark on the right sensor; 600/400 latched it black and faulted
 * the crossing. The line reads ~90%. */
#ifndef BOARD_LINE_BLACK_PERMILLE
#define BOARD_LINE_BLACK_PERMILLE 750U
#endif
#ifndef BOARD_LINE_WHITE_PERMILLE
#define BOARD_LINE_WHITE_PERMILLE 550U
#endif
#if BOARD_LINE_WHITE_PERMILLE >= BOARD_LINE_BLACK_PERMILLE ||                  \
    BOARD_LINE_BLACK_PERMILLE > 1000
#error Straddle thresholds: WHITE < BLACK <= 1000 per mille
#endif
/* Lifted: while running, both line IRs at or above BOARD_LIFT_RAW (no surface
 * under them: the ADC near full scale; the black line stays below ~3530) and
 * GP28 black for BOARD_LIFT_FAULT_MS stops the run with VC_LIFTED (0 = off). */
#ifndef BOARD_LIFT_RAW
#define BOARD_LIFT_RAW 3700U
#endif
#ifndef BOARD_LIFT_FAULT_MS
#define BOARD_LIFT_FAULT_MS 2000U
#endif
/* Mask-pivot follower (the default; 0: edge tracing, BOARD_LINE_EDGE_TRACE 1).
 * The teammates' working follower: right IR black -> pivot right, left IR
 * black -> pivot left, neither or both -> forward. No bend states, no
 * crossing/corner deadlines (junction turns, avoidance and barcode reads are
 * unchanged). Takes precedence over BOARD_LINE_EDGE_TRACE. */
#ifndef BOARD_LINE_MASK_PIVOT
#define BOARD_LINE_MASK_PIVOT 1
#endif
/* Line lost while tracking (mask-pivot follower; 0 = off). Both straddle IRs
 * white is also "centred", so BOARD_LINE_LOST_MM of forward travel with
 * neither on black starts the spot search (+/-30, +/-60 deg at <= 15%, toward
 * the side the line was last seen first, <= 4 s): a sensor on black resumes
 * tracking, none is fault 2 cause 3. Not during a barcode read or re-read, nor
 * while a decoded turn looks for its junction (BOARD_NAV_JUNCTION_SEARCH_MM).
 * Edge tracing keeps its own lost-line rule (a full turn with both IRs white).
 * 600: the IRs sit ~+/-13 mm about an ~18 mm line (90 mm ahead of the axle),
 * so both read white only within ~10 mm of side play. A bend of radius R
 * forces a touch within ~2*sqrt(2*R*10 mm): <= ~280 mm for R up to 1 m. On a
 * straight a one- or two-tick pivot (1.3-2.5 deg at 20 ms) crosses that play
 * in ~230-450 mm. Only drifting along one edge of a long straight with nearly
 * matched wheels runs longer; that costs a short search that finds the line
 * within a few degrees. Measure the longest both-white run on the track and
 * keep this about twice it. */
#ifndef BOARD_LINE_LOST_MM
#define BOARD_LINE_LOST_MM 600U
#endif
#if BOARD_LINE_LOST_MM > 10000
#error BOARD_LINE_LOST_MM must be 0 (off) .. 10000 mm
#endif
/* Edge tracing (vehicle_edge_trace.h), used when BOARD_LINE_MASK_PIVOT is 0.
 * One IR rides one edge of the line at BOARD_LINE_EDGE_SET darkness and
 * steers BOARD_LINE_EDGE_KP_MILLI steer units per 1000 darkness of error
 * (plus _KD_MILLI per sample change), up to +/-400. BOARD_LINE_EDGE_RIGHT 1:
 * the right IR rides the line's right edge. Power drops by up to
 * BOARD_LINE_EDGE_SLOW_PERMILLE at full steer. Darkness is per mille of each
 * sensor's calibrated white..black range. */
#ifndef BOARD_LINE_EDGE_TRACE
#define BOARD_LINE_EDGE_TRACE 0
#endif
#if !BOARD_LINE_MASK_PIVOT && !BOARD_LINE_EDGE_TRACE
#error Select one line follower
#endif
#ifndef BOARD_LINE_EDGE_RIGHT
#define BOARD_LINE_EDGE_RIGHT 1
#endif
#ifndef BOARD_LINE_EDGE_SET
#define BOARD_LINE_EDGE_SET 500
#endif
#ifndef BOARD_LINE_EDGE_KP_MILLI
#define BOARD_LINE_EDGE_KP_MILLI 800
#endif
#ifndef BOARD_LINE_EDGE_KD_MILLI
#define BOARD_LINE_EDGE_KD_MILLI 0
#endif
#ifndef BOARD_LINE_EDGE_DEEP
#define BOARD_LINE_EDGE_DEEP 920 /* edge IR nearly fully on the line ... */
#endif
#ifndef BOARD_LINE_EDGE_DEEP_MS
#define BOARD_LINE_EDGE_DEEP_MS                                                \
    250U /* ... this long: pivot toward the edge side */
#endif
#ifndef BOARD_LINE_EDGE_SLOW_PERMILLE
#define BOARD_LINE_EDGE_SLOW_PERMILLE 350
#endif
#if BOARD_LINE_EDGE_SET < 100 || BOARD_LINE_EDGE_SET > 900 ||                  \
    BOARD_LINE_EDGE_KP_MILLI < 0 || BOARD_LINE_EDGE_SLOW_PERMILLE < 0 ||       \
    BOARD_LINE_EDGE_SLOW_PERMILLE > 900
#error Edge tracing: set 100..900, kp >= 0, slow 0..900
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
#define BOARD_RAMP_SPEED 1
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
#define BOARD_RAMP_HOLD_MS                                                     \
    400U /* rejects the ~0.25 s pitch bump at the ramp entry */
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
/* 15: at 10% the robot stalled at a hump's crest edge on the track
 * (2026-11-20, fault 7); 15% gives the speed loop more drive there. */
#define BOARD_RAMP_DOWN_POWER 15U
#endif
#if BOARD_RAMP_SPEED &&                                                        \
    (BOARD_RAMP_UP_POWER > 50 || BOARD_RAMP_DOWN_POWER > 50 ||                 \
     BOARD_RAMP_LEVEL_CDEG >= BOARD_RAMP_TILT_CDEG ||                          \
     BOARD_IMU_SMOOTH_MS < 1 || BOARD_IMU_SMOOTH_MS > 5000)
#error Ramp test settings are out of range
#endif
/* Level-ground zeros, measured 2026-11-11 over MQTT: 161 still samples in
 * three level windows, accel median (-54,-250,909) mg, sd <7 mg -> pitch
 * +15.38 deg, roll -3.40 deg (per-sample sd 0.36 / 0.31 deg). Checks the same
 * session: front raised read +19.6 deg pitch (roll -1.1), right side raised
 * -13.6 deg roll, so the axis signs hold. */
#ifndef BOARD_PITCH_ZERO_CDEG
#define BOARD_PITCH_ZERO_CDEG 1538
#endif
#ifndef BOARD_ROLL_ZERO_CDEG
#define BOARD_ROLL_ZERO_CDEG (-340)
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
#define BOARD_BYPASS_VALIDATED                                                 \
    1 /* obstacle bypass on (0: stop at an obstacle) */
#endif
/* Speed PID and wheel trims: accepted 2026-11-06 floor calibration
 * (calibration/motor_recalibration_2026-11-06.json, 1088 vs 1094 counts in
 * 2 s at 35%, "looked straight"). The 2026-11-04 trial (MAX_CPS 2500, ceiling
 * 30, trims 73/98) asked for 750 cps at 30% demand, which the 30% PWM ceiling
 * cannot reach: both wheels saturated (~500 cps) and the PID stopped
 * regulating, so wheel speed followed each motor's raw strength.
 * 2026-11-09 18:30 run (11-06 values, MAX_CPS 1700): the feedforward alone
 * drove 30% demand to L 594 / R 628 cps (target 510) and 9% reverse to
 * L 192 / R 182 (target 153). MAX_CPS 1950 makes the targets match what the
 * feedforward delivers at both points (585 and 176 cps), the right trim
 * cancels its 6% excess, and Ki 40 (was 10) settles a speed error in ~1 s
 * instead of ~4 s. */
#ifndef BOARD_PID_ENABLED
#define BOARD_PID_ENABLED 1
#endif
#ifndef BOARD_PID_CALIBRATED
#define BOARD_PID_CALIBRATED 1
#endif
#ifndef BOARD_PID_MAX_CPS
#define BOARD_PID_MAX_CPS                                                      \
    1950 /* measured wheel counts/s at the reference full scale */
#endif
#ifndef BOARD_PID_KP_MILLI
#define BOARD_PID_KP_MILLI 30
#endif
#ifndef BOARD_PID_KI_MILLI
#define BOARD_PID_KI_MILLI 40
#endif
#ifndef BOARD_PID_KD_MILLI
#define BOARD_PID_KD_MILLI 0
#endif
#ifndef BOARD_PID_MAX_POWER
#define BOARD_PID_MAX_POWER 35
#endif
#ifndef S2_MOTOR1_TRIM_PERCENT
#define S2_MOTOR1_TRIM_PERCENT 86U /* left wheel duty scale (subsystem2.h) */
#endif
#ifndef S2_MOTOR2_TRIM_PERCENT
#define S2_MOTOR2_TRIM_PERCENT 93U /* right wheel duty scale */
#endif
#if BOARD_PID_ENABLED != 0 && BOARD_PID_ENABLED != 1
#error BOARD_PID_ENABLED must be 0 or 1
#endif
#if BOARD_PID_ENABLED &&                                                       \
    (BOARD_PID_KP_MILLI < 0 || BOARD_PID_KP_MILLI > 10000 ||                   \
     BOARD_PID_KI_MILLI < 0 || BOARD_PID_KI_MILLI > 10000 ||                   \
     BOARD_PID_KD_MILLI < 0 || BOARD_PID_KD_MILLI > 10000 ||                   \
     (BOARD_PID_KP_MILLI + BOARD_PID_KI_MILLI + BOARD_PID_KD_MILLI) == 0)
#error PID gains must be in 0..10000 with at least one nonzero feedback gain
#endif
#if BOARD_PID_ENABLED &&                                                       \
    (BOARD_PID_MAX_CPS < 1 || BOARD_PID_MAX_CPS > 100000 ||                    \
     BOARD_PID_MAX_POWER < 1 || BOARD_PID_MAX_POWER > 50)
#error PID requires a measured speed scale and an output cap of 1..50 percent
#endif
/* In-place spins (PID builds, subsystem2/turn_drive.h; navigation turns and
 * the spot scan): tyre scrub needs more torque than straight driving. A spin
 * may use up to TURN_MAX_POWER % PWM, holds each wheel at >= KICK_POWER %
 * until it has moved KICK_COUNTS (at most KICK_MS), kicks a wheel again after
 * REKICK_MS without progress, and faults only after STALL_MS without progress
 * (straight driving and follower pivots: 500 ms). */
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
#define BOARD_TURN_KICK_COUNTS                                                 \
    5 /* encoder counts of real motion that end a spin kick */
#endif
#ifndef BOARD_TURN_REKICK_MS
#define BOARD_TURN_REKICK_MS                                                   \
    100U /* a spinning wheel this long without progress is kicked again */
#endif
#ifndef BOARD_TURN_STALL_MS
#define BOARD_TURN_STALL_MS 1200U
#endif
#if BOARD_PID_ENABLED &&                                                       \
    (BOARD_PID_TURN_MAX_POWER < BOARD_PID_MAX_POWER ||                         \
     BOARD_PID_TURN_MAX_POWER > 100 || BOARD_TURN_KICK_POWER < 0 ||            \
     BOARD_TURN_KICK_POWER > BOARD_PID_TURN_MAX_POWER ||                       \
     BOARD_TURN_KICK_MS > 1000 || BOARD_TURN_STALL_MS < 500 ||                 \
     BOARD_TURN_STALL_MS > 5000)
#error Turn drive limits, kick or stall settings are out of range
#endif
/* Low-speed breakaway (PID builds, turn_drive.h): a straight, reverse or
 * follower-pivot move commanded below LOW_KICK_BELOW % gets the spin kick's
 * timing (KICK_MS, KICK_COUNTS, REKICK_MS) at LOW_KICK_POWER % PWM. The
 * barcode re-read drives at 6-10 % and stalled from standstill (fault 7,
 * 2026-11-20 and 2026-11-24). Moves at or above LOW_KICK_BELOW (normal 20/30 %
 * runs, the 15 % crest and search caps) are unchanged. 0: off. */
#ifndef BOARD_LOW_KICK_BELOW
#define BOARD_LOW_KICK_BELOW 15U
#endif
#ifndef BOARD_LOW_KICK_POWER
#define BOARD_LOW_KICK_POWER 30U
#endif
#if BOARD_PID_ENABLED &&                                                       \
    (BOARD_LOW_KICK_BELOW > 100 || BOARD_LOW_KICK_POWER > BOARD_PID_MAX_POWER)
#error Low-speed kick settings are out of range
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
/* Pulse change for 90 deg. Measure at both +/-45 and +/-90 deg. */
#ifndef BOARD_SERVO_US_PER_90
#define BOARD_SERVO_US_PER_90 750
#endif
#ifndef BOARD_SERVO_CENTER_US
#define BOARD_SERVO_CENTER_US 1554
#endif
#if BOARD_SERVO_CENTER_US < 1300 || BOARD_SERVO_CENTER_US > 1700
#error BOARD_SERVO_CENTER_US is the straight-ahead pulse: 1300..1700 us
#endif
#endif

/*** end of file ***/
