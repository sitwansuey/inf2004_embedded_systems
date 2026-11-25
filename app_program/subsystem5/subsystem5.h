#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"
#include "../robot_config.h"
#include "../shared/avoidance_config.h"
#define S5_PRI_MAIN 20
#define S5_LOCK_WAIT_MS 5
/* Acquisition at 20 cm, inclusive. The vehicle detects an obstacle earlier
 * (VC_DETECT_MM, vehicle_approach.h) and holds still about here for it. */
#define S5_OBSTACLE_MM 200U
/* Consecutive forward echoes within S5_OBSTACLE_MM that confirm an obstacle
 * (S5 starts acquisition; the vehicle stops tracking). A far or off-axis
 * echo restarts the count; a no-echo retry is no reading and neither counts
 * nor restarts it, but three in a row (open space) do. 2 (3 in 8c). */
#define S5_ACQUISITION_SAMPLES 2U
#define S5_SCAN_MOTION_TOLERANCE_COUNTS                                        \
    3 /* about 1 mm with measured 631 counts / 220 mm */
#define S5_SCAN_MOTION_RESTARTS 2U
#define S5_SCAN_ECHO_FAILURES                                                  \
    3U /* retry at the same angle before resetting                             \
        */
#define S5_STALE_MS 500U
#define S5_INVALID_STALE_MS                                                    \
    2250U /* max 180-degree servo travel plus scheduling margin */
#define S5_STARTUP_CENTER_MS                                                   \
    1000U /* allow travel from any initial position                            \
           */
#define S5_SETTLE_MS 150
#define S5_SAMPLE_MS 65 /* datasheet: measurement cycle over 60 ms */
#define S5_WIDTH_PUBLISH_MS                                                    \
    300U /* widths published, vehicle stopped, before the side is chosen */
#define S5_MAX_ANGLE 90 /* servo and geometry limit */
/* Acquisition: left at 20, 40, 60 deg until one is clear, then the edge is
 * bisected to 1 deg between the last object and the first clear angle; the
 * right is checked at the mirrored angle (the object there: searched outward
 * the same way). */
#define S5_SCAN_STEP_DEG 20
#define S5_SCAN_COARSE_STEPS 3
#define S5_REFINE_STEP_DEG 1
#define S5_TARGET_DEPTH_TOLERANCE_MM 20U
/* The HC-SR04 beam still returns an obstacle's corner about this far past
 * its true edge, so a sweep finds each edge this much too wide (24 Nov: a
 * 15 cm box at 18 cm gave edges at 41 and 45 deg instead of ~23 deg). Only the
 * REPORTED width subtracts it (s5_width_reported: telemetry and the event
 * log); the bypass plan keeps the measured bounds and their clearance.
 * 0: report the uncorrected tangent width. */
#ifndef S5_BEAM_HALF_DEG
#define S5_BEAM_HALF_DEG 20
#endif
#if S5_BEAM_HALF_DEG < 0 || S5_BEAM_HALF_DEG > 45
#error S5_BEAM_HALF_DEG must be 0..45
#endif
#define S5_SERVO_MS_PER_DEG 10U
/* Positive angles point left of the vehicle's forward direction. */
typedef enum
{
    S5_MONITOR,
    S5_SCAN_LEFT,
    S5_REFINE_LEFT,
    S5_SCAN_RIGHT,
    S5_REFINE_RIGHT,
    S5_VERIFY_CLEAR,
    S5_READY,
    S5_BLOCKED,
    S5_FAULT,
    S5_ROTATING,
    S5_CENTERING,
    S5_GUIDING,
    S5_APPROACH,
    S5_OVERCOME,
    S5_REORG
} s5_state_t;
/* Four avoidance stages group the detailed sensor/motion substates. */
typedef enum
{
    S5_STAGE_NONE,
    S5_STAGE_ACQUISITION,
    S5_STAGE_APPROACH,
    S5_STAGE_OVERCOME,
    S5_STAGE_REORG
} s5_stage_t;
static inline s5_stage_t
s5_stage_for_state (s5_state_t state)
{
    switch (state)
    {
        case S5_SCAN_LEFT:
        case S5_REFINE_LEFT:
        case S5_SCAN_RIGHT:
        case S5_REFINE_RIGHT:
        case S5_VERIFY_CLEAR:
        case S5_READY:
        case S5_BLOCKED:
            return S5_STAGE_ACQUISITION;
        case S5_ROTATING:
        case S5_APPROACH:
            return S5_STAGE_APPROACH;
        case S5_OVERCOME:
            return S5_STAGE_OVERCOME;
        case S5_REORG:
            return S5_STAGE_REORG;
        default:
            return S5_STAGE_NONE;
    }
}
typedef enum
{
    S5_ECHO_OK,
    S5_ECHO_STUCK_HIGH,
    S5_ECHO_NO_RISE,
    S5_ECHO_NO_FALL,
    S5_ECHO_TOO_SHORT
} s5_echo_result_t;
/* Decoded API view. Shared storage and EDR use s5_data.h's explicit packed
 * bytes; never transmit this native structure or depend on its padding. */
typedef struct
{
    s5_state_t state;
    INT        angle_deg;
    INT        sample_angle_deg; /* actual angle associated with distance_mm */
    INT  avoidance_deg; /* selected heading: positive left, negative right */
    UW   scan_id;
    BOOL acquisition_clear;   /* directional clearance; repeated no-return may
                                 establish assumed open space */
    BOOL approach_plan_valid; /* acquisition finished and verified */
    INT  planned_rotate_deg;
    UW   planned_rotate_ms, planned_approach_mm, planned_return_mm;
    UW   obstacle_distance_mm; /* forward distance when this scan began */
    UW   distance_mm;
    UW   echo_us; /* raw high-pulse duration; partial on no-fall timeout */
    UW   time_ms;
    /* Wheel position at echo start/end; forward is positive on both wheels.
     * Unsigned subtraction followed by INT cast gives a wrap-safe count delta.
     */
    INT              motor1_counts_start, motor2_counts_start;
    INT              motor1_counts_end, motor2_counts_end;
    UW               encoder_start_ms, encoder_end_ms;
    BOOL             encoder_valid;
    BOOL             valid;
    s5_echo_result_t echo_result;
    INT              left_bound_deg; /* first object-hit angle, positive left */
    INT  right_bound_deg; /* first object-hit angle, negative right */
    BOOL left_bound_valid, right_bound_valid;
    UW   left_bound_distance_mm,
        right_bound_distance_mm; /* mean of 3 fresh samples */
    BOOL object_width_valid;
    UW   object_width_mm;
    UW   object_width_min_mm;
    UW   object_width_max_mm;
    /* Centre (forward axis) to left/right edge; 0 unless object_width_valid. */
    UW object_left_mm;
    UW object_right_mm;
    /* One bound is the other edge mirrored: that side was blocked through
     * 60 deg or its edge failed. object_width_mm is then one-sided (twice the
     * measured half; a lower bound when blocked) and only the measured side
     * is planned. FALSE: both edges measured. */
    BOOL width_one_sided;
} s5_status_t;
/* User-selected policy: three no-return samples at one settled angle imply
 * open space. valid stays FALSE and distance stays unknown on the wire. */
static inline BOOL
s5_sample_is_open (const s5_status_t * s)
{
    return s && !s->valid && s->acquisition_clear &&
           s->sample_angle_deg == s->angle_deg &&
           (s->echo_result == S5_ECHO_NO_RISE ||
            s->echo_result == S5_ECHO_NO_FALL);
}
static inline BOOL
s5_direction_clear (const s5_status_t * s, UW threshold)
{
    return s && s->sample_angle_deg == s->angle_deg &&
           (s5_sample_is_open(s) || (s->valid && s->distance_mm > threshold));
}

IMPORT ER s5_controller_bind (void);
IMPORT ER s5_get_status (s5_status_t * status);
/* Legacy single-rotation handshake. S5 centres the sensor and
 * monitors forward distance until completion or abort. */
IMPORT ER s5_avoidance_begin (UW scan_id);
/* Call after the controller stops its completed or aborted turn.
 * Enters forward guidance with the 5 cm limit until forward exceeds 10 cm. */
IMPORT ER s5_avoidance_complete (void);
/* Main requests a navigation substate and look direction. The S5 task owns
 * servo movement and invalidates the old sample until settling completes. */
IMPORT ER s5_navigation_set (UW scan_id, s5_state_t state, INT look_deg);
/* S5 owns obstacle thresholds and interpretation; main owns motion phases. */
#define S5_GUIDANCE_OBSTACLE_MM 50U
#define S5_ROTATE_STOP_MM S5_GUIDANCE_OBSTACLE_MM
#define S5_ROTATE_MS_PER_DEG 5U
#define S5_SENSOR_OFFSET_MM 90U
#define S5_CLEARANCE_RADIUS_MM AVOID_CLEARANCE_MM
#define S5_BYPASS_ARC_RADIUS_MM                                                \
    150U /* center path; steering derives from measured axle width */
#define S5_CRUISE_MM_PER_SECOND                                                \
    245U /* measured 1.20 rev/s with 65 mm wheels at 30% power */
#define S5_OVERCOME_CLEARANCE_MM 200U
#define S5_NAVIGATION_TIMEOUT_MS 30000U
/* Side corridor (checked before a side is chosen). The robot centre goes out
 * to the planned lateral offset; its outer side reaches this much further. */
#ifndef S5_SIDE_HALF_WIDTH_MM
#define S5_SIDE_HALF_WIDTH_MM S5_CLEARANCE_RADIUS_MM
#endif
/* Echoes up to this far beyond the face depth lie beside the diagonal and
 * the pass; farther ones are ignored. */
#ifndef S5_SIDE_CORRIDOR_AHEAD_MM
#define S5_SIDE_CORRIDOR_AHEAD_MM 300U
#endif
typedef enum
{
    S5_PATH_UNAVAILABLE,
    S5_PATH_STOP,
    S5_PATH_FORWARD,
    S5_PATH_READY,
    S5_PATH_ROTATE_CLEAR,
    S5_PATH_FAULT,
    S5_PATH_IDENTIFY,
    S5_PATH_GUIDE_FORWARD
} s5_path_state_t;
typedef struct
{
    s5_path_state_t state;
    INT             turn_deg; /* clearing angle plus clearance margin, signed */
    UW              turn_ms;
    UW approach_mm, return_mm; /* timed diagonal and return-to-line legs */
} s5_path_t;
#if APP_S5_MASTER_TEST
IMPORT void s5_master_test (void);
#endif
#if APP_ACQUISITION_TEST || APP_S5_MASTER_TEST
IMPORT BOOL acquisition_test_run (s5_status_t * result);
IMPORT void aqcuisition_test (void);
#endif
#if APP_ACQUISITION_LEFT_TEST
IMPORT void acquisition_left_test (void);
#endif
#if APP_TEST && APP_S5_DIAGNOSTICS
IMPORT void s5_servo_sweep_test (void);
/* Test-only API: command an angle, wait 1 s, optionally measure once. */
IMPORT ER s5_test_position (INT angle, BOOL take_sample, s5_status_t * copy);
#endif
IMPORT s5_path_t s5_assess_path (const s5_status_t * sensor, BOOL available,
                                 UW now_ms);
/* Planned lateral offset of the robot centre for a side's bypass: the larger
 * of that edge's lateral offset and half the planning width, plus the
 * clearance radius. 0 when that bound or the width is not usable. */
IMPORT UW s5_plan_lateral_mm (const s5_status_t * sensor, BOOL left);
/* TRUE when the sample's echo lies inside the corridor: lateral offset
 * (range*sin) at most lateral_mm + S5_SIDE_HALF_WIDTH_MM and forward offset
 * (range*cos) at most ahead_mm, both from the sensor. Uses sample_angle_deg
 * and distance_mm; the caller checks the sample is settled at its angle.
 * No valid echo is FALSE (nothing seen). */
IMPORT BOOL s5_corridor_blocked (const s5_status_t * sample, UW lateral_mm,
                                 UW ahead_mm);
#endif

/*** end of file ***/
