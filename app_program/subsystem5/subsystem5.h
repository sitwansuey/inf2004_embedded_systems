#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H
#include "../app.h"
#define S5_PRI_MAIN 20
#define S5_OBSTACLE_MM 200U /* acquisition at 20 cm, inclusive */
#define S5_ACQUISITION_SAMPLES 2U
#define S5_SCAN_ECHO_FAILURES 3U /* retry at the same angle before resetting */
#define S5_STALE_MS 500U
#define S5_STARTUP_CENTER_MS 1000U /* allow travel from any initial position */
#define S5_SETTLE_MS 150
#define S5_SAMPLE_MS 65 /* datasheet: measurement cycle over 60 ms */
#define S5_MAX_ANGLE 50
#define S5_SCAN_STEP_DEG 10
#define S5_REFINE_STEP_DEG 1
#define S5_TARGET_DEPTH_TOLERANCE_MM 20U
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
static inline s5_stage_t s5_stage_for_state(s5_state_t state)
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
typedef struct
{
    s5_state_t state;
    INT angle_deg;
    INT sample_angle_deg; /* actual angle associated with distance_mm */
    INT avoidance_deg;    /* selected heading: positive left, negative right */
    UW scan_id;
    BOOL acquisition_clear;   /* confirmed no-echo opening, not a valid distance */
    BOOL approach_plan_valid; /* acquisition finished and verified */
    INT planned_rotate_deg;
    UW planned_rotate_ms, planned_approach_mm, planned_return_mm;
    UW obstacle_distance_mm; /* forward distance when this scan began */
    UW distance_mm;
    UW echo_us; /* raw high-pulse duration; partial on no-fall timeout */
    UW time_ms;
    /* Wheel position at echo start/end; forward is positive on both wheels.
     * Unsigned subtraction followed by INT cast gives a wrap-safe count delta. */
    INT motor1_counts_start, motor2_counts_start;
    INT motor1_counts_end, motor2_counts_end;
    UW encoder_start_ms, encoder_end_ms;
    BOOL encoder_valid;
    BOOL valid;
    s5_echo_result_t echo_result;
    INT left_bound_deg;  /* first object-hit angle, positive left */
    INT right_bound_deg; /* first object-hit angle, negative right */
    BOOL left_bound_valid, right_bound_valid;
    UW left_bound_distance_mm, right_bound_distance_mm; /* mean of 3 fresh samples */
    BOOL object_width_valid;
    UW object_width_mm;
    UW object_width_min_mm;
    UW object_width_max_mm;
} s5_status_t;
IMPORT ER s5_get_status(s5_status_t *status);
/* Legacy single-rotation handshake. S5 centres the sensor and
 * monitors forward distance until completion or abort. */
IMPORT ER s5_avoidance_begin(UW scan_id);
/* Call after the controller stops its completed or aborted turn.
 * Enters forward guidance with the 5 cm limit until forward exceeds 10 cm. */
IMPORT ER s5_avoidance_complete(void);
/* Main requests a navigation substate and look direction. The S5 task owns
 * servo movement and invalidates the old sample until settling completes. */
IMPORT ER s5_navigation_set(UW scan_id, s5_state_t state, INT look_deg);
/* S5 owns obstacle thresholds and interpretation; main owns motion phases. */
#define S5_GUIDANCE_OBSTACLE_MM 50U
#define S5_ROTATE_STOP_MM S5_GUIDANCE_OBSTACLE_MM
#define S5_ROTATE_WAIT_MS 2000U
#define S5_ROTATE_MS_PER_DEG 5U
#define S5_SENSOR_OFFSET_MM 90U
#define S5_CLEARANCE_RADIUS_MM 75U
#define S5_CRUISE_MM_PER_SECOND 245U /* measured 1.20 rev/s with 65 mm wheels at 30% power */
#define S5_OVERCOME_CLEARANCE_MM 200U
#define S5_OBJECT_RANGE_MM 50U
#define S5_NAVIGATION_TIMEOUT_MS 30000U
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
    INT turn_deg; /* clearing angle plus clearance margin, signed */
    UW turn_ms;
    UW approach_mm, return_mm; /* timed diagonal and return-to-line legs */
} s5_path_t;
/* Navigation uses the 5 cm limit; ordinary cruise uses 20 cm. */
static inline UW s5_obstacle_threshold(s5_state_t state)
{
    return state == S5_MONITOR || state == S5_CENTERING || state == S5_FAULT
               ? S5_OBSTACLE_MM
               : S5_GUIDANCE_OBSTACLE_MM;
}
#if APP_S5_MASTER_TEST
IMPORT void s5_master_test(void);
#endif
#if APP_ACQUISITION_TEST || APP_S5_MASTER_TEST
IMPORT BOOL acquisition_test_run(s5_status_t *result);
IMPORT void aqcuisition_test(void);
#endif
#if APP_ACQUISITION_LEFT_TEST
IMPORT void acquisition_left_test(void);
#endif
#if APP_TEST
/* Test-only API: command an angle, wait 1 s, optionally measure once. */
IMPORT ER s5_test_position(INT angle, BOOL take_sample, s5_status_t *copy);
#endif
IMPORT s5_path_t s5_assess_path(const s5_status_t *sensor, BOOL available, UW now_ms);
#endif
