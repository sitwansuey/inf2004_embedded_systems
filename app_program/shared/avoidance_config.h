#ifndef AVOIDANCE_CONFIG_H
#define AVOIDANCE_CONFIG_H
#define AVOID_MAX_WIDTH_MM 500U
#define AVOID_CLEARANCE_MM 75U
#define AVOID_POWER 25U
#define AVOID_SIDE_CLEAR_MM 200U
#define AVOID_SIDE_CONFIRM_SAMPLES 3U
#define AVOID_SIDE_CHECK_TIMEOUT_MS 4000U
#define AVOID_PASS_CHECK_MIN_MM 100U
#define AVOID_PASS_CHECK_MAX_MM 250U
/* VC_BACKUP: straight reverse after the plan is latched, ended by time or
 * by the odometry distance bound, whichever comes first. */
#define AVOID_BACKUP_MS 1000U
#define AVOID_BACKUP_MAX_MM 300U
#define AVOID_BACKUP_POWER 25U
/* Stay stopped until S5 has left its scan (its encoder motion guard would
 * otherwise discard the published plan and widths), at most this long. */
#define AVOID_BACKUP_SETTLE_MS 300U
/* Odometry moving forward by more than this while reversing is a motor fault. */
#define AVOID_BACKUP_WRONG_WAY_MM 20U
/* The reverse ramps START_POWER -> POWER over RAMP_MS and back down over its
 * last RAMP_MS: a step into or out of 25% rocks the robot, and the gravity-only
 * tilt drops out (VC_SENSOR tilt) or reads above 1.8 g. */
#define AVOID_BACKUP_START_POWER 10U
#define AVOID_BACKUP_RAMP_MS 300U
/* Then stand still before the forward turn-out (a direct reverse to forward
 * switch measured 2.3 g): at least BRAKE_MS, until odometry has stayed within
 * STILL_UM for STILL_MS, at most BRAKE_MAX_MS. */
#define AVOID_BACKUP_BRAKE_MS 300U
#define AVOID_BACKUP_STILL_MS 150U
#define AVOID_BACKUP_STILL_UM 1000
#define AVOID_BACKUP_BRAKE_MAX_MS 1000U
#endif
