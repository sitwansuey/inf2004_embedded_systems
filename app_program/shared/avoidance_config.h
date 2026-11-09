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
#endif
