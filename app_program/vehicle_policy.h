#ifndef CSC02_VEHICLE_POLICY_H
#define CSC02_VEHICLE_POLICY_H
#include <stdint.h>
#include <stdbool.h>
#include "shared/avoidance_config.h"
enum { VC_DISARMED,VC_TRACK,VC_ACQUIRE,VC_TURN_OUT,VC_DIAGONAL,VC_ALIGN,
       VC_PASS,VC_TURN_IN,VC_RETURN,VC_RESTORE,VC_REACQUIRE,VC_NAV_TURN,VC_FAULT,
       VC_BACKUP /* appended: reverse after ACQUIRE, before TURN_OUT */ };
enum { VC_OK,VC_SENSOR,VC_LINE,VC_SHOCK,VC_TRANSPORT,VC_TIMEOUT,VC_UNCALIBRATED,VC_MOTOR };
#define VC_TRACK_HANDOFF_MS 100U
typedef struct {
    uint32_t now_ms, scan_id, side_sample_ms;
    int32_t distance_um, yaw_mdeg;
    int16_t line_error, turn_deg;
    uint16_t approach_mm, return_mm, obstacle_width_mm;
    uint8_t sensor_state;
    uint8_t line_state;
    uint8_t line_black_mask; /* bit 0 right, bit 1 left */
    uint8_t nav;
    /* A decoded turn waits for the next junction, then drives on nav_offset_mm
     * (line sensors to wheels) before pivoting; with no junction it turns after
     * nav_search_mm. 0/0 turns at once (builds without a junction signal). */
    uint16_t nav_offset_mm, nav_search_mm;
    /* After a U-turn the robot drives back over the same marker: ignore
     * (acknowledge) another U-turn within this distance. 0 disables. */
    uint16_t uturn_ignore_mm;
    /* VC_NAV_TURN spins at nav_turn_power (0: maximum_power) by encoder yaw;
     * from the min angle on it continues at nav_seek_power (0: the same)
     * until line_touch, and ends by the max angle regardless. A touch before
     * the min angle (old line, crossing) never ends it. Degrees for 90 deg
     * turns, then U-turns; max 0 ends exactly at 90/180 by yaw alone. */
    uint8_t nav_turn_power, nav_seek_power;
    uint8_t nav_turn_min_deg, nav_turn_max_deg, nav_uturn_min_deg, nav_uturn_max_deg;
    uint16_t bypass_steering; /* track width / (2 * planned arc radius), permille */
    uint8_t line_fresh : 1;
    uint8_t steering : 1;
    uint8_t front_clear : 1;
    uint8_t obstacle : 1;
    uint8_t scan_ready : 1;
    uint8_t sensor_fresh : 1;
    uint8_t width_valid : 1;
    uint8_t scan_blocked : 1;
    uint8_t side_observed : 1;
    uint8_t side_clear : 1;
    uint8_t side_ready : 1;
    uint8_t odom_valid : 1;
    uint8_t shock : 1;
    uint8_t terrain_valid : 1;
    uint8_t slope : 1;
    uint8_t motor_ok : 1;
    uint8_t bypass_enabled : 1;
    uint8_t navigation_event : 1;
    uint8_t safety_event : 1;
    uint8_t transport_ok : 1;
    uint8_t recovery_event : 1;
    uint8_t line_detected : 1;
    uint8_t curve_hold : 1; /* bounded turn memory across brief IR gaps */
    uint8_t nav_hold : 1; /* defer a decoded turn while a barcode re-read repositions */
    uint8_t sensor_acquiring : 1; /* S5 still in an acquisition-stage state (its motion guard is armed) */
    uint8_t junction : 1; /* crossing line: both line sensors and GP28 on black */
    uint8_t line_touch : 1; /* fresh sample with a line sensor on black */
    uint8_t spot_scan_all : 1; /* left/right nav turns also spot-scan (a U-turn always does) unless ended on a touch */
} vc_input_t;
typedef struct {
    uint32_t entered_ms, armed_ms, lease_ms, scan_id;
    uint32_t track_requested_ms;
    uint32_t line_found_ms;
    uint32_t bypass_drive_ms, last_step_ms;
    int32_t origin_um, origin_yaw, turn_progress;
    int32_t nav_origin_um; /* where the pending turn's junction search / offset began */
    int32_t uturn_end_um; /* odometry where the last U-turn finished */
    uint16_t approach_mm, return_mm, pass_check_mm;
    uint16_t backup_mm; /* distance reversed in VC_BACKUP, added to the first pass check only */
    uint32_t backup_started_ms; /* first reversing tick of VC_BACKUP */
    uint32_t brake_started_ms, brake_moved_ms; /* VC_BACKUP stand-still: start, last odometry movement */
    int32_t brake_last_um; /* odometry at the last movement while braking */
    uint32_t soft_start_ms; /* avoidance arc started driving from standstill */
    uint32_t side_last_ms, side_check_started_ms;
    uint8_t side_clear_count, side_present_count, side_have_sample;
    uint16_t bypass_steering;
    int16_t turn_deg;
    int16_t steering; /* forward bend, negative left / positive right */
    uint8_t phase : 4;
    uint8_t fault : 4;
    uint8_t command : 3;
    uint8_t power : 7;
    uint8_t maximum_power : 7;
    uint8_t look_side : 1;
    uint8_t armed : 1;
    uint8_t side : 1; /* 0 left, 1 right */
    uint8_t pending_nav : 3; /* retained until execution, STOP or fault */
    uint8_t navigation_ack : 1; /* one-tick STRAIGHT acknowledgement */
    uint8_t track_wait : 1;
    uint8_t search_command : 3;
    uint8_t line_settling : 1;
    uint8_t last_line_side : 2; /* 0 unknown, 1 right, 2 left */
    uint8_t search_right_first : 1;
    uint8_t backup_moving : 1; /* VC_BACKUP has started reversing */
    uint8_t backup_braking : 1; /* VC_BACKUP reverse done: standing still before TURN_OUT */
    uint8_t soft_start : 1; /* TURN_OUT..RESTORE power still ramping up (soft_start_ms) */
    uint8_t drive_prev : 1; /* the previous tick commanded forward with power */
    uint8_t nav_touched : 1; /* VC_NAV_TURN: a straddle sensor touched the new line (past min) */
    uint8_t fault_cause : 2; /* last VC_SENSOR: 0 other, 1 sonar, 2 tilt, 3 odometry */
    uint8_t nav_search : 1; /* pending turn actionable in TRACK: looking for its junction */
    uint8_t nav_offset : 1; /* junction seen: straight on for nav_offset_mm, then turn */
    uint8_t spot_scan : 1; /* REACQUIRE after a turn ended without a line touch: only a sensor on black ends it */
    uint8_t uturn_guard : 1; /* uturn_end_um is valid: repeat U-turns are ignored nearby */
} vc_state_t;
enum { VC_CAUSE_OTHER,VC_CAUSE_SONAR,VC_CAUSE_TILT,VC_CAUSE_ODOM };
void vc_track_requested(vc_state_t *s, uint32_t now);
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power);
bool vc_start(vc_state_t *s, uint32_t now, unsigned power);
void vc_stop(vc_state_t *s,unsigned reason);
void vc_step(vc_state_t *s,const vc_input_t *i);
#endif
