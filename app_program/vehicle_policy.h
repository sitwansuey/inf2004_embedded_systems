#ifndef CSC02_VEHICLE_POLICY_H
#define CSC02_VEHICLE_POLICY_H
#include <stdint.h>
#include <stdbool.h>
#include "shared/avoidance_config.h"
enum { VC_DISARMED,VC_TRACK,VC_ACQUIRE,VC_TURN_OUT,VC_DIAGONAL,VC_ALIGN,
       VC_PASS,VC_TURN_IN,VC_RETURN,VC_RESTORE,VC_REACQUIRE,VC_NAV_TURN,VC_FAULT,
       VC_BACKUP /* appended: reverse after ACQUIRE, before TURN_OUT */ };
enum { VC_OK,VC_SENSOR,VC_LINE,VC_SHOCK,VC_TRANSPORT,VC_TIMEOUT,VC_UNCALIBRATED,VC_MOTOR,VC_LIFTED };
#define VC_TRACK_HANDOFF_MS 100U
/* Line data missing (late sampler) while tracking or searching: hold still,
 * a VC_LINE (line data) fault only once the gap lasts this long. */
#define VC_LINE_GAP_MS 250U
/* No fresh sonar sample (late S5 task): hold still; VC_SENSOR (sonar) only
 * once the gap lasts this long. */
#define VC_SONAR_GAP_MS 1000U
/* No valid odometry (one late controller period, two encoder snapshots more
 * than 60 ms apart, or a failed encoder copy leaves it invalid for two
 * ticks): hold still; VC_SENSOR (odometry) only once the gap lasts this
 * long. */
#define VC_ODOM_GAP_MS 200U
/* Spot search (VC_REACQUIRE): at most this long, then VC_LINE (not found),
 * also when the sonar or late line data holds the search. */
#define VC_SEARCH_MS 4000U
/* S3 TRACK lease, renewed every tick. A busy S3 status lock (E_TMOUT) is
 * retried next tick; it stops with VC_TRANSPORT only once the lease would
 * have run out since the last good control or renew. */
#define VC_S3_LEASE_MS 100U
/* S3 latched STALE/SENSOR with no event (a sampler hiccup) stops barcode
 * capture until TRACK is sent again: vehicle.c re-sends it at most every
 * VC_S3_REARM_MS while tracking on fresh samples. A latch still returning
 * VC_S3_LATCH_MS after it was first seen is VC_LINE (cause event). S3 still
 * holding a decoded marker (awaiting the controller, no error) with no turn
 * pending in the VC for VC_S3_REARM_MS also gets TRACK again (marker clear). */
#define VC_S3_REARM_MS 100U
#define VC_S3_LATCH_MS 1000U
typedef struct {
    uint32_t now_ms, side_sample_ms;
    int32_t distance_um, yaw_mdeg;
    int16_t turn_deg;
    uint16_t approach_mm, return_mm, obstacle_width_mm;
    uint8_t line_black_mask; /* bit 0 right, bit 1 left */
    uint8_t nav;
    /* A decoded turn waits for the next junction, then drives on nav_offset_mm
     * (line sensors to wheels) before pivoting; with no junction it turns after
     * nav_search_mm. 0/0 turns at once (host tests). */
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
    /* Line lost in TRACK: this much forward travel with neither straddle
     * sensor on black spot-scans for the line (both white is also "centred").
     * 0: off, and the distance starts again (e.g. during a barcode re-read). */
    uint16_t line_lost_mm;
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
    uint8_t line_detected : 1;
    uint8_t nav_hold : 1; /* defer a decoded turn while a barcode re-read repositions */
    uint8_t backup_ready : 1; /* new APPROACH/0 sample after BACKUP entry */
    uint8_t front_pending : 1; /* pan has no clear forward sample yet */
    uint8_t sensor_acquiring : 1; /* S5 still in an acquisition-stage state (its motion guard is armed) */
    uint8_t junction : 1; /* crossing line: both line sensors and GP28 on black */
    uint8_t line_touch : 1; /* fresh sample with a line sensor on black */
    uint8_t spot_scan_all : 1; /* left/right nav turns also spot-scan (a U-turn always does) unless ended on a touch */
} vc_input_t;
typedef struct {
    uint32_t entered_ms, armed_ms, lease_ms;
    uint32_t track_requested_ms;
    uint32_t line_found_ms;
    uint32_t line_gap_ms; /* first tick of the current line-data gap */
    uint32_t sonar_gap_ms; /* first tick of the current sonar gap */
    uint32_t front_wait_ms; /* return pan waiting for a forward sample */
    uint32_t odom_gap_ms; /* first tick of the current odometry gap */
    int32_t origin_um, origin_yaw, turn_progress;
    int32_t avoid_yaw; /* heading when the obstacle stopped tracking: the bypass restores it */
    int32_t restore_mdeg; /* VC_RESTORE: turn back to avoid_yaw (positive left) */
    int32_t nav_origin_um; /* where the pending turn's junction search / offset began */
    int32_t uturn_end_um; /* odometry where the last U-turn finished */
    int32_t line_seen_um; /* TRACK: odometry of the last straddle black (or the furthest back since) */
    uint16_t approach_mm, return_mm;
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
    uint8_t nav_touched : 1; /* VC_NAV_TURN: a straddle sensor touched the new line (past min); bypass return: the lead sensor touched the line */
    uint8_t fault_cause : 2; /* VC_SENSOR: 0 other, 1 sonar, 2 tilt, 3 odometry; VC_LINE: VC_LINE_CAUSE_* */
    uint8_t nav_search : 1; /* pending turn actionable in TRACK: looking for its junction */
    uint8_t nav_offset : 1; /* junction seen: straight on for nav_offset_mm, then turn */
    uint8_t spot_scan : 1; /* REACQUIRE after a turn ended without a line touch: only a sensor on black ends it */
    uint8_t uturn_guard : 1; /* uturn_end_um is valid: repeat U-turns are ignored nearby */
    uint8_t direct_return : 1; /* side clear at the end of the diagonal: TURN_IN turns 2x turn_deg straight back */
    uint8_t clear_run : 1; /* side look clear: AVOID_CLEAR_RUN_MM straight on before TURN_IN */
    uint8_t line_gap : 1; /* line data missing since line_gap_ms */
    uint8_t sonar_gap : 1; /* no fresh sonar sample since sonar_gap_ms */
    uint8_t front_wait : 1;
    uint8_t odom_gap : 1; /* no valid odometry since odom_gap_ms */
    uint8_t line_found : 1; /* bypass return found the line: RESTORE the heading, then TRACK */
    uint8_t line_seen : 1; /* line_seen_um is set (cleared on every phase entry) */
} vc_state_t;
enum { VC_CAUSE_OTHER,VC_CAUSE_SONAR,VC_CAUSE_TILT,VC_CAUSE_ODOM };
/* VC_LINE causes (same 2-bit fault_cause field). */
enum { VC_LINE_CAUSE_OTHER,VC_LINE_CAUSE_DATA,VC_LINE_CAUSE_EVENT,VC_LINE_CAUSE_NOT_FOUND };
void vc_track_requested(vc_state_t *s, uint32_t now);
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power);
/* ARM again during a leased ARM run: a new lease from now under vc_arm's
 * rules; the phase is kept. false (nothing changes) when disarmed, in a
 * START run (lease 0), in FAULT, for a bad lease, or for a power other than
 * the run's maximum_power. */
bool vc_renew(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power);
bool vc_start(vc_state_t *s, uint32_t now, unsigned power);
void vc_stop(vc_state_t *s,unsigned reason);
void vc_step(vc_state_t *s,const vc_input_t *i);
/* Avoidance arc smoothing (vehicle.c, forward TURN_OUT..RESTORE): power rises
 * from 0 by AVOID_POWER per AVOID_RAMP_POWER_MS (drops are immediate) and the
 * steering moves at most 400 per AVOID_STEER_RAMP_MS. Each step's allowance
 * comes from the ms elapsed since the ramp started (the VC's clock), so the
 * ramps take real ms at any VC tick (it asks for 10 ms, really runs at 20). */
typedef struct {uint32_t start_ms,elapsed_ms;int32_t power_m,steer;} vc_avoid_ramp_t;
/* Outside an arc: power and steering back to 0; the ramp starts again now. */
void vc_avoid_ramp_reset(vc_avoid_ramp_t *r,uint32_t now);
/* One arc tick toward power (%) and steering: r->power_m (milli-%), r->steer. */
void vc_avoid_ramp_step(vc_avoid_ramp_t *r,uint32_t now,unsigned power,int32_t steering);
#endif
