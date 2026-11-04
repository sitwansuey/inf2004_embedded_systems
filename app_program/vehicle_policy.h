#ifndef CSC02_VEHICLE_POLICY_H
#define CSC02_VEHICLE_POLICY_H
#include <stdint.h>
#include <stdbool.h>
enum { VC_DISARMED,VC_TRACK,VC_ACQUIRE,VC_TURN_OUT,VC_DIAGONAL,VC_ALIGN,
       VC_PASS,VC_TURN_IN,VC_RETURN,VC_RESTORE,VC_REACQUIRE,VC_NAV_TURN,VC_FAULT };
enum { VC_OK,VC_SENSOR,VC_LINE,VC_SHOCK,VC_TRANSPORT,VC_TIMEOUT,VC_UNCALIBRATED,VC_MOTOR };
#define VC_TRACK_HANDOFF_MS 100U
typedef struct {
    uint32_t now_ms, scan_id;
    int32_t distance_um, yaw_mdeg;
    int16_t line_error, turn_deg;
    uint16_t approach_mm, return_mm;
    uint8_t sensor_state;
    uint8_t line_state;
    uint8_t line_black_mask; /* bit 0 right, bit 1 left */
    uint8_t nav;
    uint16_t bypass_steering; /* track width / (2 * planned arc radius), permille */
    uint8_t line_fresh : 1;
    uint8_t steering : 1;
    uint8_t front_clear : 1;
    uint8_t obstacle : 1;
    uint8_t scan_ready : 1;
    uint8_t sensor_fresh : 1;
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
} vc_input_t;
typedef struct {
    uint32_t entered_ms, armed_ms, lease_ms, scan_id;
    uint32_t track_requested_ms;
    uint32_t line_found_ms;
    uint32_t bypass_drive_ms, last_step_ms;
    int32_t origin_um, origin_yaw, turn_progress;
    uint16_t approach_mm, return_mm;
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
} vc_state_t;
void vc_track_requested(vc_state_t *s, uint32_t now);
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power);
bool vc_start(vc_state_t *s, uint32_t now, unsigned power);
void vc_stop(vc_state_t *s,unsigned reason);
void vc_step(vc_state_t *s,const vc_input_t *i);
#endif
