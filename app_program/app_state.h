#ifndef CSC02_APP_STATE_H
#define CSC02_APP_STATE_H
#include "app.h"
#include "vehicle_policy.h"
#include "subsystem4/result_data.h"
#include "subsystem2/odometry.h"
typedef struct {
    vc_state_t vehicle;
    uint8_t s4_result[S4_RESULT_SIZE];
    uint32_t time_ms, sequence;
    uint16_t speed_bits : 12; /* odometry's -2000..2000 mm/s, same low bits */
    uint8_t navigation : 3;
    uint8_t barcode : 5; /* 0 none, 1..26 = last decoded A..Z */
    uint8_t odometry_valid : 1;
    /* S4 shocks seen (rising edges, wraps). A shock lasts one 20 ms
     * accelerometer sample, so telemetry sends a frame for each new count. */
    uint8_t shock_count;
    /* Straddle line follower this tick (straddle builds; telemetry line
     * fields): 0 straight, 1/2 steer right/left, 3/4 pivot right/left. */
    uint8_t line_follow : 3;
    uint8_t line_mask : 2; /* straddle sensors on black: 1 right, 2 left */
    int16_t line_steer; /* S2 steering units, positive = right */
} app_state_t;
static inline int32_t app_speed_mm_s(const app_state_t *s) {
    return odom_signed_value(s->speed_bits,12U);
}
ER app_state_init(void);
ER app_state_publish(const app_state_t *p);
ER app_get_state(app_state_t *p);
ER vehicle_start(void);
#endif
