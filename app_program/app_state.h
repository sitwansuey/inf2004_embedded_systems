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
    uint8_t barcode : 2;
    uint8_t odometry_valid : 1;
} app_state_t;
static inline int32_t app_speed_mm_s(const app_state_t *s) {
    return odom_signed_value(s->speed_bits,12U);
}
ER app_state_init(void);
ER app_state_publish(const app_state_t *p);
ER app_get_state(app_state_t *p);
ER vehicle_start(void);
#endif
