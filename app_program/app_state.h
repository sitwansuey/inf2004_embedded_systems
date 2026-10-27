#ifndef CSC02_APP_STATE_H
#define CSC02_APP_STATE_H
#include "app.h"
#include "vehicle_policy.h"
#include "subsystem4/terrain.h"
typedef struct {
    vc_state_t vehicle;
    terrain_t terrain;
    uint32_t time_ms, sequence;
    int32_t speed_mm_s;
    uint8_t navigation : 3;
    uint8_t barcode : 2;
    uint8_t odometry_valid : 1;
} app_state_t;
ER app_state_init(void);
ER app_state_publish(const app_state_t *p);
ER app_get_state(app_state_t *p);
ER vehicle_start(void);
#endif
