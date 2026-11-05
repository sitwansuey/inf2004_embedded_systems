#ifndef CSC02_WHEEL_CALIBRATION_POLICY_H
#define CSC02_WHEEL_CALIBRATION_POLICY_H
#include <stdint.h>
#include <stdbool.h>
#define WHEEL_CAL_MAX_LEASE_MS 3000U
typedef struct {uint32_t started_ms,lease_ms;unsigned power;bool running;} wheel_cal_t;
static inline void wheel_cal_stop(wheel_cal_t *s) {s->running=false;s->power=0;}
static inline bool wheel_cal_arm(wheel_cal_t *s,uint32_t now,unsigned power,unsigned lease,bool ready) {
    if(!s || s->running || !ready || !power || power>30 || !lease || lease>WHEEL_CAL_MAX_LEASE_MS)return false;
    s->started_ms=now;s->lease_ms=lease;s->power=power;s->running=true;return true;
}
static inline unsigned wheel_cal_step(wheel_cal_t *s,uint32_t now,bool ready) {
    if(!s || !s->running)return 0;
    if(!ready || now-s->started_ms>=s->lease_ms){wheel_cal_stop(s);return 0;}
    return s->power;
}
#endif
