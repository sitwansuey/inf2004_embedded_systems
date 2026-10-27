#include "odometry.h"
#include <limits.h>
#include <stdlib.h>
#include "../shared/edr.h"
bool odom_config_valid(const odom_config_t *c) {
    return c && c->ticks_per_rev>0 && c->ticks_per_rev<=100000 &&
           c->circumference_mm>=20 && c->circumference_mm<=2000 &&
           c->track_mm>=40 && c->track_mm<=1000;
}
bool odom_calibration_pack(const odom_config_t *c, odom_calibration_t *out) {
    if (!out || !odom_config_valid(c)) return false;
    odom_calibration_t value={0};
    value.ticks_per_rev=c->ticks_per_rev & UINT32_C(0x1ffff);
    value.circumference_mm=c->circumference_mm & UINT32_C(0x7ff);
    value.track_mm=c->track_mm & UINT32_C(0x3ff);
    *out=value; return true;
}
bool odom_calibration_unpack(const odom_calibration_t *c, odom_config_t *out) {
    if (!c || !out) return false;
    odom_config_t value={c->ticks_per_rev,c->circumference_mm,c->track_mm};
    if (!odom_config_valid(&value)) return false;
    *out=value; return true;
}
void odom_step(odom_t *s,const odom_config_t *c,uint32_t now,
               uint32_t left,uint32_t right,bool usable) {
    if (!s) return;
    s->valid=0; odom_clear_motion(s);
    if (!odom_config_valid(c) || !usable) { s->initialized=0; return; }
    uint32_t dt=now-s->time_ms;
    if (!s->initialized || dt>100) {
        s->time_ms=now; s->left=left; s->right=right; s->initialized=1; return;
    }
    if (dt==0) return;
    int32_t dl, dr;
    if (!edr_count_delta(left,s->left,&dl) || !edr_count_delta(right,s->right,&dr)) {
        s->initialized=0; return;
    }
    s->time_ms=now; s->left=left; s->right=right;
    int64_t lu=(int64_t)dl*c->circumference_mm*1000/c->ticks_per_rev;
    int64_t ru=(int64_t)dr*c->circumference_mm*1000/c->ticks_per_rev;
    /* More than 2 m/s is a discontinuity, not a calibrated observation. */
    if (llabs(lu)>2000LL*dt || llabs(ru)>2000LL*dt) { s->initialized=0; return; }
    int64_t du=(lu+ru)/2, total=(int64_t)s->distance_um+du;
    int64_t yaw=(ru-lu)*57296/(1000LL*c->track_mm); /* rad -> millidegrees */
    if (total<INT32_MIN || total>INT32_MAX) { s->initialized=0; return; }
    /* Per-wheel limits prove |du|<=200000 and |du/dt|<=2000.
     * Keep arithmetic wide until after those bounds and total checks. */
    s->distance_um=(int32_t)total;
    s->delta_bits=(uint32_t)du & UINT32_C(0x7ffff);
    s->speed_bits=(uint32_t)(du/dt) & UINT32_C(0xfff);
    int64_t turn=((int64_t)odom_turn_mdeg(s)+yaw)%360000;
    s->turn_bits=(uint32_t)turn & UINT32_C(0xfffff);
    s->valid=1;
}
