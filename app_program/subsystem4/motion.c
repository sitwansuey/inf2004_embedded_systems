#include "motion.h"
#include "terrain.h"
#include "packed_bits.h"
#include <stdlib.h>

static void event(s4_motion_t *s,unsigned bit) {
    s->events=(uint8_t)((unsigned)s->events|bit)&63U;
    ++s->revision;
}
static unsigned motion_event(unsigned motion) {
    return motion==S4_MOTION_STATIONARY?S4_EVENT_STATIONARY:
        motion==S4_MOTION_ACCELERATING?S4_EVENT_ACCELERATING:
        motion==S4_MOTION_TURNING?S4_EVENT_TURNING:0U;
}
void s4_motion_step(s4_motion_t *s,uint32_t now,const odom_t *o,
                    uint8_t terrain,bool height_valid,uint8_t impact_sequence) {
    if(!s || !o)return;
    if(impact_sequence!=s->impact_sequence) {
        s->impact_sequence=impact_sequence&15U;event(s,S4_EVENT_IMPACT);
    }
    if(terrain!=s->previous_terrain) {
        if(terrain==TERRAIN_ASCENT)event(s,S4_EVENT_CLIMBING);
        if(terrain==TERRAIN_DESCENT)event(s,S4_EVENT_DESCENDING);
        s->previous_terrain=terrain&7U;
    }
    if(height_valid && !s->height_seen) {
        if(s->humps<255U)++s->humps;
        ++s->revision;
    }
    s->height_seen=height_valid;
    if(!o->valid || now-o->time_ms>60U) {
        s->motion=S4_MOTION_UNKNOWN;s->initialized=0;s->rate_valid=0;return;
    }
    int speed=odom_speed_mm_s(o);
    if(!s->initialized) {
        s->sampled_ms=o->time_ms;s->yaw_bits=o->turn_bits;
        s->acceleration_speed_bits=o->speed_bits;
        s->acceleration_ms=o->time_ms;s->candidate_ms=o->time_ms;
        s->candidate=S4_MOTION_UNKNOWN;s->motion=S4_MOTION_UNKNOWN;
        s->rate_code=6000;s->rate_valid=0;s->initialized=1;return;
    }
    uint32_t dt=o->time_ms-s->sampled_ms;
    if(dt==0U)return; /* duplicates cannot advance confirmation/filter history */
    if(dt>60U) {s->initialized=0;s->motion=S4_MOTION_UNKNOWN;s->rate_valid=0;return;}
    int yaw=odom_turn_mdeg(o)-odom_signed_value(s->yaw_bits,20U);
    if(yaw>180000)yaw-=360000;
    if(yaw< -180000)yaw+=360000;
    /* At <=2m/s per wheel and >=40mm track, |yaw rate| <5730 dps.
     * <=30ms therefore guarantees an unambiguous modular angle difference. */
    bool rate_ok=dt<=30U;
    int rate=rate_ok?yaw/(int)dt:0;
    rate_ok=rate_ok && rate>=-6000 && rate<=6000;
    if(rate_ok) {
        int old=s->rate_valid?(int)s->rate_code-6000:rate;
        rate=(old*3+rate)/4;
        s->rate_code=(uint32_t)(rate+6000)&16383U;
    }
    s->rate_valid=rate_ok;
    uint32_t interval=o->time_ms-s->acceleration_ms;
    int acceleration=0;
    if(interval>=100U && interval<=200U) {
        int previous=odom_signed_value(s->acceleration_speed_bits,12U);
        acceleration=(abs(speed)-abs(previous))*1000/(int)interval;
    }
    unsigned wanted=S4_MOTION_MOVING;
    int stop_limit=s->motion==S4_MOTION_STATIONARY?15:5;
    int turn_limit=s->motion==S4_MOTION_TURNING?5:S4_MOTION_TURN_DPS;
    if(abs(speed)<=stop_limit && !rate_ok)wanted=S4_MOTION_UNKNOWN;
    else if(abs(speed)<=stop_limit && abs(rate)<turn_limit)wanted=S4_MOTION_STATIONARY;
    else if(rate_ok && abs(rate)>=turn_limit)wanted=S4_MOTION_TURNING;
    else if(acceleration>=S4_MOTION_ACCEL_MM_S2)wanted=S4_MOTION_ACCELERATING;
    else if(acceleration<=-S4_MOTION_ACCEL_MM_S2)wanted=S4_MOTION_DECELERATING;
    /* Acceleration is measured over a 100ms window; its own window provides
     * confirmation. Other classes require 100ms stable evidence. */
    if(wanted!=s->candidate) {s->candidate=wanted&7U;s->candidate_ms=o->time_ms;}
    if(wanted==S4_MOTION_ACCELERATING || wanted==S4_MOTION_DECELERATING ||
       o->time_ms-s->candidate_ms>=S4_MOTION_CONFIRM_MS) {
        if(s->motion!=wanted) {s->motion=wanted&7U;event(s,motion_event(wanted));}
    }
    if(interval>=100U) {
        s->acceleration_ms=o->time_ms;s->acceleration_speed_bits=o->speed_bits;
    }
    s->sampled_ms=o->time_ms;s->yaw_bits=o->turn_bits;
}
void s4_motion_snapshot(const s4_motion_t *s,uint32_t now,s4_motion_result_t *out) {
    bool current=s->initialized && now-s->sampled_ms<=60U;
    *out=(s4_motion_result_t){.sampled_ms=now,
        .turn_rate_dps=current && s->rate_valid?(int)s->rate_code-6000:0,
        .motion=current?(uint8_t)s->motion:S4_MOTION_UNKNOWN,
        .flags=(uint8_t)((current?S4_MOTION_ODOMETRY:0U)|
            (current && s->rate_valid?S4_MOTION_RATE:0U)),
        .events=s->events,.revision=s->revision,.humps=s->humps};
}
static bool valid(const s4_motion_result_t *v) {
    return v && v->turn_rate_dps>=-6000 && v->turn_rate_dps<=6000 &&
        v->motion<=S4_MOTION_TURNING && v->flags<=3 && v->events<=63 &&
        (!(v->flags&S4_MOTION_RATE) || (v->flags&S4_MOTION_ODOMETRY)) &&
        ((v->flags&S4_MOTION_RATE) || v->turn_rate_dps==0) &&
        ((v->flags&S4_MOTION_ODOMETRY) || v->motion==S4_MOTION_UNKNOWN);
}
bool s4_motion_encode(const s4_motion_result_t *v,uint8_t *out,size_t capacity) {
    if(!out || capacity<S4_MOTION_BYTES || !valid(v))return false;
    uint8_t bytes[S4_MOTION_BYTES]={0};
    s4_bits_put(bytes,0,32,v->sampled_ms);s4_bits_put(bytes,32,14,(uint32_t)(v->turn_rate_dps+6000));
    s4_bits_put(bytes,46,3,v->motion);s4_bits_put(bytes,49,2,v->flags);
    s4_bits_put(bytes,51,6,v->events);s4_bits_put(bytes,57,8,v->revision);s4_bits_put(bytes,65,8,v->humps);
    for(unsigned n=0;n<S4_MOTION_BYTES;++n)out[n]=bytes[n];
    return true;
}
bool s4_motion_decode(const uint8_t *bytes,size_t length,s4_motion_result_t *out) {
    if(!bytes || !out || length!=S4_MOTION_BYTES || (bytes[9]&254U))return false;
    s4_motion_result_t v={.sampled_ms=(uint32_t)s4_bits_get(bytes,0,32),
        .turn_rate_dps=(int32_t)s4_bits_get(bytes,32,14)-6000,
        .motion=(uint8_t)s4_bits_get(bytes,46,3),.flags=(uint8_t)s4_bits_get(bytes,49,2),
        .events=(uint8_t)s4_bits_get(bytes,51,6),.revision=(uint8_t)s4_bits_get(bytes,57,8),
        .humps=(uint8_t)s4_bits_get(bytes,65,8)};
    if(!valid(&v))return false;
    *out=v;return true;
}
