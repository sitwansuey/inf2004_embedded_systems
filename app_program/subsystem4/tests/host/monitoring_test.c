#include "../../motion.h"
#include "../../terrain.h"
#include "../../sensor_data.h"
#include "../../accel_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void sample(s4_motion_t *s,uint32_t t,int speed,int yaw) {
    odom_t o={0};o.valid=1;o.time_ms=t;
    o.speed_bits=(uint32_t)speed&4095U;o.turn_bits=(uint32_t)yaw&1048575U;
    s4_motion_step(s,t,&o,TERRAIN_FLAT,false,0);
}
static void motion(void) {
    s4_motion_t s={0};s4_motion_result_t v;
    for(uint32_t t=0;t<=140;t+=20)sample(&s,t,0,0);
    assert(s.motion==S4_MOTION_STATIONARY && (s.events&S4_EVENT_STATIONARY));
    for(uint32_t t=160;t<=400;t+=20)sample(&s,t,(int)t-140,0);
    assert(s.motion==S4_MOTION_ACCELERATING && (s.events&S4_EVENT_ACCELERATING));
    for(uint32_t t=420;t<=700;t+=20)sample(&s,t,260,0);
    assert(s.motion==S4_MOTION_MOVING);
    for(uint32_t t=720;t<=960;t+=20)sample(&s,t,260,(int)(t-700U)*30);
    assert(s.motion==S4_MOTION_TURNING && (s.events&S4_EVENT_TURNING));
    s4_motion_snapshot(&s,960,&v);
    assert((v.flags&S4_MOTION_RATE) && v.turn_rate_dps>=25 && v.turn_rate_dps<=30);
    sample(&s,960,260,7800);assert(s.motion==S4_MOTION_TURNING);
    odom_t invalid={0};s4_motion_step(&s,980,&invalid,TERRAIN_ASCENT,false,1);
    assert(s4_motion_impact(&s) && s.motion==S4_MOTION_UNKNOWN && (s.events&S4_EVENT_CLIMBING));
    s4_motion_step(&s,1000,&invalid,TERRAIN_DESCENT,true,1);
    assert(s.humps==1 && (s.events&S4_EVENT_DESCENDING));
    s4_motion_step(&s,1020,&invalid,TERRAIN_FLAT,true,1);assert(s.humps==1);
    s4_motion_snapshot(&s,1020,&v);assert(v.flags==0 && (v.events&S4_EVENT_IMPACT));
    uint8_t bytes[S4_MOTION_BYTES+1];memset(bytes,0xa5,sizeof(bytes));
    assert(s4_motion_encode(&v,bytes,sizeof(bytes)) && bytes[10]==0xa5);
    s4_motion_result_t decoded={0};assert(s4_motion_decode(bytes,10,&decoded));
    assert(decoded.flags==v.flags && decoded.events==v.events && decoded.humps==1);
    for(unsigned bit=73;bit<80;++bit) {
        bytes[9]|=(uint8_t)(1U<<(bit-72U));assert(!s4_motion_decode(bytes,10,&decoded));bytes[9]&=1U;
    }
    for(size_t n=0;n<=12;++n)if(n!=10)assert(!s4_motion_decode(bytes,n,&decoded));
    v.turn_rate_dps=6001;assert(!s4_motion_encode(&v,bytes,sizeof(bytes)));
    v.turn_rate_dps=0;v.motion=6;assert(!s4_motion_encode(&v,bytes,sizeof(bytes)));
    s=(s4_motion_t){0};sample(&s,UINT32_MAX-39U,100,359000);
    sample(&s,UINT32_MAX-19U,100,359600);sample(&s,0,100,200);
    s4_motion_snapshot(&s,0,&v);assert(v.turn_rate_dps==30 && (v.flags&S4_MOTION_RATE));
    sample(&s,40,100,1400);s4_motion_snapshot(&s,40,&v);assert(!(v.flags&S4_MOTION_RATE));
    s=(s4_motion_t){0};
    for(uint32_t t=0;t<=140;t+=20)sample(&s,t,300,0);
    for(uint32_t t=160;t<=380;t+=20)sample(&s,t,440-(int)t,0);
    assert(s.motion==S4_MOTION_DECELERATING);
    puts("S4-U37 PASS motion confirmation, acceleration window, turn-rate filtering/wrap/gaps, sticky events, impact latch and compact codec");
}
static void impacts(void) {
    s4_sensor_cache_t cache;s4_sensor_status_t view;
    s4_data_reset(&cache);s4_data_init_result(&cache,LSM303_OK);
    lsm303_frame_t f={0};f.accel.fresh=f.accel.valid=true;
    f.accel.status=LSM303_DATA_OK;f.accel.raw[0]=1900;f.accel.raw[2]=1000;
    for(unsigned n=0;n<17;++n) {
        f.accel.read_time_us=(uint64_t)n*20000U;s4_data_update(&cache,&f,LSM303_OK);
    }
    f.accel.raw[0]=0;f.accel.read_time_us=340000;
    s4_data_update(&cache,&f,LSM303_OK);s4_data_snapshot(&cache,340000,&view);
    assert(view.impact_sequence==1 && view.latest.accel.raw[0]==0 && sizeof(cache)==63);
    f.accel.raw[0]=1900;f.accel.overrun=true;s4_data_update(&cache,&f,LSM303_OK);
    s4_data_snapshot(&cache,340000,&view);assert(view.impact_sequence==1);
    f.accel.overrun=false;f.accel.valid=false;f.accel.status=LSM303_DATA_CLIPPED;
    f.accel.raw[0]=2047;s4_data_update(&cache,&f,LSM303_OK);
    s4_data_snapshot(&cache,340000,&view);assert(view.impact_sequence==2 && !view.accel_usable);
    s4_data_init_result(&cache,LSM303_OK);s4_data_snapshot(&cache,340000,&view);
    assert(view.impact_sequence==2);
    puts("S4-U38 PASS acquisition-rate high-g revision survives quiet samples/reinit; overrun rejected; cache remains 63 bytes");
}
static void geometry(void) {
    const int32_t flat[3]={0,0,1000}, left[3]={0,174,985},right[3]={0,-174,985};
    terrain_geometry_t g={.track_mm=100,.one_sided=1,.configured=1};
    for(unsigned side=0;side<2;++side) {
        terrain_t s={0};uint32_t t=0;int32_t distance=0;
        for(unsigned n=0;n<6;++n) {t+=20;terrain_step_model(&s,t,flat,true,distance,true,true,0,0,&g);}
        assert(s.state==TERRAIN_FLAT);
        for(unsigned n=0;n<4;++n) {t+=20;distance+=10000;terrain_step_model(&s,t,side?right:left,true,distance,true,true,0,0,&g);}
        assert(s.peak_um>=8600 && s.peak_um<=8800);
        for(unsigned n=0;n<5;++n) {t+=20;distance+=10000;terrain_step_model(&s,t,flat,true,distance,true,true,0,0,&g);}
        assert(s.height_valid && s.height_mm==9 && s.maximum_mm==9);
        terrain_step_model(&s,t+20,left,true,distance,true,true,0,0,&g);
        terrain_step_model(&s,t+100,left,true,distance,true,true,0,0,&g);
        assert(!s.height_valid && s.maximum_mm==9);
    }
    terrain_t missing={0};g.configured=0;
    terrain_step_model(&missing,20,left,true,0,true,true,0,0,&g);
    assert(missing.tilt_valid && !missing.height_valid && missing.state==TERRAIN_INVALID);
    g.configured=1;g.left_mm=20;
    for(unsigned side=0;side<2;++side) {
        terrain_t s={0};uint32_t t=0;
        for(unsigned n=0;n<6;++n) {t+=20;terrain_step_model(&s,t,flat,true,0,true,true,0,0,&g);}
        t+=20;terrain_step_model(&s,t,side?right:left,true,0,true,true,0,0,&g);
        assert(side?(s.peak_um>=5100 && s.peak_um<=5300):(s.peak_um>=12000 && s.peak_um<=12300));
        for(unsigned n=0;n<6;++n) {t+=20;terrain_step_model(&s,t,flat,true,0,true,true,0,0,&g);}
        assert(!s.height_valid); /* Hand tilting without travel is not a hump. */
    }
    puts("S4-U39 PASS analytic one-sided left/right IMU rise, complete traversal, gap recovery, retained maximum and missing geometry rejection");
}
static void calibration(void) {
    s4_accel_calibration_t c={.bias_x=100,.bias_y=-100,.bias_z=50,
        .gain_x=500,.gain_y=500,.gain_z=500,.active=1};
    const int32_t raw[3]={100,-100,1050};int32_t body[3]={7,8,9};
    assert(s4_calibrated_body(raw,1,&c,body));
    assert(body[0]==0 && body[1]==0 && body[2]==1000);
    c.gain_z=750;assert(s4_calibrated_body(raw,0,&c,body) && body[2]==1250);
    c.active=0;assert(s4_calibrated_body(raw,0,&c,body) && body[2]==1050);
    c.bias_x=511;int32_t old[3];memcpy(old,body,sizeof(old));
    assert(!s4_calibrated_body(raw,0,&c,body) && !memcmp(body,old,sizeof(old)));
    puts("S4-U41 PASS independent bias/gain correction, body mapping, inactive identity and unchanged invalid output");
}
int main(int argc,char **argv) {
    if(argc==2) {
        uint8_t bytes[S4_MOTION_BYTES];
        if(strlen(argv[1])!=2U*sizeof(bytes))return 2;
        for(unsigned n=0;n<sizeof(bytes);++n) {
            unsigned value;if(sscanf(argv[1]+n*2U,"%2x",&value)!=1)return 2;
            bytes[n]=(uint8_t)value;
        }
        s4_motion_result_t v;
        if(!s4_motion_decode(bytes,sizeof(bytes),&v))return 2;
        printf("%u %d %u %u %u %u %u\n",v.sampled_ms,(int)v.turn_rate_dps,
            v.motion,v.flags,v.events,v.revision,v.humps);return 0;
    }
    motion();impacts();geometry();calibration();return 0;
}
