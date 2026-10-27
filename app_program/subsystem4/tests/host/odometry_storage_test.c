/* Production odometry storage: boundary/isolation and unchanged rejection
 * behavior. Synthetic calibration, never a physical accuracy claim. */
#include "../../../subsystem2/odometry.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const odom_config_t scale={100000,100,40}; /* one um per tick */

static void speed_domain(void)
{
    for (int32_t speed=-2000; speed<=2000; ++speed) {
        odom_t s={0};
        odom_step(&s,&scale,0,0,0,true);
        odom_step(&s,&scale,1,(uint32_t)speed,(uint32_t)speed,true);
        assert(s.valid && odom_speed_mm_s(&s)==speed);
        assert(odom_delta_um(&s)==speed && s.distance_um==speed);
        assert(odom_turn_mdeg(&s)==0 && s.time_ms==1);
        assert(s.left==(uint32_t)speed && s.right==(uint32_t)speed);
    }
    for (int32_t direction=-1; direction<=1; direction+=2) {
        odom_t s={0}; int32_t delta=direction*200000;
        odom_step(&s,&scale,UINT32_MAX-99U,0,0,true);
        odom_step(&s,&scale,0,(uint32_t)delta,(uint32_t)delta,true);
        assert(s.valid && odom_delta_um(&s)==delta);
        assert(odom_speed_mm_s(&s)==direction*2000);
        assert(s.distance_um==delta && odom_turn_mdeg(&s)==0);
        odom_step(&s,&scale,1,(uint32_t)(delta+direction*2001),
                  (uint32_t)(delta+direction*2001),true);
        assert(!s.valid && !s.initialized);
        assert(odom_delta_um(&s)==0 && odom_speed_mm_s(&s)==0);
        assert(s.distance_um==delta);
    }
}

static void signed_turns_and_rejections(void)
{
    for (int32_t direction=-1; direction<=1; direction+=2) {
        odom_t s={0};uint32_t left=0,right=0;int32_t expected=0;
        odom_step(&s,&scale,0,0,0,true);
        for (uint32_t ms=1; ms<=300; ++ms) {
            left-=(uint32_t)(direction*2000);
            right+=(uint32_t)(direction*2000);
            expected=(expected+direction*5729)%360000;
            odom_step(&s,&scale,ms,left,right,true);
            assert(s.valid && odom_turn_mdeg(&s)==expected);
            assert(odom_speed_mm_s(&s)==0 && odom_delta_um(&s)==0);
            assert(s.distance_um==0);
        }
    }
    odom_t s={0};
    odom_step(&s,&scale,0,0,0,true);
    s.distance_um=INT32_MAX;
    odom_step(&s,&scale,1,1,1,true);
    assert(!s.valid && !s.initialized && s.distance_um==INT32_MAX);
    odom_step(&s,&scale,2,1,1,true);
    odom_step(&s,&scale,3,UINT32_C(0x80000001),1,true);
    assert(!s.valid && !s.initialized); /* ambiguous half-range count */
    odom_step(&s,&scale,4,1,1,true);
    odom_step(&s,&scale,4,1,1,true);
    assert(!s.valid && s.initialized && odom_speed_mm_s(&s)==0);
    odom_step(&s,&scale,105,1,1,true);
    assert(!s.valid && s.initialized && odom_delta_um(&s)==0);
}

static void calibration_domain(void)
{
    const odom_config_t good[]={{1,20,40},{100000,2000,1000},{100,200,100}};
    const odom_config_t bad[]={{0,20,40},{100001,20,40},{UINT32_MAX,20,40},
        {1,19,40},{1,2001,40},{1,20,39},{1,20,1001}};
    odom_calibration_t packed={0};
    for (unsigned n=0; n<sizeof(good)/sizeof(good[0]); ++n) {
        odom_config_t result={0};
        assert(odom_calibration_pack(&good[n],&packed));
        assert(odom_calibration_unpack(&packed,&result));
        assert(result.ticks_per_rev==good[n].ticks_per_rev);
        assert(result.circumference_mm==good[n].circumference_mm);
        assert(result.track_mm==good[n].track_mm);
    }
    unsigned char saved[sizeof(packed)];memcpy(saved,&packed,sizeof(saved));
    for (unsigned n=0; n<sizeof(bad)/sizeof(bad[0]); ++n) {
        assert(!odom_calibration_pack(&bad[n],&packed));
        assert(memcmp(saved,&packed,sizeof(saved))==0);
    }
    odom_config_t result={123,456,789},before=result;
    packed=(odom_calibration_t){0};
    assert(!odom_calibration_unpack(&packed,&result));
    assert(memcmp(&result,&before,sizeof(result))==0);
    packed.ticks_per_rev=100001;packed.circumference_mm=20;packed.track_mm=40;
    assert(!odom_calibration_unpack(&packed,&result));
    assert(memcmp(&result,&before,sizeof(result))==0);
    assert(!odom_calibration_pack(NULL,&packed));
    assert(!odom_calibration_pack(&scale,NULL));
    assert(!odom_calibration_unpack(NULL,&result));
    assert(!odom_calibration_unpack(&packed,NULL));
}

int main(void)
{
    speed_domain();signed_turns_and_rejections();calibration_domain();
    printf("S4-U32 PASS 4001 speeds, signed delta/yaw bounds and wraps, "
           "discontinuity/overflow, checked calibration; odometry=%zu calibration=%zu bytes\n",
           sizeof(odom_t),sizeof(odom_calibration_t));
    return 0;
}
