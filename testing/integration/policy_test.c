#include "../../app_program/vehicle_policy.h"
#include "../../app_program/subsystem2/odometry.h"
#include "../../app_program/subsystem4/terrain.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
static vc_input_t healthy(void) {
    return (vc_input_t){.now_ms=100,.line_fresh=1,.steering=1,.front_clear=1,
        .sensor_fresh=1,.odom_valid=1,.terrain_valid=1,.motor_ok=1,.transport_ok=1};
}
static vc_state_t armed(void) {vc_state_t s={0};assert(vc_arm(&s,100,1000,30));return s;}
static void controller(void) {
    vc_input_t i=healthy();vc_state_t s=armed();vc_step(&s,&i);assert(s.command==1 && s.power==30);
    i.line_error=-500;vc_step(&s,&i);assert(s.command==6);
    i.line_error=500;vc_step(&s,&i);assert(s.command==5);
    i.slope=1;vc_step(&s,&i);assert(s.power==20);
    for(unsigned n=0;n<8;++n) {
        s=armed();i=healthy();
        switch(n) {case 0:i.sensor_fresh=0;break;case 1:i.terrain_valid=0;break;
        case 2:i.odom_valid=0;break;case 3:i.motor_ok=0;break;case 4:i.transport_ok=0;break;
        case 5:i.shock=1;break;case 6:i.line_fresh=0;break;case 7:i.steering=0;break;}
        vc_step(&s,&i);assert(s.command==0 && s.power==0 && !s.armed && s.phase==VC_FAULT);
        assert(!vc_arm(&s,i.now_ms,1000,30));
    }
    s=armed();i=healthy();i.now_ms=1100;vc_step(&s,&i);assert(s.fault==VC_TIMEOUT);
    s=(vc_state_t){0};assert(vc_arm(&s,UINT32_MAX-10,100,30));i=healthy();i.now_ms=20;
    vc_step(&s,&i);assert(s.command==1);i.now_ms=100;vc_step(&s,&i);assert(s.fault==VC_TIMEOUT);
    assert(!vc_arm(&s,0,1001,30));assert(!vc_arm(&s,0,1,51));
    s=armed();i=healthy();i.navigation_event=1;i.nav=4;vc_step(&s,&i);
    assert(s.phase==VC_NAV_TURN);i.navigation_event=0;i.yaw_mdeg=179000;vc_step(&s,&i);
    i.yaw_mdeg=181000;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command);
    /* Any close obstacle overrides current steering in the same iteration. */
    s=armed();i=healthy();i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command);
    i.scan_ready=1;i.turn_deg=30;i.approach_mm=200;i.return_mm=100;vc_step(&s,&i);
    assert(s.phase==VC_FAULT && s.fault==VC_UNCALIBRATED);
    s=armed();i=healthy();i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
    i.scan_ready=1;i.bypass_enabled=1;i.turn_deg=30;i.approach_mm=200;i.return_mm=100;
    vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command);
    vc_step(&s,&i);assert(!s.command); /* needs a fresh, forward-facing reading */
    i.front_clear=1;vc_step(&s,&i);assert(s.command==3);
    i.yaw_mdeg=30000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && !s.command);
    i.distance_um=200000;vc_step(&s,&i);assert(s.phase==VC_ALIGN && !s.command);
    i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_PASS && s.look_side && !s.command);
    i.side_clear=0;vc_step(&s,&i);assert(!s.command && s.look_side);
    i.side_ready=1;vc_step(&s,&i);assert(!s.command && !s.look_side);
    i.front_clear=0;vc_step(&s,&i);assert(!s.command);
    i.front_clear=1;vc_step(&s,&i);assert(s.command==1);
    i.distance_um+=50000;vc_step(&s,&i);assert(!s.command && s.look_side);
    i.side_clear=1;vc_step(&s,&i);assert(s.phase==VC_TURN_IN && !s.command);
    i.yaw_mdeg=-90000;vc_step(&s,&i);assert(s.phase==VC_RETURN && !s.command);
    i.distance_um+=100000;vc_step(&s,&i);assert(s.phase==VC_RESTORE && !s.command);
    i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command);
    i.steering=0;vc_step(&s,&i);assert(!s.command);
    i.steering=1;vc_step(&s,&i);assert(s.phase==VC_TRACK && !s.command);
    /* Never resume expired or interrupted open-loop maneuvers. */
    s=armed();s.phase=VC_TURN_OUT;s.entered_ms=100;i=healthy();i.now_ms=6000;
    s.armed_ms=6000;vc_step(&s,&i);assert(s.phase==VC_FAULT);
    puts("INT-CR1-01 controller: priority, expiry, stale data, complete bounded bypass, STOP handoffs PASS");
}
static void odometry(void) {
    odom_config_t c={100,200,100};odom_t s={0};
    odom_step(&s,&c,0,UINT32_MAX-1,UINT32_MAX-1,true);assert(!s.valid);
    odom_step(&s,&c,20,3,3,true);assert(s.valid && odom_delta_um(&s)==10000 && odom_speed_mm_s(&s)==500);
    odom_step(&s,&c,40,3,8,true);assert(s.valid && odom_turn_mdeg(&s)>5700 && odom_turn_mdeg(&s)<5740);
    odom_step(&s,&c,200,4,9,true);assert(!s.valid);
    odom_step(&s,&c,220,4,9,false);assert(!s.valid);
    s=(odom_t){0};odom_step(&s,&c,UINT32_MAX-10,0,0,true);
    odom_step(&s,&c,9,1,1,true);assert(s.valid && odom_speed_mm_s(&s)==100);
    odom_step(&s,&c,29,INT_MAX,INT_MAX,true);assert(!s.valid);
    puts("INT-CR1-02 odometry: wrap, scale, speed, yaw, gaps and discontinuities PASS");
}
static void terrain(void) {
    terrain_t s={0};int32_t a[3]={0,0,1000};
    terrain_step(&s,0,a,true,0,true,false,0,0);assert(!s.tilt_valid && !s.height_valid);
    terrain_step(&s,0,a,true,0,true,true,0,0);assert(s.tilt_valid);
    for(uint32_t ms=20;ms<=80;ms+=20)terrain_step(&s,ms,a,true,0,true,true,0,0);
    assert(s.state==TERRAIN_FLAT);
    int distance=0;uint32_t time=80;
    a[0]=-500;a[2]=866;
    for(unsigned n=0;n<5;++n) {distance+=20000;time+=20;terrain_step(&s,time,a,true,distance,true,true,0,0);}
    assert(s.state==TERRAIN_ASCENT && s.peak_um>49000);
    a[0]=0;a[2]=1000;time+=20;terrain_step(&s,time,a,true,distance,true,true,0,0);assert(s.state==TERRAIN_CREST);
    a[0]=500;a[2]=866;
    for(unsigned n=0;n<5;++n) {distance+=20000;time+=20;terrain_step(&s,time,a,true,distance,true,true,0,0);}
    assert(s.state==TERRAIN_DESCENT && !s.height_valid);
    a[0]=0;a[2]=1000;
    for(unsigned n=0;n<5;++n) {time+=20;terrain_step(&s,time,a,true,distance,true,true,0,0);}
    assert(s.state==TERRAIN_FLAT && s.height_valid && s.height_mm==50 && s.maximum_mm==50);
    time+=20;a[0]=1900;terrain_step(&s,time,a,true,distance,true,true,0,0);
    assert(s.shock && s.state==TERRAIN_SHOCK && s.maximum_mm==50 && !s.height_valid);
    time+=20;a[0]=0;terrain_step(&s,time,a,false,distance,true,true,0,0);assert(!s.tilt_valid);
    puts("INT-CR1-03 terrain: calibration gate, ordered hump, maximum, impact and invalid samples PASS");
}
int main(void) {controller();odometry();terrain();return 0;}
