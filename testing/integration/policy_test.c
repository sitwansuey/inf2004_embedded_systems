#include "../../app_program/vehicle_policy.h"
#include "../../app_program/subsystem2/odometry.h"
#include "../../app_program/subsystem4/terrain.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
static vc_input_t healthy(void) {
    return (vc_input_t){.now_ms=100,.line_fresh=1,.steering=1,.front_clear=1,
        .sensor_fresh=1,.odom_valid=1,.terrain_valid=1,.motor_ok=1,.transport_ok=1,.width_valid=1,.obstacle_width_mm=200,.bypass_steering=376,
        .nav_offset_mm=90,.nav_search_mm=600};
}
static vc_state_t armed(void) {vc_state_t s={0};assert(vc_arm(&s,100,1000,30));return s;}
static void controller(void) {
    vc_input_t i=healthy();vc_state_t s=armed();vc_step(&s,&i);assert(s.command==1 && s.power==30);
    i.line_error=-500;vc_step(&s,&i);assert(s.command==1); /* Wheel owner applies differential steering. */
    i.line_error=500;vc_step(&s,&i);assert(s.command==1);
    i.slope=1;vc_step(&s,&i);assert(s.power==20);
    for(unsigned n=0;n<8;++n) {
        s=armed();i=healthy();
        switch(n) {case 0:i.sensor_fresh=0;break;case 1:i.terrain_valid=0;break;
        case 2:i.odom_valid=0;break;case 3:i.motor_ok=0;break;case 4:i.transport_ok=0;break;
        case 5:i.shock=1;break;case 6:i.line_fresh=0;break;case 7:i.steering=0;break;}
        vc_step(&s,&i);assert(s.command==0 && s.power==0 && !s.armed && s.phase==VC_FAULT);
        /* sonar 1, tilt 2, odometry 3; non-sensor faults carry no cause */
        assert(s.fault_cause==(n<3?n+1U:0U) && s.fault==(n<3?VC_SENSOR:s.fault));
        i.now_ms+=20;vc_step(&s,&i);assert(s.fault_cause==(n<3?n+1U:0U)); /* latched */
        assert(!vc_arm(&s,i.now_ms,1000,30));
    }
    /* Check order: stale sonar before tilt before odometry. */
    s=armed();i=healthy();i.sensor_fresh=i.terrain_valid=i.odom_valid=0;vc_step(&s,&i);
    assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR);
    s=armed();i=healthy();i.terrain_valid=i.odom_valid=0;vc_step(&s,&i);
    assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_TILT);
    /* STOP or RESET clears it; a new start never inherits a stale cause. */
    vc_stop(&s,VC_OK);assert(s.phase==VC_DISARMED && !s.fault_cause);
    s.fault_cause=VC_CAUSE_ODOM;assert(vc_start(&s,200,30) && !s.fault_cause);
    s.fault_cause=VC_CAUSE_TILT;s.armed=0;assert(vc_arm(&s,300,1000,30) && !s.fault_cause);
    s=armed();i=healthy();i.now_ms=1100;vc_step(&s,&i);assert(s.fault==VC_TIMEOUT);
    s=(vc_state_t){0};assert(vc_arm(&s,UINT32_MAX-10,100,30));i=healthy();i.now_ms=20;
    vc_step(&s,&i);assert(s.command==1);i.now_ms=100;vc_step(&s,&i);assert(s.fault==VC_TIMEOUT);
    assert(!vc_arm(&s,0,1001,30));assert(!vc_arm(&s,0,1,51));
    s=armed();i=healthy();i.navigation_event=1;i.nav=4;vc_step(&s,&i);
    /* A decoded turn waits for the junction, then the 90 mm sensor-to-wheel offset. */
    assert(s.phase==VC_TRACK && s.pending_nav==4 && s.command==1);i.navigation_event=0;
    i.junction=1;vc_step(&s,&i);assert(s.phase==VC_TRACK && s.nav_offset && s.command==1);
    i.junction=0;i.distance_um=90000;vc_step(&s,&i);
    /* No turn settings (power 0, no angle window): armed power, exactly 180 deg by yaw.
     * (vehicle.c sends 25 % in every build; the window only in straddle builds.) */
    assert(s.phase==VC_NAV_TURN);i.yaw_mdeg=179000;i.line_touch=1;vc_step(&s,&i);
    assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==30);
    i.yaw_mdeg=181000;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command && s.spot_scan);
    /* Turn power never exceeds the armed (Start) power. */
    s=armed();s.maximum_power=10;i=healthy();i.nav_turn_power=25;i.nav_seek_power=15;
    i.navigation_event=1;i.nav=2;vc_step(&s,&i);i.navigation_event=0;
    i.junction=1;vc_step(&s,&i);i.junction=0;i.distance_um=90000;vc_step(&s,&i);
    assert(s.phase==VC_NAV_TURN);vc_step(&s,&i);assert(s.command==4 && s.power==10);
    /* Straddle settings: 25 % to 60 deg, a touch there is ignored, then 15 %
     * until a sensor touches the new line and releases it into the gap
     * (centred), which ends a LEFT (+90, S2 LEFT 3) turn. */
    s=armed();i=healthy();i.nav_turn_power=25;i.nav_seek_power=15;
    i.nav_turn_min_deg=60;i.nav_turn_max_deg=120;i.nav_uturn_min_deg=150;i.nav_uturn_max_deg=210;
    i.spot_scan_all=1;i.navigation_event=1;i.nav=1;vc_step(&s,&i);i.navigation_event=0;
    i.junction=1;vc_step(&s,&i);i.junction=0;i.distance_um=90000;vc_step(&s,&i);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==90);
    i.line_touch=1;i.line_black_mask=2;i.yaw_mdeg=45000;vc_step(&s,&i);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    i.line_touch=0;i.line_black_mask=0;i.yaw_mdeg=70000;vc_step(&s,&i);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    i.line_touch=1;i.line_black_mask=2;i.yaw_mdeg=82000;vc_step(&s,&i);assert(s.phase==VC_NAV_TURN && s.command==3 && s.nav_touched);
    i.line_touch=0;i.line_black_mask=0;i.yaw_mdeg=88000;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.spot_scan && !s.command);
    vc_step(&s,&i);assert(s.phase==VC_TRACK);
    /* Any close obstacle overrides current steering in the same iteration. */
    s=armed();i=healthy();i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command);
    i.scan_ready=1;i.turn_deg=30;i.approach_mm=200;i.return_mm=100;vc_step(&s,&i);
    assert(s.phase==VC_FAULT && s.fault==VC_UNCALIBRATED);
    s=armed();i=healthy();i.obstacle=1;i.front_clear=0;i.navigation_event=1;i.nav=2;vc_step(&s,&i);
    assert(s.pending_nav==2);i.navigation_event=0;
    i.scan_ready=1;i.bypass_enabled=1;i.turn_deg=30;i.approach_mm=200;i.return_mm=100;
    vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
    /* front_clear=0 does not gate it; the reverse ramps up from START_POWER */
    vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_START_POWER);
    /* Distance bound: stand still before the forward turn-out. */
    i.distance_um=-300000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && s.backup_braking);
    i.now_ms=100+AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);
    assert(s.phase==VC_TURN_OUT && !s.command && s.backup_mm==300 && s.pass_check_mm==475);
    vc_step(&s,&i);assert(!s.command); /* needs a fresh, forward-facing reading */
    i.front_clear=1;vc_step(&s,&i);assert(s.command==1 && s.steering==-376);
    i.yaw_mdeg=30000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1);
    i.distance_um=-100000;vc_step(&s,&i);assert(s.phase==VC_ALIGN && s.command==1 && s.return_mm==200);
    i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_PASS && !s.look_side && s.command==1);
    i.distance_um+=475000;vc_step(&s,&i);assert(!s.command && s.look_side);
    i.front_clear=0;i.side_observed=i.side_ready=i.side_clear=1;
    for(unsigned n=0;n<3;++n){i.now_ms=i.side_sample_ms=120U+AVOID_BACKUP_BRAKE_MS+n*20U;vc_step(&s,&i);assert(!s.command);}
    assert(s.phase==VC_TURN_IN);i.front_clear=1;
    i.yaw_mdeg=-30000;vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1);
    i.distance_um+=200000;vc_step(&s,&i);assert(s.phase==VC_RESTORE && s.command==1);
    i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command);
    i.steering=0;vc_step(&s,&i);assert(s.command==3);
    i.steering=1;vc_step(&s,&i);assert(s.line_settling);
    i.now_ms+=20;vc_step(&s,&i);assert(s.phase==VC_TRACK && !s.command);
    assert(s.pending_nav==2);i.obstacle=0;
    vc_track_requested(&s,i.now_ms);i.steering=0;vc_step(&s,&i);
    assert(!s.command && s.armed && s.pending_nav==2);
    i.steering=1;vc_step(&s,&i);
    assert(s.phase==VC_TRACK && s.pending_nav==2 && s.command==1 && !s.nav_offset);
    i.junction=1;vc_step(&s,&i);i.junction=0;i.distance_um+=89999;vc_step(&s,&i);
    assert(s.phase==VC_TRACK && s.nav_offset && s.command==1 && !s.steering && s.pending_nav==2);
    i.distance_um+=1;vc_step(&s,&i);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==-90 && !s.pending_nav && !s.command);
    /* Never resume expired or interrupted open-loop maneuvers. */
    s=armed();s.phase=VC_TURN_OUT;s.entered_ms=100;i=healthy();i.now_ms=10101;
    s.armed_ms=10101;vc_step(&s,&i);assert(s.phase==VC_FAULT);
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
