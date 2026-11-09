/* Barcode turns run at the next junction after the sensor-to-wheel offset;
 * a U-turn then spot-scans until a line sensor is on black (pure policy). */
#include "../../app_program/vehicle_policy.h"
#include <assert.h>
#include <stdio.h>
static vc_input_t healthy(void) {
    return (vc_input_t){.now_ms=100,.sensor_fresh=1,.motor_ok=1,.terrain_valid=1,.odom_valid=1,
        .transport_ok=1,.front_clear=1,.line_fresh=1,.steering=1,.line_detected=1,.line_state=1,
        .nav_offset_mm=90,.nav_search_mm=600};
}
/* Tracking at 30% when turn `nav` is decoded here. */
static vc_state_t decoded(vc_input_t *i,unsigned nav) {
    vc_state_t s={0};assert(vc_start(&s,i->now_ms,30));
    i->navigation_event=1;i->nav=(uint8_t)nav;vc_step(&s,i);
    i->navigation_event=0;i->nav=0;return s;
}
static void drive(vc_state_t *s,vc_input_t *i,int32_t um) {i->now_ms+=10;i->distance_um+=um;vc_step(s,i);}
static bool forward(const vc_state_t *s) {return s->phase==VC_TRACK && s->command==1 && s->power==30 && !s->steering;}
static void line(vc_input_t *i,unsigned mask,unsigned gp28) {
    i->line_black_mask=(uint8_t)mask;i->line_touch=mask!=0U;i->junction=mask==3U && gp28;
}
static void right_at_junction(void) {
    vc_input_t i=healthy();vc_state_t s=decoded(&i,2);
    assert(forward(&s) && s.pending_nav==2 && s.nav_search && !s.nav_offset);
    /* Ordinary tracking for 400 mm: side touches, and both line sensors on
     * black with GP28 white (a bend) is not a junction. */
    for(unsigned n=0;n<40;++n) {
        line(&i,n%4U,0);drive(&s,&i,10000);
        assert(forward(&s) && s.pending_nav==2 && !s.nav_offset);
    }
    line(&i,3,1);drive(&s,&i,5000);assert(forward(&s) && s.nav_offset && s.pending_nav==2);
    /* Still on the crossing line: measured from the first junction tick. */
    drive(&s,&i,5000);assert(forward(&s) && s.nav_offset);
    line(&i,1,0);
    for(unsigned n=0;n<8;++n) {drive(&s,&i,10000);assert(forward(&s) && s.nav_offset && s.pending_nav==2);}
    drive(&s,&i,4999);assert(forward(&s) && s.nav_offset); /* 89.999 mm */
    drive(&s,&i,1);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==-90 && !s.pending_nav && !s.command && !s.nav_offset);
    i.yaw_mdeg=-45000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4);
    i.yaw_mdeg=-90000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan && !s.command);
    /* Left/right REACQUIRE is unchanged: fresh straddle data is the line. */
    line(&i,0,0);drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.command);
    puts("Right turn waits through tracking, then 90 mm straight from the junction PASS");
}
static void hold_and_search_limit(void) {
    vc_input_t i=healthy();i.nav_hold=1;vc_state_t s=decoded(&i,1);
    assert(s.phase==VC_TRACK && s.pending_nav==1 && !s.nav_search);
    /* The re-read repositions (reverse, then forward over the marker): a
     * junction seen meanwhile is ignored. */
    line(&i,3,1);drive(&s,&i,-100000);
    assert(s.phase==VC_TRACK && s.pending_nav==1 && !s.nav_search && !s.nav_offset);
    drive(&s,&i,200000);assert(s.phase==VC_TRACK && s.pending_nav==1 && !s.nav_offset);
    line(&i,0,0);i.nav_hold=0;drive(&s,&i,0);
    assert(forward(&s) && s.nav_search && !s.nav_offset);
    /* No junction: turn after 600 mm of travel. */
    for(unsigned n=0;n<59;++n) {drive(&s,&i,10000);assert(forward(&s) && s.pending_nav==1);}
    drive(&s,&i,9999);assert(forward(&s) && s.pending_nav==1);
    drive(&s,&i,1);assert(s.phase==VC_NAV_TURN && s.turn_deg==90 && !s.pending_nav);
    /* STRAIGHT is still acknowledged at once; 0/0 distances turn at once. */
    i=healthy();s=decoded(&i,3);
    assert(s.phase==VC_TRACK && s.navigation_ack && !s.pending_nav && !s.command && !s.nav_search);
    i=healthy();i.nav_offset_mm=i.nav_search_mm=0;s=decoded(&i,2);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==-90 && !s.pending_nav);
    puts("Junction ignored during the re-read hold; 600 mm search limit; STRAIGHT and 0/0 immediate PASS");
}
static void leave_track(void) {
    vc_input_t i=healthy();vc_state_t s=decoded(&i,2);
    line(&i,3,1);drive(&s,&i,10000);assert(s.nav_offset);
    line(&i,0,0);drive(&s,&i,50000);assert(forward(&s) && s.nav_offset);
    /* An obstacle drops the junction progress but keeps the turn. */
    i.obstacle=1;i.front_clear=0;drive(&s,&i,0);
    assert(s.phase==VC_ACQUIRE && s.pending_nav==2 && !s.nav_offset && !s.nav_search);
    i.obstacle=0;i.front_clear=1;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.pending_nav==2);
    drive(&s,&i,0);assert(s.phase==VC_TRACK && s.pending_nav==2);
    /* Back in TRACK the search restarts; the old offset is not resumed. */
    drive(&s,&i,0);assert(forward(&s) && s.nav_search && !s.nav_offset);
    drive(&s,&i,100000);assert(forward(&s) && s.pending_nav==2);
    line(&i,3,1);drive(&s,&i,0);assert(forward(&s) && s.nav_offset);
    line(&i,0,0);drive(&s,&i,90000);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==-90 && !s.pending_nav);
    vc_stop(&s,VC_OK);assert(!s.nav_search && !s.nav_offset && !s.spot_scan);
    puts("Leaving TRACK resets the junction progress and keeps the turn PASS");
}
static vc_state_t u_turned(vc_input_t *i) {
    vc_state_t s=decoded(i,4);
    line(i,3,1);drive(&s,i,0);assert(s.nav_offset);
    line(i,0,0);drive(&s,i,90000);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==180 && !s.pending_nav);
    i->yaw_mdeg=90000;drive(&s,i,0);assert(s.phase==VC_NAV_TURN && s.command==3);
    i->yaw_mdeg=180000;drive(&s,i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan && !s.command);
    return s;
}
static void spot_scan(void) {
    vc_input_t i=healthy();vc_state_t s=u_turned(&i);
    /* Fresh, "steerable" straddle data with both sensors white is not the
     * line: rotate on the spot, slowly, through the bounded sweep. */
    assert(i.line_fresh && i.steering && i.line_detected && !i.line_touch);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==3 && s.power<=15);
    i.yaw_mdeg=195000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==3 && s.power<=15);
    i.yaw_mdeg=210000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.command); /* +30 reached */
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4 && s.power<=15);
    i.yaw_mdeg=185000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4);
    /* The right sensor touches the line: settle, then track. */
    line(&i,1,0);drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.line_settling && s.command==4 && s.power<=15);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4);
    drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.spot_scan && !s.command);
    drive(&s,&i,0);assert(forward(&s));
    /* Already touching right after the turn: TRACK at once. */
    i=healthy();s=u_turned(&i);
    line(&i,2,0);drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.spot_scan && !s.command);
    /* Never touching: the sweep stays slow and bounded, then stops. */
    i=healthy();s=u_turned(&i);
    for(unsigned n=0;n<500U && s.armed;++n) {
        drive(&s,&i,0);
        assert(!s.command || ((s.command==3 || s.command==4) && s.power<=15));
    }
    assert(s.phase==VC_FAULT && s.fault==VC_LINE);
    /* BOARD_NAV_SPOT_SCAN_ALL: left/right turns spot-scan too. */
    i=healthy();i.spot_scan_all=1;s=decoded(&i,1);
    line(&i,3,1);drive(&s,&i,0);line(&i,0,0);drive(&s,&i,90000);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==90);
    i.yaw_mdeg=90000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && (s.command==3 || s.command==4) && s.power<=15);
    puts("U-turn spot scan: sweeps until a sensor is on black, immediate when touching, bounded PASS");
}
/* After a U-turn the robot crosses the same marker again on the way back:
 * that repeat U-turn is acknowledged and ignored for uturn_ignore_mm. */
static void uturn_repeat(void) {
    vc_input_t i=healthy();i.uturn_ignore_mm=1200;vc_state_t s=u_turned(&i);
    assert(s.uturn_guard);
    line(&i,2,0);drive(&s,&i,0);assert(s.phase==VC_TRACK);
    line(&i,0,0);drive(&s,&i,400000);
    i.navigation_event=1;i.nav=4;drive(&s,&i,0);i.navigation_event=0;i.nav=0;
    assert(s.navigation_ack && !s.pending_nav && s.phase==VC_TRACK);
    drive(&s,&i,10000);assert(!s.navigation_ack && forward(&s));
    /* A different marker in the window is still obeyed. */
    i.navigation_event=1;i.nav=2;drive(&s,&i,0);i.navigation_event=0;i.nav=0;
    assert(s.pending_nav==2);
    /* ...and once well past, a U-turn marker counts again. */
    i.yaw_mdeg=0;s=u_turned(&i);line(&i,2,0);drive(&s,&i,0);line(&i,0,0);
    drive(&s,&i,1200000);assert(!s.uturn_guard);
    i.navigation_event=1;i.nav=4;drive(&s,&i,0);i.navigation_event=0;i.nav=0;
    assert(s.pending_nav==4 && !s.navigation_ack);
    /* Disabled (0): every U-turn marker acts, as before. */
    i=healthy();s=u_turned(&i);assert(!s.uturn_guard);
    puts("U-turn repeat: the same marker read on the way back does not turn again PASS");
}
int main(void) {
    right_at_junction();hold_and_search_limit();leave_track();spot_scan();uturn_repeat();
    return 0;
}
