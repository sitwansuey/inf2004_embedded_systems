/* Barcode turns run at the next junction after the sensor-to-wheel offset,
 * spin until a straddle sensor finds the new line, and spot-scan when they
 * end without one; a line lost while tracking spot-scans too (pure policy). */
#include "../../app_program/vehicle_policy.h"
#include <assert.h>
#include <stdio.h>
static vc_input_t healthy(void) {
    return (vc_input_t){.now_ms=100,.sensor_fresh=1,.motor_ok=1,.terrain_valid=1,.odom_valid=1,
        .transport_ok=1,.front_clear=1,.line_fresh=1,.steering=1,.line_detected=1,
        .nav_offset_mm=90,.nav_search_mm=600,
        /* robot_config.h defaults */
        .nav_turn_power=25,.nav_seek_power=15,.nav_turn_min_deg=60,.nav_turn_max_deg=120,
        .nav_uturn_min_deg=150,.nav_uturn_max_deg=210};
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
    /* The right sensor is still on black: before 60 deg that is not the new line. */
    i.yaw_mdeg=-45000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==25);
    /* Past 60 deg a touch is the new line, but only its near edge: keep
     * turning until the sensor releases it into the gap (centred). */
    i.yaw_mdeg=-80000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==15 && s.nav_touched);
    line(&i,0,0);i.yaw_mdeg=-88000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan && !s.command);
    /* Left/right REACQUIRE is unchanged: fresh straddle data is the line. */
    drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.command);
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
    i->yaw_mdeg=90000;drive(&s,i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    /* No touch from 150 deg: seek on to 210 deg, then spot-scan. */
    i->yaw_mdeg=180000;drive(&s,i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    i->yaw_mdeg=210000;drive(&s,i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan && !s.command);
    return s;
}
static void spot_scan(void) {
    vc_input_t i=healthy();vc_state_t s=u_turned(&i);
    /* Fresh, "steerable" straddle data with both sensors white is not the
     * line: rotate on the spot, slowly, through the bounded sweep. */
    assert(i.line_fresh && i.steering && i.line_detected && !i.line_touch);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==3 && s.power<=15);
    i.yaw_mdeg=225000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==3 && s.power<=15);
    i.yaw_mdeg=240000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.command); /* +30 reached */
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4 && s.power<=15);
    i.yaw_mdeg=215000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4);
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
    /* BOARD_NAV_SPOT_SCAN_ALL: left/right turns without a touch spot-scan too. */
    i=healthy();i.spot_scan_all=1;s=decoded(&i,1);
    line(&i,3,1);drive(&s,&i,0);line(&i,0,0);drive(&s,&i,90000);
    assert(s.phase==VC_NAV_TURN && s.turn_deg==90);
    i.yaw_mdeg=90000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    i.yaw_mdeg=120000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan);
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
/* Junction reached and the turn entered (the axle is over the crossing). */
static vc_state_t turning(vc_input_t *i,unsigned nav) {
    vc_state_t s=decoded(i,nav);
    line(i,3,1);drive(&s,i,0);line(i,0,0);drive(&s,i,90000);
    assert(s.phase==VC_NAV_TURN && !s.command && !s.pending_nav);
    return s;
}
/* 25 % by encoder yaw to the min angle (a touch before it is the old line or
 * the crossing), then 15 % until a straddle sensor touches the new line; no
 * touch by the max angle ends it anyway, then a spot scan. */
static void line_guided_turns(void) {
    /* LEFT is +90 deg, spun counterclockwise: S2 LEFT (3). */
    vc_input_t i=healthy();i.spot_scan_all=1;vc_state_t s=turning(&i,1);
    assert(s.turn_deg==90);
    drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    line(&i,2,0);i.yaw_mdeg=20000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    line(&i,3,1);i.yaw_mdeg=59999;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    line(&i,0,0);i.yaw_mdeg=60000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    i.yaw_mdeg=75000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    /* The first touch is the line's near edge: turn on until the sensor
     * releases it (centred between the sensors), then track: no spot scan. */
    line(&i,2,0);i.yaw_mdeg=78000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15 && s.nav_touched);
    i.yaw_mdeg=84000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3);
    line(&i,0,0);i.yaw_mdeg=88000;drive(&s,&i,0);
    assert(s.phase==VC_REACQUIRE && !s.spot_scan && !s.command);
    drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.command);
    drive(&s,&i,0);assert(forward(&s));
    /* A touch from exactly the min angle counts; just before it does not. */
    i=healthy();s=turning(&i,1);line(&i,1,0);i.yaw_mdeg=59000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.power==25 && !s.nav_touched);
    i.yaw_mdeg=60000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.nav_touched);
    line(&i,0,0);i.yaw_mdeg=62000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan);
    /* Both sensors on the new line is centred too. */
    i=healthy();s=turning(&i,1);line(&i,3,0);i.yaw_mdeg=85000;drive(&s,&i,0);
    assert(s.phase==VC_REACQUIRE && !s.spot_scan);
    /* A white sample without a touch first is not a release. */
    i=healthy();s=turning(&i,1);line(&i,0,0);i.yaw_mdeg=70000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && !s.nav_touched);
    /* A stale line sample neither touches nor releases. */
    line(&i,2,0);i.line_fresh=0;i.yaw_mdeg=75000;drive(&s,&i,0);i.line_fresh=1;
    assert(s.phase==VC_FAULT || (s.phase==VC_NAV_TURN && !s.nav_touched));
    /* RIGHT is -90 deg, clockwise: S2 RIGHT (4). No touch: ends at 120 deg. */
    i=healthy();i.spot_scan_all=1;s=turning(&i,2);assert(s.turn_deg==-90);
    drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==25);
    i.yaw_mdeg=-90000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==15);
    i.yaw_mdeg=-119999;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==15);
    i.yaw_mdeg=-120000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan && !s.command);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && (s.command==3 || s.command==4) && s.power<=15);
    /* Turning the wrong way never counts as progress. */
    i=healthy();s=turning(&i,2);line(&i,1,0);i.yaw_mdeg=70000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.command==4 && s.power==25);
    /* Without SPOT_SCAN_ALL a line-guided turn that ran to its max angle
     * without a touch has overshot the arm: it still searches for it. */
    i=healthy();s=turning(&i,2);i.yaw_mdeg=-120000;drive(&s,&i,0);
    assert(s.phase==VC_REACQUIRE && s.spot_scan);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && (s.command==3 || s.command==4));
    /* ...but a turn that ended centred on the line tracks at once. */
    i=healthy();s=turning(&i,2);i.yaw_mdeg=-70000;line(&i,1,0);drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.nav_touched);
    line(&i,0,0);i.yaw_mdeg=-80000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan);
    drive(&s,&i,0);assert(s.phase==VC_TRACK);
    /* A touch that never releases ends at the max angle; the spot scan then
     * sees the sensor on black and tracks at once. */
    i=healthy();s=turning(&i,2);line(&i,1,0);i.yaw_mdeg=-80000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.nav_touched);
    i.yaw_mdeg=-120000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.spot_scan);
    drive(&s,&i,0);assert(s.phase==VC_TRACK);
    /* Slope cap, a front stop and the 5 s phase timeout still apply. */
    i=healthy();s=turning(&i,1);i.slope=1;drive(&s,&i,0);assert(s.command==3 && s.power==20);
    i.slope=0;i.front_clear=0;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && !s.command);
    i.front_clear=1;i.now_ms+=4970;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3);
    drive(&s,&i,0);assert(s.phase==VC_FAULT && s.fault==VC_TIMEOUT);
    puts("Line-guided turns: 25% to 60 deg, early touches ignored, touch then release (centred) to 120 deg ends it, else spot scan PASS");
}
/* A U-turn passes the side arm near 90 deg: only touches from 150 deg count,
 * and 210 deg ends it without one. */
static void line_guided_uturn(void) {
    vc_input_t i=healthy();i.uturn_ignore_mm=1200;vc_state_t s=turning(&i,4);
    assert(s.turn_deg==180);
    drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    line(&i,1,0);i.yaw_mdeg=85000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    i.yaw_mdeg=120000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.power==25);
    line(&i,0,0);i.yaw_mdeg=149999;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.power==25);
    i.yaw_mdeg=150000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15);
    line(&i,2,0);i.yaw_mdeg=172000;drive(&s,&i,0);
    assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==15 && s.nav_touched);
    line(&i,0,0);i.yaw_mdeg=178000;drive(&s,&i,0);
    assert(s.phase==VC_REACQUIRE && !s.spot_scan && !s.command && s.uturn_guard);
    drive(&s,&i,0);assert(s.phase==VC_TRACK);
    /* Encoder yaw alone (max 0): exactly 90 deg, any touch ignored. */
    i=healthy();i.nav_turn_max_deg=0;s=turning(&i,1);line(&i,2,0);
    i.yaw_mdeg=70000;drive(&s,&i,0);assert(s.phase==VC_NAV_TURN && s.command==3 && s.power==25);
    i.yaw_mdeg=90000;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan);
    puts("Line-guided U-turn: side arm ignored before 150 deg, touch then release ends it, guard kept PASS");
}
/* Line lost while tracking: both straddle sensors white is also "centred",
 * so line_lost_mm of forward travel with neither on black spot-scans, toward
 * the line's last side first; a sensor on black resumes tracking, none in the
 * bounded sweep is VC_LINE not found. */
static vc_state_t tracking(vc_input_t *i) {
    vc_state_t s={0};assert(vc_start(&s,i->now_ms,30));
    i->line_lost_mm=600;drive(&s,i,0);assert(forward(&s));return s;
}
static void line_lost(void) {
    vc_input_t i=healthy();vc_state_t s=tracking(&i);
    /* Touches 550 mm apart (right, left, a junction): never. */
    for(unsigned n=0;n<300U;++n) {
        bool touch=n%55U==54U;unsigned m=n/55U%3U+1U;
        line(&i,touch?m:0U,touch && m==3U);drive(&s,&i,10000);assert(forward(&s));
    }
    /* The line goes after a right touch: 600 mm on, search right first. */
    line(&i,1,0);drive(&s,&i,10000);assert(forward(&s) && s.last_line_side==1);
    line(&i,0,0);
    for(unsigned n=0;n<59U;++n) {drive(&s,&i,10000);assert(forward(&s));}
    drive(&s,&i,9999);assert(forward(&s)); /* 599.999 mm */
    drive(&s,&i,1);assert(s.phase==VC_REACQUIRE && s.spot_scan && s.search_right_first && !s.command);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4 && s.power==15);
    /* The right sensor finds black: settle, then track. */
    i.yaw_mdeg=-5000;line(&i,1,0);drive(&s,&i,0);
    assert(s.phase==VC_REACQUIRE && s.line_settling && s.command==4 && s.power==15);
    drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.command==4);
    drive(&s,&i,0);assert(s.phase==VC_TRACK && !s.spot_scan && !s.command);
    drive(&s,&i,0);assert(forward(&s));
    /* Lost again after a left touch: left first, never found, stopped after 4 s. */
    line(&i,2,0);drive(&s,&i,10000);line(&i,0,0);
    for(unsigned n=0;n<60U && s.phase==VC_TRACK;++n) {assert(forward(&s));drive(&s,&i,10000);}
    assert(s.phase==VC_REACQUIRE && s.spot_scan && !s.search_right_first);
    uint32_t from=i.now_ms;
    drive(&s,&i,0);assert(s.command==3 && s.power==15);
    while(s.armed) {drive(&s,&i,0);assert(!s.command || ((s.command==3 || s.command==4) && s.power<=15));}
    assert(s.phase==VC_FAULT && s.fault==VC_LINE && s.fault_cause==VC_LINE_CAUSE_NOT_FOUND && i.now_ms-from==4000U);
    /* Off (0, as during a barcode read or re-read): never; on again, the
     * distance runs from there. */
    i=healthy();s=tracking(&i);i.line_lost_mm=0;
    for(unsigned n=0;n<100U;++n) {drive(&s,&i,10000);assert(forward(&s));}
    i.line_lost_mm=600;drive(&s,&i,599999);assert(forward(&s));
    drive(&s,&i,1);assert(s.phase==VC_REACQUIRE && s.spot_scan);
    /* Reversing never adds: 500 on, 300 back, 400 on is 600. */
    i=healthy();s=tracking(&i);
    drive(&s,&i,500000);drive(&s,&i,-300000);drive(&s,&i,399999);assert(forward(&s));
    drive(&s,&i,1);assert(s.phase==VC_REACQUIRE);
    /* Back past the last black: from the furthest point back. */
    i=healthy();s=tracking(&i);
    drive(&s,&i,100000);drive(&s,&i,-300000);drive(&s,&i,599999);assert(forward(&s));
    drive(&s,&i,1);assert(s.phase==VC_REACQUIRE);
    /* Entering TRACK starts it again (an obstacle that went away). */
    i=healthy();s=tracking(&i);drive(&s,&i,500000);
    i.obstacle=1;i.front_clear=0;drive(&s,&i,0);assert(s.phase==VC_ACQUIRE);
    i.obstacle=0;i.front_clear=1;drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && !s.spot_scan);
    drive(&s,&i,0);assert(s.phase==VC_TRACK);drive(&s,&i,0);
    drive(&s,&i,599999);assert(forward(&s));
    drive(&s,&i,1);assert(s.phase==VC_REACQUIRE && s.spot_scan);
    /* A decoded turn looking for its junction keeps its own bound: no
     * junction within nav_search_mm, it turns there. */
    i=healthy();i.nav_search_mm=1000;s=tracking(&i);
    i.navigation_event=1;i.nav=2;drive(&s,&i,0);i.navigation_event=0;i.nav=0;
    for(unsigned n=0;n<99U;++n) {drive(&s,&i,10000);assert(forward(&s) && s.pending_nav==2);}
    drive(&s,&i,9999);assert(forward(&s));
    drive(&s,&i,1);assert(s.phase==VC_NAV_TURN && s.turn_deg==-90);
    puts("Line lost while tracking: line_lost_mm without black spot-scans toward its side; found -> TRACK, never -> VC_LINE not found; touches, off, reversing, TRACK entry and a pending turn restart it PASS");
}
/* C7: the 4 s spot-search bound also ends a search the sonar holds (an
 * object within 50 mm from 3.9 s on) or late line data holds: VC_LINE not
 * found at 4 s, never the 5 s phase limit (VC_TIMEOUT) or a line-data
 * fault. A held search that found the line and is settling is not cut. */
static void held_search(void) {
    for(unsigned hold=0;hold<2U;++hold) {
        vc_input_t i=healthy();vc_state_t s=tracking(&i);
        line(&i,0,0);
        while(s.phase==VC_TRACK) drive(&s,&i,10000);
        assert(s.phase==VC_REACQUIRE && s.spot_scan);
        uint32_t from=i.now_ms;
        while(i.now_ms-from<3900U) {drive(&s,&i,0);assert(s.armed && s.phase==VC_REACQUIRE);}
        if(hold) i.front_clear=0; else i.line_fresh=0;
        while(s.armed) {drive(&s,&i,0);assert(!s.command);}
        assert(s.phase==VC_FAULT && s.fault==VC_LINE && s.fault_cause==VC_LINE_CAUSE_NOT_FOUND &&
               i.now_ms-from==VC_SEARCH_MS);
    }
    /* Found at 3.99 s, then held by the sonar while settling: the bound
     * leaves it to the phase limit (it is not a search any more). */
    vc_input_t i=healthy();vc_state_t s=tracking(&i);
    line(&i,0,0);
    while(s.phase==VC_TRACK) drive(&s,&i,10000);
    uint32_t from=i.now_ms;
    while(i.now_ms-from<3990U) drive(&s,&i,0);
    line(&i,1,0);drive(&s,&i,0);assert(s.phase==VC_REACQUIRE && s.line_settling);
    i.front_clear=0;drive(&s,&i,0);drive(&s,&i,0);assert(s.armed && s.phase==VC_REACQUIRE && !s.command);
    i.front_clear=1;drive(&s,&i,0);assert(s.phase==VC_TRACK);
    puts("Spot search held by the sonar or late line data: VC_LINE not found at 4 s, not the phase limit; a found line still settles PASS");
}
int main(void) {
    right_at_junction();hold_and_search_limit();leave_track();spot_scan();uturn_repeat();
    line_guided_turns();line_guided_uturn();line_lost();held_search();
    return 0;
}
