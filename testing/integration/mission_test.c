#include "../../app_program/subsystem2/speed_pid.h"
#include "../../app_program/subsystem2/wheel_steering.h"
#include "../../app_program/subsystem3/buddy3_barcode.h"
#include "../../app_program/vehicle_policy.h"
#include <assert.h>
#include <stdio.h>
static vc_input_t healthy(void) {
    return (vc_input_t){.now_ms=10,.sensor_fresh=1,.motor_ok=1,.terrain_valid=1,
        .odom_valid=1,.transport_ok=1,.front_clear=1,.line_fresh=1,.steering=1};
}
static void handoffs(void) {
    vc_state_t s={0};vc_input_t i=healthy();
    assert(vc_start(&s,0,30));
    /* A 150mm obstacle remains an obstacle even if manoeuvre clearance is
     * true. Its navigation marker survives the acquisition/recovery cycle. */
    i.obstacle=1;i.front_clear=0;i.navigation_event=1;i.nav=1;
    vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && s.pending_nav==1 && !s.command);
    i.now_ms=20;i.navigation_event=0;i.front_clear=1;
    vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && s.pending_nav==1 && !s.command);
    i.now_ms=30;i.obstacle=0;
    vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && s.pending_nav==1);
    i.now_ms=40;i.line_detected=1;
    vc_step(&s,&i);assert(s.phase==VC_TRACK && s.pending_nav==1);
    vc_track_requested(&s,40);i.now_ms=50;i.steering=0;
    vc_step(&s,&i);assert(s.armed && s.track_wait && !s.command);
    i.now_ms=60;i.steering=1;
    vc_step(&s,&i);assert(s.phase==VC_NAV_TURN && s.turn_deg==90 && !s.pending_nav);
    /* Delayed STRAIGHT must acknowledge only when actually consumed. */
    s=(vc_state_t){0};assert(vc_start(&s,0,30));i=healthy();
    i.front_clear=0;i.navigation_event=1;i.nav=3;i.steering=0;
    vc_step(&s,&i);assert(s.pending_nav==3 && !s.navigation_ack && !s.command);
    i.navigation_event=0;i.front_clear=1;vc_step(&s,&i);
    assert(s.navigation_ack && !s.pending_nav && s.armed && !s.command);
    vc_track_requested(&s,10);i.now_ms=20;vc_step(&s,&i);
    assert(s.track_wait && !s.navigation_ack && !s.command);
    i.now_ms=30;i.steering=1;vc_step(&s,&i);assert(s.command==1 && !s.track_wait);
    /* Bounded wait survives timer wrap; missing samples still stop the car. */
    s=(vc_state_t){0};assert(vc_start(&s,0,30));vc_track_requested(&s,UINT32_MAX-20U);
    i=healthy();i.steering=0;i.line_fresh=0;i.now_ms=78;
    vc_step(&s,&i);assert(s.armed && !s.command);
    i.now_ms=79;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_LINE);
    s=(vc_state_t){0};assert(vc_start(&s,0,30));s.pending_nav=2;vc_track_requested(&s,0);
    i=healthy();i.shock=1;vc_step(&s,&i);
    assert(s.fault==VC_SHOCK && !s.pending_nav && !s.track_wait);
    s=(vc_state_t){0};assert(vc_start(&s,0,30));s.pending_nav=4;
    vc_stop(&s,VC_OK);assert(!s.pending_nav && !s.armed);
    puts("Mission handoffs: obstacle priority, deferred navigation, TRACK acknowledgement, timeout/wrap and STOP PASS");
}
int main(void) {
    handoffs();
    vc_state_t s={0};vc_input_t i=healthy();
    assert(vc_start(&s,0,30));i.now_ms=5000;vc_step(&s,&i);
    assert(s.armed && s.command==1); /* no remote renewal */
    i.recovery_event=1;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE);
    i.recovery_event=0;i.steering=0;vc_step(&s,&i);assert(s.command==3 && s.power<=15);
    i.front_clear=0;vc_step(&s,&i);assert(!s.command);
    i.front_clear=1;i.yaw_mdeg=30000;vc_step(&s,&i);assert(!s.command);
    vc_step(&s,&i);assert(s.command==4); /* sweep across, not endless turn */
    i.line_detected=1;vc_step(&s,&i);
    assert(s.phase==VC_REACQUIRE && s.line_settling && s.command==4);
    i.now_ms+=19;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && s.command==4);
    i.now_ms+=1;vc_step(&s,&i);assert(s.phase==VC_TRACK && !s.command);
    i.line_detected=0;i.steering=1;i.recovery_event=1;vc_step(&s,&i);
    i.recovery_event=0;i.steering=0;i.now_ms+=4000;vc_step(&s,&i);
    assert(s.phase==VC_FAULT && !s.armed && !s.command);
    speed_pid_t p={0};
    assert(speed_pid_step(&p,100,0,20,20,50,100,10,0)>20);
    assert(speed_pid_step(&p,100,200,20,20,50,100,10,0)<20);
    p=(speed_pid_t){0};
    for(unsigned n=0;n<1000;++n)assert(speed_pid_step(&p,1000,0,20,40,50,100,100,0)==50);
    assert(p.integral==0); /* anti-windup */
    assert(speed_pid_step(&p,0,1000,20,0,50,100,100,0)==0);
    assert(speed_pid_step(&p,100,0,101,20,50,100,10,0)==0);
    assert(s2_steering_step(0,250,20)==40);
    assert(s2_steering_step(40,-250,20)==0);
    assert(s2_steering_scale(250,0)==1250 && s2_steering_scale(250,1)==750);
    assert(s2_steering_scale(-250,0)==750 && s2_steering_scale(-250,1)==1250);
    /* Known Code39 A/B/C/D patterns, framed by '*', in both directions. */
    const unsigned patterns[4]={0x109,0x049,0x148,0x019};
    for(unsigned letter=0;letter<4;++letter) {
        uint32_t runs[S3_BARCODE_RUN_COUNT],reverse[S3_BARCODE_RUN_COUNT];
        for(unsigned ch=0;ch<3;++ch) {
            unsigned pattern=ch==1?patterns[letter]:0x094;
            for(unsigned n=0;n<9;++n)runs[ch*10+n]=(pattern&(1U<<(8U-n)))?300U:100U;
            if(ch<2)runs[ch*10+9]=100;
        }
        s3_symbol_t symbol;
        assert(s3_barcode_decode(runs,S3_BARCODE_RUN_COUNT,&symbol)==S3_BARCODE_OK);
        assert(symbol.symbol==(char)('A'+letter));
        for(unsigned n=0;n<S3_BARCODE_RUN_COUNT;++n)reverse[n]=runs[S3_BARCODE_RUN_COUNT-1-n];
        assert(s3_barcode_decode(reverse,S3_BARCODE_RUN_COUNT,&symbol)==S3_BARCODE_OK);
        assert(symbol.symbol==(char)('A'+letter));
    }
    puts("Mission: autonomous start, bounded reacquisition, PID limits/anti-windup, A-D forward/reverse PASS");
    return 0;
}
