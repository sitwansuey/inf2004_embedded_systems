/* Obstacle bypass in the vehicle policy (vehicle_policy.c), scenario by
 * scenario: the side look at the end of the out diagonal (clear: a short
 * straight run, then straight back through the heading plus the return
 * heading, at least AVOID_RETURN_MIN_DEG; still there: align and pass it,
 * looking every AVOID_PASS_STEP_MM), the return that drives on until the
 * line is under the robot (lead-sensor touch kept across
 * TURN_IN/RETURN/RESTORE) or, with no line, AVOID_RETURN_MAX_MM (or
 * AVOID_RETURN_EXTRA_MM past the expected distance, if further), RESTORE to
 * the heading held at detection, then TRACK or the spot
 * search; no line search for the first AVOID_TURN_IN_LINE_DELAY_MS of the
 * turn-in (from its first drive tick, on into RETURN), then the far sensor
 * alone also counts for AVOID_TURN_IN_FAR_MS; and the line-data and sonar
 * gap holds. */
#include "../../app_program/vehicle_policy.h"
#include <stdio.h>
#include <string.h>
static int fails;
#define CHECK(c,msg) do{ if(!(c)){printf("FAIL %s (line %d)\n",msg,__LINE__);++fails;} }while(0)
static vc_input_t base(uint32_t t,int32_t dist_um,int32_t yaw) {
    vc_input_t i;memset(&i,0,sizeof i);
    i.now_ms=t;i.distance_um=dist_um;i.yaw_mdeg=yaw;
    i.sensor_fresh=1;i.terrain_valid=1;i.odom_valid=1;i.transport_ok=1;i.motor_ok=1;
    i.front_clear=1;i.line_fresh=1;i.steering=1;i.line_detected=1;
    return i;
}
/* Found line: expect VC_RESTORE, then drive yaw back to avoid_yaw until TRACK. */
static int restore_then_track(vc_state_t *s,uint32_t *t,int32_t *d,int32_t *yaw) {
    if (s->phase==VC_TRACK) return 1; /* already on the heading: straight to TRACK */
    if (s->phase!=VC_RESTORE || !s->line_found) return 0;
    for (int k=0;k<400 && s->phase==VC_RESTORE;++k) {
        int32_t err=s->avoid_yaw-*yaw; *yaw+=err>1000?1000:err< -1000?-1000:err;
        vc_input_t i=base(*t+=10,*d+=1000,*yaw);vc_step(s,&i);
    }
    return s->phase==VC_TRACK;
}
/* n VC ticks of 20 ms (the robot's real period), driving on, with this line mask. */
static void ticks(vc_state_t *s,uint32_t *t,int32_t *d,int32_t yaw,unsigned n,unsigned mask) {
    for (unsigned k=0;k<n;++k) {vc_input_t i=base(*t+=20,*d+=2000,yaw);i.line_black_mask=(uint8_t)mask;vc_step(s,&i);}
}
/* Turn-in ticks to the end of the line wait: its first drive tick counts 0 ms. */
#define WAIT_TICKS (AVOID_TURN_IN_LINE_DELAY_MS/20U)
static void armed_in(vc_state_t *s,unsigned phase,int turn_deg,unsigned return_mm) {
    memset(s,0,sizeof *s);
    vc_start(s,0,20);
    s->phase=(uint8_t)phase;s->turn_deg=(int16_t)turn_deg;s->side=turn_deg<0;
    s->return_mm=(uint16_t)return_mm;s->bypass_steering=200;s->entered_ms=0;
    s->origin_um=0;s->origin_yaw=0;
}
int main(void) {
    vc_state_t s;uint32_t t=0;int32_t d=0;int32_t yaw=0;
    /* 1. RETURN, left bypass (turn_deg>0, lead = right sensor bit0): touch, release -> TRACK. */
    armed_in(&s,VC_RETURN,30,200);
    unsigned masks1[]={0,0,1,1,1,0};
    for(unsigned k=0;k<6;++k){t+=10;d+=2000;vc_input_t i=base(t,d,0);i.line_black_mask=(uint8_t)masks1[k];vc_step(&s,&i);}
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw),"touch then release -> RESTORE -> TRACK");
    /* 2. Lead touch alone does not hand over. */
    armed_in(&s,VC_RETURN,30,200);t=0;d=0;
    for(unsigned k=0;k<5;++k){t+=10;d+=2000;vc_input_t i=base(t,d,0);i.line_black_mask=1;vc_step(&s,&i);}
    CHECK(s.phase==VC_RETURN && s.command==1,"lead touch alone keeps heading back");
    /* 3. Touch in TURN_IN survives into RETURN; release there -> TRACK. (The
     * turn first drives AVOID_TURN_IN_LINE_DELAY_MS with the search off.) */
    armed_in(&s,VC_TURN_IN,30,200);t=0;d=0;
    yaw=0;
    ticks(&s,&t,&d,yaw,WAIT_TICKS,0);
    ticks(&s,&t,&d,yaw,1,1);
    CHECK(s.nav_touched==1,"touch latched in TURN_IN");
    for(int k=0;k<40 && s.phase==VC_TURN_IN;++k){yaw-=2000;vc_input_t i=base(t+=10,d+=2000,yaw);i.line_black_mask=1;vc_step(&s,&i);}
    CHECK(s.phase==VC_RETURN,"TURN_IN completed into RETURN");
    CHECK(s.nav_touched==1,"touch kept across TURN_IN -> RETURN");
    {vc_input_t i=base(t+=10,d+=2000,yaw);i.line_black_mask=0;vc_step(&s,&i);}
    CHECK(restore_then_track(&s,&t,&d,&yaw),"release in RETURN after a TURN_IN touch -> RESTORE -> TRACK");
    /* 4. Line reaching the far sensor after the lead touch -> TRACK. */
    armed_in(&s,VC_RETURN,30,200);t=0;d=0;
    unsigned masks4[]={1,1,2};
    for(unsigned k=0;k<3;++k){vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=(uint8_t)masks4[k];vc_step(&s,&i);}
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw),"lead then far sensor -> RESTORE -> TRACK");
    /* 5. Both black at once -> TRACK. */
    armed_in(&s,VC_RETURN,-30,200);t=0;d=0;
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=3;vc_step(&s,&i);}
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw),"both black -> RESTORE -> TRACK");
    /* 6. Right bypass (turn_deg<0, lead = left sensor bit1): a right-sensor touch is not the lead. */
    armed_in(&s,VC_RETURN,-30,200);t=0;d=0;
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=1;vc_step(&s,&i);}
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_RETURN,"non-lead touch then release is not a hand-over");
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=2;vc_step(&s,&i);}
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);}
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw),"right bypass: left touch then release -> RESTORE -> TRACK");
    /* 7. No line: drives on straight well past the expected distance (200 mm
     * at 30 deg is 100 mm off the line: 141 mm at the 45 deg return heading,
     * plus the extra 391 mm), restoring only after AVOID_RETURN_MAX_MM. */
    armed_in(&s,VC_RETURN,30,200);t=0;d=0;
    while(s.phase==VC_RETURN && d<2000000){
        vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);
        if(s.phase==VC_RETURN)CHECK(s.command==1 && s.steering==0,"straight on in RETURN");
    }
    printf("no line: RETURN ended at %d mm (limit %u)\n",(int)(d/1000),AVOID_RETURN_MAX_MM);
    CHECK(s.phase==VC_RESTORE,"no line -> RESTORE");
    CHECK(d/1000>=(int)AVOID_RETURN_MAX_MM && d/1000<=(int)AVOID_RETURN_MAX_MM+2,
          "RETURN drove straight on to AVOID_RETURN_MAX_MM");
    /* 7b. A return expected further out (1500 mm at 30 deg: 750 mm off the
     * line, 1061 mm at 45 deg) keeps that plus the extra, 1311 mm. */
    armed_in(&s,VC_RETURN,30,1500);t=0;d=0;
    while(s.phase==VC_RETURN && d<3000000){vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_RESTORE && d/1000>=1061+(int)AVOID_RETURN_EXTRA_MM && d/1000<=1063+(int)AVOID_RETURN_EXTRA_MM,
          "long return: the expected distance plus the extra when beyond AVOID_RETURN_MAX_MM");
    /* 8. Touch in RETURN, distance exhausted, release in RESTORE -> TRACK. */
    armed_in(&s,VC_RETURN,30,0);t=0;d=0;
    while(s.phase==VC_RETURN && d<2000000){vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=1;vc_step(&s,&i);}
    CHECK(s.phase==VC_RESTORE && s.nav_touched,"touch kept into RESTORE");
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);}
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw),"release in RESTORE -> TRACK");
    /* 9. A stale touch from a nav turn does not carry into a new bypass return. */
    armed_in(&s,VC_PASS,30,200);t=0;s.nav_touched=1;s.look_side=1;s.side_check_started_ms=0;
    for(int k=0;k<4 && s.phase==VC_PASS;++k){vc_input_t i=base(t+=10,d,0);i.side_observed=1;i.side_ready=1;i.side_clear=1;i.side_sample_ms=t;vc_step(&s,&i);}
    for(int k=0;k<200 && s.phase==VC_PASS;++k){vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);} /* the clear run */
    CHECK(s.phase==VC_TURN_IN && !s.nav_touched,"PASS -> TURN_IN clears an old touch");
    /* 10. End of the diagonal: stop and look; 3 clear -> direct TURN_IN through 2x turn_deg. */
    armed_in(&s,VC_DIAGONAL,30,0);s.approach_mm=150;t=0;d=0;
    while(s.phase==VC_DIAGONAL && !s.look_side && d<1000000){vc_input_t i=base(t+=10,d+=2000,0);vc_step(&s,&i);}
    CHECK(s.look_side && s.phase==VC_DIAGONAL && s.return_mm>=150,"diagonal end starts a side look");
    {vc_input_t i=base(t+=10,d,0);vc_step(&s,&i);CHECK(s.command==0,"stopped while looking");}
    for(int k=0;k<3;++k){vc_input_t i=base(t+=70,d,0);i.side_observed=1;i.side_ready=1;i.side_clear=1;i.side_sample_ms=t;vc_step(&s,&i);}
    CHECK(s.phase==VC_DIAGONAL && s.clear_run,"diagonal clear -> short straight run first");
    {int32_t run0=d;while(s.phase==VC_DIAGONAL && d<run0+1000000){vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);CHECK(s.phase!=VC_DIAGONAL || s.command==1,"drives on during the run");}
     printf("clear run before turning back: %d mm\n",(int)((d-run0)/1000));
     /* 1 mm per 10 ms tick: AVOID_CLEAR_RUN_MS of driving (well past AVOID_CLEAR_RUN_MM). */
     CHECK(d-run0>=(int32_t)AVOID_CLEAR_RUN_MM*1000 && d-run0>=(int32_t)(AVOID_CLEAR_RUN_MS/10U)*1000 &&
           d-run0<=(int32_t)(AVOID_CLEAR_RUN_MS/10U+1U)*1000,"run lasts AVOID_CLEAR_RUN_MS");}
    CHECK(s.phase==VC_TURN_IN && s.direct_return,"diagonal clear -> direct TURN_IN after the run");
    yaw=0;int steps=0;
    while(s.phase==VC_TURN_IN && steps<200){yaw-=1000;++steps;vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);}
    printf("direct TURN_IN turned %d deg (turn_deg 30, return heading 45)\n",-yaw/1000);
    CHECK(s.phase==VC_RETURN && -yaw>=75000 && -yaw<77000,"direct TURN_IN turns turn_deg plus the 45 deg return heading");
    /* 11. Diagonal look sees the obstacle -> ALIGN, then PASS looks every AVOID_PASS_STEP_MM. */
    armed_in(&s,VC_DIAGONAL,30,0);s.approach_mm=150;t=0;d=0;
    while(s.phase==VC_DIAGONAL && !s.look_side && d<1000000){vc_input_t i=base(t+=10,d+=2000,0);vc_step(&s,&i);}
    for(int k=0;k<3;++k){vc_input_t i=base(t+=70,d,0);i.side_observed=1;i.side_ready=1;i.side_clear=0;i.side_sample_ms=t;vc_step(&s,&i);}
    CHECK(s.phase==VC_ALIGN && !s.direct_return,"diagonal obstacle -> ALIGN");
    yaw=0;
    while(s.phase==VC_ALIGN){yaw-=1000;vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);}
    CHECK(s.phase==VC_PASS,"ALIGN -> PASS");
    int32_t pass0=d;
    while(s.phase==VC_PASS && !s.look_side && d<pass0+1000000){vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);}
    printf("first pass leg %d mm (step %u)\n",(int)((d-pass0)/1000),AVOID_PASS_STEP_MM);
    CHECK(s.look_side && d-pass0<=(int32_t)AVOID_PASS_STEP_MM*1000+1000,"PASS looks after one step");
    for(int k=0;k<3;++k){vc_input_t i=base(t+=70,d,yaw);i.side_observed=1;i.side_ready=1;i.side_clear=1;i.side_sample_ms=t;vc_step(&s,&i);}
    CHECK(s.phase==VC_PASS && s.clear_run,"PASS clear -> short run first");
    while(s.phase==VC_PASS && d<pass0+2000000){vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);}
    CHECK(s.phase==VC_TURN_IN && !s.direct_return,"PASS clear -> normal TURN_IN after the run");
    /* 12. TRACK: a short line-data gap holds still; a long one is a line (data) fault. */
    armed_in(&s,VC_TRACK,0,0);t=1000;d=0;
    {vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);CHECK(s.command==1,"tracking drives");}
    for(int k=0;k<20;++k){vc_input_t i=base(t+=10,d,0);i.line_fresh=0;i.steering=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_TRACK && s.command==0,"200 ms gap: holds still, no fault");
    {vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);CHECK(s.phase==VC_TRACK && s.command==1 && !s.line_gap,"fresh data resumes");}
    for(int k=0;k<20;++k){vc_input_t i=base(t+=10,d,0);i.line_fresh=0;i.steering=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_TRACK,"a new 200 ms gap after fresh data: still no fault");
    for(int k=0;k<10;++k){vc_input_t i=base(t+=10,d,0);i.line_fresh=0;i.steering=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_FAULT && s.fault==VC_LINE && s.fault_cause==VC_LINE_CAUSE_DATA,"gap >= 250 ms -> line fault, cause data");
    /* 13. Sonar gap: holds still up to VC_SONAR_GAP_MS, then a sonar sensor fault. */
    armed_in(&s,VC_TRACK,0,0);t=5000;d=0;
    {vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);CHECK(s.command==1,"tracking drives");}
    for(int k=0;k<90;++k){vc_input_t i=base(t+=10,d,0);i.sensor_fresh=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_TRACK && s.command==0,"900 ms without sonar: holds, no fault");
    {vc_input_t i=base(t+=10,d+=1000,0);vc_step(&s,&i);CHECK(s.phase==VC_TRACK && s.command==1 && !s.sonar_gap,"fresh sonar resumes");}
    for(int k=0;k<101;++k){vc_input_t i=base(t+=10,d,0);i.sensor_fresh=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_FAULT && s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR,"1 s without sonar -> sonar fault");
    /* 14. A decoded turn arriving during a sonar hold is kept. */
    armed_in(&s,VC_TRACK,0,0);t=9000;
    {vc_input_t i=base(t+=10,d,0);i.sensor_fresh=0;i.navigation_event=1;i.nav=1;vc_step(&s,&i);}
    CHECK(s.pending_nav==1,"nav event kept during a sonar hold");
    /* 15. Line found while heading 30 deg right of the original direction:
     * RESTORE arcs left back to it (steering negative = left), then TRACK. */
    armed_in(&s,VC_RETURN,30,200);s.avoid_yaw=0;t=20000;s.entered_ms=t;d=0;yaw=-30000;
    {vc_input_t i=base(t+=10,d+=1000,yaw);i.line_black_mask=1;vc_step(&s,&i);}
    {vc_input_t i=base(t+=10,d+=1000,yaw);i.line_black_mask=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_RESTORE && s.line_found && s.restore_mdeg==30000,"found line -> RESTORE 30 deg back");
    {vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);CHECK(s.command==1 && s.steering<0,"RESTORE arcs left (steering negative)");}
    CHECK(restore_then_track(&s,&t,&d,&yaw),"RESTORE reaches the original heading -> TRACK");
    printf("restored to yaw %d mdeg (original 0)\n",(int)yaw);
    /* 16. No line by the end of RETURN: RESTORE to the original heading, then the spot search. */
    armed_in(&s,VC_RETURN,30,0);s.avoid_yaw=0;t=30000;s.entered_ms=t;d=0;yaw=-30000;
    while(s.phase==VC_RETURN){vc_input_t i=base(t+=10,d+=2000,yaw);vc_step(&s,&i);}
    CHECK(s.phase==VC_RESTORE && !s.line_found,"no line -> RESTORE");
    for(int k=0;k<400 && s.phase==VC_RESTORE;++k){yaw+=yaw<0?1000:0;vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);}
    CHECK(s.phase==VC_REACQUIRE && s.spot_scan && s.search_right_first,"no line: restored -> spot search, right first");
    /* 17. Right bypass (turn_deg<0): the line found heading 30 deg left of the
     * original direction; RESTORE arcs right (steering positive), then TRACK. */
    armed_in(&s,VC_RETURN,-30,200);s.avoid_yaw=0;t=40000;s.entered_ms=t;d=0;yaw=30000;
    {vc_input_t i=base(t+=10,d+=1000,yaw);i.line_black_mask=2;vc_step(&s,&i);}
    {vc_input_t i=base(t+=10,d+=1000,yaw);i.line_black_mask=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_RESTORE && s.line_found && s.restore_mdeg==-30000,"right bypass: found line -> RESTORE 30 deg back");
    {vc_input_t i=base(t+=10,d+=1000,yaw);vc_step(&s,&i);CHECK(s.command==1 && s.steering>0,"RESTORE arcs right (steering positive)");}
    CHECK(restore_then_track(&s,&t,&d,&yaw),"right bypass: RESTORE -> TRACK");
    /* 18. A stale line sample on the way back neither touches nor releases;
     * the return drives on. */
    armed_in(&s,VC_RETURN,30,200);t=50000;s.entered_ms=t;d=0;
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_fresh=0;i.line_black_mask=1;vc_step(&s,&i);
     CHECK(s.phase==VC_RETURN && s.command==1 && !s.nav_touched,"stale touch ignored, still driving");}
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=1;vc_step(&s,&i);}
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_fresh=0;i.line_black_mask=0;vc_step(&s,&i);
     CHECK(s.phase==VC_RETURN && s.nav_touched && !s.line_found,"stale release ignored");}
    {vc_input_t i=base(t+=10,d+=2000,0);i.line_black_mask=0;vc_step(&s,&i);
     CHECK(s.phase==VC_RESTORE && s.line_found,"fresh release hands over");}
    /* 19. A sonar gap on the way back holds still, then resumes; a gap of
     * VC_SONAR_GAP_MS is a sonar fault. */
    armed_in(&s,VC_RETURN,30,200);t=60000;s.entered_ms=t;d=0;
    {vc_input_t i=base(t+=10,d+=2000,0);i.sensor_fresh=0;vc_step(&s,&i);CHECK(s.phase==VC_RETURN && s.command==0 && s.armed,"sonar gap: RETURN holds still");}
    {vc_input_t i=base(t+=10,d,0);vc_step(&s,&i);CHECK(s.phase==VC_RETURN && s.command==1 && !s.sonar_gap,"fresh sonar: RETURN drives on");}
    for(int k=0;k<100;++k){vc_input_t i=base(t+=10,d,0);i.sensor_fresh=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_RETURN && s.command==0,"990 ms sonar gap: still holding");
    {vc_input_t i=base(t+=10,d,0);i.sensor_fresh=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_FAULT && s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR,"1 s sonar gap in RETURN -> sonar fault");
    /* 20. The spot search after a bypass holds through a short line-data gap
     * and faults VC_LINE (line data) once it lasts VC_LINE_GAP_MS. */
    armed_in(&s,VC_REACQUIRE,30,0);s.spot_scan=1;s.search_right_first=1;t=70000;s.entered_ms=t;d=0;
    {vc_input_t i=base(t+=10,d,0);vc_step(&s,&i);CHECK(s.phase==VC_REACQUIRE && s.command==4,"spot search turns right first");}
    for(int k=0;k<25;++k){vc_input_t i=base(t+=10,d,0);i.line_fresh=0;i.steering=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_REACQUIRE && s.command==0,"240 ms into a line gap: search holds still");
    {vc_input_t i=base(t+=10,d,0);i.line_fresh=0;i.steering=0;vc_step(&s,&i);}
    CHECK(s.phase==VC_FAULT && s.fault==VC_LINE && s.fault_cause==VC_LINE_CAUSE_DATA,"250 ms line gap -> line fault, cause data");
    /* 21. The spot search ends only on a sensor actually on black (fresh
     * straddle data with both sensors white is not the line). */
    armed_in(&s,VC_REACQUIRE,-30,0);s.spot_scan=1;t=80000;s.entered_ms=t;d=0;
    {vc_input_t i=base(t+=10,d,0);vc_step(&s,&i);CHECK(s.phase==VC_REACQUIRE && s.command==3,"spot search turns left first (right bypass)");}
    {vc_input_t i=base(t+=10,d,5000);i.line_touch=1;i.line_black_mask=2;vc_step(&s,&i);CHECK(s.line_settling,"touch: settle");}
    {vc_input_t i=base(t+=20,d,5000);i.line_touch=1;i.line_black_mask=2;vc_step(&s,&i);CHECK(s.phase==VC_TRACK,"then track");}
    /* 22. Turning back in: nothing counts as the line for the first
     * AVOID_TURN_IN_LINE_DELAY_MS of the turn's driving (left bypass, lead =
     * right sensor). 25 Nov (8g): the far sensor on black as the turn began,
     * then the lead, handed over at once. The turn drives on meanwhile. */
    armed_in(&s,VC_TURN_IN,30,200);t=100000;s.entered_ms=t;d=0;yaw=0;
    ticks(&s,&t,&d,yaw,1,2);
    CHECK(s.phase==VC_TURN_IN && s.command==1 && s.turn_in_timing && s.turn_in_driven_ms==0,"turn-in drives; the line wait starts");
    {const unsigned early[]={2,3,1,1,0,2,0,3,0};
     for(unsigned k=0;k<sizeof early/sizeof early[0];++k) ticks(&s,&t,&d,yaw,5,early[k]);}
    CHECK(s.phase==VC_TURN_IN && s.command==1 && !s.nav_touched && !s.line_found && s.turn_in_driven_ms==900,
          "far, both, lead, release in the first 900 ms of turning: ignored, still turning");
    ticks(&s,&t,&d,yaw,4,1);
    CHECK(s.phase==VC_TURN_IN && !s.nav_touched && s.turn_in_driven_ms==AVOID_TURN_IN_LINE_DELAY_MS-20U,
          "lead touch up to 20 ms before the wait ends: ignored");
    ticks(&s,&t,&d,yaw,1,1);
    CHECK(s.phase==VC_TURN_IN && s.nav_touched && !s.line_found,"lead touch once the wait is over: latched");
    ticks(&s,&t,&d,yaw,1,0);
    CHECK(s.phase==VC_RESTORE && s.line_found,"then the release hands over (RESTORE)");
    /* 23. Through the real entry: the clear run at the end of the diagonal
     * turns in on a drive tick, and the wait starts there; a hold for the
     * forward check after it does not count, and neither the entry tick nor
     * the first drive tick after the hold looks for the line. */
    armed_in(&s,VC_DIAGONAL,30,0);s.approach_mm=150;t=110000;s.entered_ms=t;d=0;yaw=0;
    while(s.phase==VC_DIAGONAL && !s.look_side && d<1000000){vc_input_t i=base(t+=20,d+=2000,0);vc_step(&s,&i);}
    for(int k=0;k<3;++k){vc_input_t i=base(t+=70,d,0);i.side_observed=1;i.side_ready=1;i.side_clear=1;i.side_sample_ms=t;vc_step(&s,&i);}
    while(s.phase==VC_DIAGONAL && d<2000000){vc_input_t i=base(t+=20,d+=1000,0);i.line_black_mask=3;vc_step(&s,&i);}
    CHECK(s.phase==VC_TURN_IN && s.command==1 && s.turn_in_timing && s.turn_in_driven_ms==0 && !s.line_found,
          "clear run -> TURN_IN drives on its entry tick and starts the wait (both sensors black ignored)");
    for(int k=0;k<50;++k){vc_input_t i=base(t+=20,d,yaw);i.front_clear=0;i.line_black_mask=1;vc_step(&s,&i);}
    CHECK(s.phase==VC_TURN_IN && s.command==0 && !s.nav_touched && s.turn_in_driven_ms==0,"1 s hold for the forward check: stopped, not counted");
    ticks(&s,&t,&d,yaw,1,3);
    CHECK(s.phase==VC_TURN_IN && s.command==1 && !s.line_found && s.turn_in_driven_ms<100U,
          "first drive tick after the hold: both black ignored, the hold not counted");
    /* 23b. Holds after the turn has started (sonar gap, odometry gap, the
     * forward check) do not count either; after the wait the far window works
     * in TURN_IN as in RETURN. */
    armed_in(&s,VC_TURN_IN,30,200);t=120000;s.entered_ms=t;d=0;yaw=0;
    ticks(&s,&t,&d,yaw,10,0);
    CHECK(s.turn_in_driven_ms==180,"10 drive ticks: 180 ms of turning");
    for(int k=0;k<45;++k){vc_input_t i=base(t+=20,d,yaw);i.sensor_fresh=0;i.line_black_mask=2;vc_step(&s,&i);}
    CHECK(s.phase==VC_TURN_IN && s.armed && s.command==0,"900 ms sonar gap: held, no fault");
    for(int k=0;k<7;++k){vc_input_t i=base(t+=20,d,yaw);i.odom_valid=0;i.line_black_mask=2;vc_step(&s,&i);}
    for(int k=0;k<25;++k){vc_input_t i=base(t+=20,d,yaw);i.front_clear=0;i.line_black_mask=2;vc_step(&s,&i);}
    CHECK(s.phase==VC_TURN_IN && s.armed && s.command==0 && s.turn_in_driven_ms==180,"odometry gap and forward hold: held, not counted");
    ticks(&s,&t,&d,yaw,1,2);
    CHECK(s.phase==VC_TURN_IN && s.command==1 && !s.line_found && s.turn_in_driven_ms<300U,
          "1.5 s later, still in the wait: the far sensor on black where the turn began is ignored");
    {unsigned n=0;while(s.turn_in_driven_ms<AVOID_TURN_IN_LINE_DELAY_MS && n<200U){ticks(&s,&t,&d,yaw,1,0);++n;}}
    CHECK(s.phase==VC_TURN_IN && !s.line_found && s.turn_in_driven_ms==AVOID_TURN_IN_LINE_DELAY_MS,"wait over, nothing under the sensors");
    ticks(&s,&t,&d,yaw,1,2);
    CHECK(s.phase==VC_RESTORE && s.line_found,"far sensor alone just after the wait, in TURN_IN -> RESTORE");
    /* 24. A turn that ends sooner: the wait carries on into RETURN. */
    armed_in(&s,VC_TURN_IN,30,200);t=130000;s.entered_ms=t;d=0;yaw=0;
    ticks(&s,&t,&d,yaw,15,0);
    yaw=-45000;ticks(&s,&t,&d,yaw,1,0);
    CHECK(s.phase==VC_RETURN && s.turn_in_timing && s.turn_in_driven_ms==300,"turn done after 300 ms: RETURN keeps the wait");
    ticks(&s,&t,&d,yaw,5,1);ticks(&s,&t,&d,yaw,5,0);
    CHECK(s.phase==VC_RETURN && s.command==1 && !s.nav_touched,"touch and release 320-500 ms in: ignored, straight on");
    ticks(&s,&t,&d,yaw,25,0);
    CHECK(s.phase==VC_RETURN && s.turn_in_driven_ms==AVOID_TURN_IN_LINE_DELAY_MS,"RETURN counts on to the end of the wait");
    ticks(&s,&t,&d,yaw,2,1);ticks(&s,&t,&d,yaw,1,0);
    CHECK(s.phase==VC_RESTORE && s.line_found,"after the wait, touch and release in RETURN hand over");
    /* 25. For AVOID_TURN_IN_FAR_MS after the wait the far sensor alone counts
     * (the lead crossed the line during the wait); later it does not. */
    armed_in(&s,VC_RETURN,30,200);t=140000;s.entered_ms=t;d=0;
    s.turn_in_timing=1;s.turn_in_ms=t;s.turn_in_driven_ms=AVOID_TURN_IN_LINE_DELAY_MS+AVOID_TURN_IN_FAR_MS-40U;
    ticks(&s,&t,&d,0,1,2);
    CHECK(s.phase==VC_RESTORE && s.line_found,"far sensor alone 20 ms before the far window ends -> RESTORE");
    armed_in(&s,VC_RETURN,30,200);t=150000;s.entered_ms=t;d=0;
    s.turn_in_timing=1;s.turn_in_ms=t;s.turn_in_driven_ms=AVOID_TURN_IN_LINE_DELAY_MS+AVOID_TURN_IN_FAR_MS-20U;
    ticks(&s,&t,&d,0,1,2);
    CHECK(s.phase==VC_RETURN && !s.line_found,"far sensor alone once the far window is over: not the line");
    armed_in(&s,VC_RETURN,-30,200);t=160000;s.entered_ms=t;d=0;
    s.turn_in_timing=1;s.turn_in_ms=t;s.turn_in_driven_ms=AVOID_TURN_IN_LINE_DELAY_MS-20U;
    ticks(&s,&t,&d,0,1,1);
    CHECK(s.phase==VC_RESTORE && s.line_found,"right bypass: far (right) sensor alone as the wait ends -> RESTORE");
    /* 26. The wait belongs to one return: TRACK and a stop clear it. */
    yaw=0;CHECK(restore_then_track(&s,&t,&d,&yaw) && !s.turn_in_timing,"back in TRACK: wait cleared");
    s.turn_in_timing=1;vc_stop(&s,0);
    CHECK(!s.turn_in_timing,"STOP clears the wait");
    if(fails) {printf("bypass_return: %d FAILED\n",fails);return 1;}
    puts("Bypass return: diagonal side look (clear run, direct 2x return), align and pass in steps, return until the line is under the robot or the extra distance, RESTORE to the detection heading, TRACK or spot search; no line search for the first second of the turn-in (far sensor alone right after it); line-data and sonar gap holds PASS");
    return 0;
}
