/* Run the actual controller task with synthetic owner snapshots. */
#include "s5_hardware_host.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#define BOARD_TICKS_PER_REV 100
#define BOARD_WHEEL_CIRCUMFERENCE_MM 200
#define BOARD_TRACK_WIDTH_MM 100
#define BOARD_TERRAIN_CALIBRATED 1
#define BOARD_MOTORS_DISABLED 0
#define BOARD_BYPASS_VALIDATED 1 /* synthetic policy fixture only */
ID app_start_task(FP task, PRI priority, UW core, const char *name);
#include "../../app_program/vehicle.c"
#include "../../app_program/subsystem5/s5_scan.h" /* mode 15 runs S5's real acquisition step */
#if !BOARD_LINE_MASK_PIVOT && !BOARD_LINE_EDGE_RIGHT
#error The edge-trace scenarios below ride the right edge (BOARD_LINE_EDGE_RIGHT 1)
#endif
/* Mode 0 also offers MAP frames: valid, invalid letter, stale. */
static unsigned acks,ack_log[4][3],map_calls,map_letter,map_nav,map_step;
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault)
{
    (void)phase;(void)fault;
    if(opcode==4U && acks<4U) {ack_log[acks][0]=seq;ack_log[acks][1]=opcode;ack_log[acks++][2]=result;}
}
INT tm_printf(UB *format, ...) { (void)format;return 0; }
static jmp_buf done;
static app_state_t saved;
static uint32_t clock_ms, iteration;
static unsigned failure_mode, stops, positive_requests;
/* Robot events posted for the dashboard's event log (shared/events.h). */
#define EVENT_LOG 16U
static unsigned event_count,event_id[EVENT_LOG],event_value[EVENT_LOG],event_tick[EVENT_LOG];
void s1_event_post(unsigned id,unsigned value)
{
    if(event_count<EVENT_LOG) {event_id[event_count]=id;event_value[event_count]=value;event_tick[event_count]=iteration;}
    ++event_count;
}
/* Mode 7: the sonar reports an unusable distance from SONAR_GAP_AT on. */
#define SONAR_GAP_AT 8U
#define SONAR_GAP_TICKS (VC_SONAR_GAP_MS/TICK_MS)
/* Mode 10: barcode miss -> retry reverse, with one forward sonar glitch. */
#define BR_MISS_AT 10U
#define BR_GLITCH_AT 25U
#define BR_ITERATIONS 40U
static unsigned reverse_requests, reverse_after_glitch, left_track;
/* A reverse request, then a forward request or a STOP in the same tick
 * (iteration+1 of the last reverse; 0 none). */
static unsigned reverse_tick, after_reverse;
/* Mode 11: S3 starts reading a marker during normal tracking (first pass). */
#define READ_FROM 12U
#define READ_UNTIL 16U
static unsigned read_count, read_max, normal_count, normal_min;
/* Mode 12: one >1.8 g accelerometer sample while armed and tracking.
 * Mode 21: the same impact clipped at +/-2 g (the driver marks it invalid). */
#define SHOCK_AT 9U
static bool shock_mode(void) { return failure_mode==12U || failure_mode==21U; }
static unsigned shock_published, after_shock;
static uint8_t shock_flags, shock_faults; /* S4 result published at SHOCK_AT */
static bool reset_offered;
/* Modes 7, 13..20 and 22..27 START the mission (no lease) so they can run
 * past one second; the others ARM at 30% with a 1000 ms lease. */
static bool starts(void) { return failure_mode==7U || (failure_mode>=13U && failure_mode<=20U) || failure_mode>=22U; }
#define MODE_COUNT 29U
#define TICK_MS 20U /* tk_dly_tsk below */
/* Per-tick record of what reached S2 and what the task published. */
#define LOG_TICKS 300U
enum { SENT_NONE, SENT_FOLLOW, SENT_MOVE, SENT_REVERSE };
static uint8_t sent_kind[LOG_TICKS], sent_cmd[LOG_TICKS], sent_power[LOG_TICKS], stopped[LOG_TICKS];
static int32_t sent_steer[LOG_TICKS];
static uint8_t pub_seen[LOG_TICKS], pub_follow[LOG_TICKS], pub_mask[LOG_TICKS];
static uint8_t pub_phase[LOG_TICKS], pub_armed[LOG_TICKS];
static int16_t pub_steer[LOG_TICKS];
static void log_send(unsigned kind,unsigned command,UINT power,INT steering)
{
    if(iteration>=LOG_TICKS)return;
    sent_kind[iteration]=(uint8_t)kind;sent_cmd[iteration]=(uint8_t)command;
    sent_power[iteration]=(uint8_t)power;sent_steer[iteration]=steering;
}
/* Line under the straddle sensors, as darkness per mille of each sensor's
 * measured white..black range (GP26 right, GP27 left). "Centred" is both
 * WHITE for the mask-pivot follower and the right IR on the line's edge (the
 * set darkness) for edge tracing, so modes 7..13 and 15 drive straight on
 * (steer 0, full power) with either follower. */
#if BOARD_LINE_MASK_PIVOT
#define CENTRED_RIGHT 0
#else
#define CENTRED_RIGHT BOARD_LINE_EDGE_SET
#endif
/* Mode 13: S3 reads a marker (READING, >= 3 elements) while the straddle
 * sensors see right, left, both, neither, then right again. */
#define SR_FROM 12U
#define SR_UNTIL 24U
#define SR_ITERATIONS 28U
static const uint8_t sr_masks[SR_UNTIL-SR_FROM]={1,1,1,2,2,2,3,3,0,0,1,1};
/* Mode 14: the follower, tick by tick from FOLLOW_FROM: what the sensors
 * see, whether the sonar looks aside (a TRACK STOP: front unknown, no
 * obstacle), and what must reach S2 and the published line view. */
#define FOLLOW_FROM 10U
typedef struct {
    int16_t right,left; uint8_t stop;
    uint8_t kind,cmd,power; int16_t steer; uint8_t follow,mask;
} follow_step_t;
#define F_STRAIGHT(r,l,m) {r,l,0,SENT_FOLLOW,S2_MOVE_FORWARD,30,0,0,m}
#define F_PIVOT(r,l,cmd,code,m) {r,l,0,SENT_MOVE,cmd,30,0,code,m}
#define F_STEER(r,l,power,steer,m) {r,l,0,SENT_FOLLOW,S2_MOVE_FORWARD,power,steer,(steer)>0?1:2,m}
#define F_STOP(r,l,m) {r,l,1,SENT_NONE,0,0,0,0,m}
#if BOARD_LINE_MASK_PIVOT
/* Mask-pivot: right IR black pivots right, left IR black pivots left, both
 * or neither drive straight; the current sample only (no timing), so a
 * resume after a STOP acts on the sensors at once. */
static const follow_step_t follow_steps[]={
    F_STRAIGHT(0,0,0),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STRAIGHT(1000,1000,3),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),
    F_STRAIGHT(0,0,0),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),
    F_STRAIGHT(0,0,0),
};
#else
/* Edge tracing (right IR on the right edge, set 500, kp 0.8, max 400): the
 * steer is proportional to the darkness error and the power drops by up to
 * 35% at full steer; the left IR on black pivots left until the right IR is
 * back on the edge; both black is a crossing (straight until the left IR is
 * white); the right IR deep in the line for 250 ms pivots right until it is
 * back at the edge. Masks follow the 750/550 per mille hysteresis. */
#define DEEP_TICKS ((BOARD_LINE_EDGE_DEEP_MS+TICK_MS-1U)/TICK_MS)
static const follow_step_t follow_steps[]={
    F_STRAIGHT(500,0,0),
    F_STEER(750,0,24,200,1),F_STEER(250,0,24,-200,0),
    F_STRAIGHT(500,0,0),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),F_PIVOT(300,0,S2_MOVE_PIVOT_LEFT,4,0),
    F_STEER(600,0,27,80,0),
    F_STRAIGHT(1000,1000,3),F_STRAIGHT(1000,800,3),
    /* Crossing over; the right IR deep in the line: full steer, then a pivot. */
    F_STEER(1000,500,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),F_PIVOT(700,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STRAIGHT(500,0,0),
    F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),
    F_STEER(750,0,24,200,1),
};
_Static_assert(DEEP_TICKS==13U,"12 deep ticks of full steer, then the pivot");
#endif
#define FOLLOW_STEPS (sizeof(follow_steps)/sizeof(follow_steps[0]))
static const follow_step_t *follow_step(unsigned n)
{ return failure_mode==14U && n>=FOLLOW_FROM && n<FOLLOW_FROM+FOLLOW_STEPS?&follow_steps[n-FOLLOW_FROM]:NULL; }
/* Mode 15: forward sonar samples while tracking a centred line, one per
 * tick from OC_FROM (H: one near echo re-read for several ticks, F far,
 * N near 150 mm, D no-echo, C close 40 mm, I invalidated by S5 re-centring)
 * and what TRACK must do (d drive, s hold still, a VC_ACQUIRE): the held
 * echo counts once; a no-echo holds only right after a near echo and is no
 * reading; C and I hold; acquisition only on the third near echo.
 * Each new sample goes through S5's own acquisition step, as its scan task
 * does (subsystem5.c): S5 counts the same echoes, so the third near echo
 * starts SCAN_LEFT and is published already turning to 20 deg (invalid);
 * S5 then settles the servo for 200 ms, so the VC re-reads that status. The
 * VC acquires on S5's stage with no forward echo in hand, and EV_OBSTACLE
 * detected must still carry the 15 cm. I is S5's status invalidated outside
 * its step (re-centring), published as is. */
#define OC_FROM 12U
static const char oc_sample[]="HHHHHFNNDFDCFINNDNNN";
static const char oc_expect[]="ddddddddsddsdsddsaaa";
#define OC_ITERATIONS (OC_FROM+sizeof(oc_sample)-1U)
#define OC_ACQUIRE_AT (OC_FROM+17U) /* the third near echo */
static s5_status_t oc_status;static s5_scan_context_t oc_scan;static bool oc_sampled;
static void oc_s5_publish(s5_status_t *s,char sample)
{
    if(oc_status.state!=S5_MONITOR || (sample=='H' && oc_sampled)) {*s=oc_status;return;}
    oc_sampled=true;
    if(sample!='I') {
        s->sample_angle_deg=s->angle_deg;
        INT target=s5_scan_step(s,&oc_scan);
        if(target!=s->angle_deg) {s->valid=FALSE;s->angle_deg=target;} /* never valid while panning */
    }
    oc_status=*s;
}
/* Mode 16: lifted from LIFT_AT (both line IRs read past BOARD_LIFT_RAW, GP28
 * black): VC_LIFTED after BOARD_LIFT_FAULT_MS (0: never). */
#define LIFT_AT 10U
#define LIFT_TICKS 100U /* 2000 ms */
/* Mode 17: a hump. The front rises 10 deg from HUMP_FROM (the accelerometer
 * tilts with it), level again from HUMP_LEVEL: with BOARD_RAMP_SPEED the
 * smoothed tilt held 400 ms is the way up (EV_HUMP detected), 1.5 deg below
 * its peak the crest (EV_HUMP crest; power capped at BOARD_RAMP_DOWN_POWER),
 * level for 400 ms the end (EV_HUMP level; normal power). */
#define HUMP_FROM 10U
#define HUMP_LEVEL 70U
#define HUMP_ITERATIONS 125U
#define HUMP_COUNTS_PER_TICK 5U /* 10 mm per tick: S4 integrates the height */
static bool scenario_nose_up(unsigned n) { return failure_mode==17U && n>=HUMP_FROM && n<HUMP_LEVEL; }
/* The mask-pivot follower drives the 1.2 m centred (both IRs white): a
 * crossing line (both black, straight on) every 200 mm keeps it within
 * BOARD_LINE_LOST_MM of the line. */
#define HUMP_CROSS_TICKS 20U
_Static_assert(!BOARD_LINE_LOST_MM || HUMP_CROSS_TICKS*HUMP_COUNTS_PER_TICK*2U<BOARD_LINE_LOST_MM,"mode 17 line contact");
static bool hump_cross(unsigned n) { return BOARD_LINE_MASK_PIVOT && failure_mode==17U && n%HUMP_CROSS_TICKS==0U; }
/* Mode 18: two decoded markers (straight on). S3 queues the first turn one
 * tick before its status shows the decode (DEC1_AT, then DEC1_AT+1); the
 * second turn and its decode arrive in the same tick (DEC2_AT). The event log
 * lists each EV_BARCODE attempt before the EV_INSTRUCTION it decoded. */
#define DEC1_AT 10U
#define DEC2_AT 16U
#define DEC_ITERATIONS 20U
static unsigned nav_given; /* iteration whose turn was handed out */
/* Mode 19: as mode 8 (S5 READY at 100 mm: acquisition), then a STOP command
 * at AVOID_STOP_AT ends the avoidance: EV_OBSTACLE back, how 0. */
#define AVOID_STOP_AT 10U
/* Mode 20: S5's status lock is busy (s5_get_status fails) from S5_BUSY_FROM
 * to S5_BUSY_UNTIL, across the START phase change: the last snapshot is
 * reused, but the new sensor mode is requested only once a fresh read
 * succeeds (s5_navigation_set would time out on the busy lock). */
#define S5_BUSY_FROM 7U
#define S5_BUSY_UNTIL 10U
static bool s5_busy(unsigned n) { return failure_mode==20U && n>=S5_BUSY_FROM && n<S5_BUSY_UNTIL; }
static unsigned nav_set_busy, nav_set_after;
/* Modes 22 and 23 run S3's real core (buddy3_core.c) behind its controller
 * API, as subsystem3.c does. A sampler hiccup (a late sample or ADC reply)
 * latches STALE, or SENSOR, with no event; S3 then aborts every barcode
 * capture until the VC sends TRACK again. The VC reads the next, fresh
 * sample with the latch.
 * Mode 22: one STALE latch at LATCH_AT, in the middle of a *B* marker (GP28,
 * narrow 1 tick, wide 3, the robot at 2 mm per tick): the VC re-sends TRACK
 * (marker clear) at once and keeps driving, and the next marker decodes.
 * Mode 23: a turn is pending (F, left: no junction yet) and S3 latches
 * SENSOR again on every sample from LATCH_AT: TRACK (holding the marker) is
 * re-sent at most every VC_S3_REARM_MS, then VC_LINE cause event after
 * VC_S3_LATCH_MS.
 * Mode 24: as mode 10 (synthetic S3), with the status latched STALE from the
 * miss at BR_MISS_AT: no re-send while the re-read brakes; from the reverse
 * on, as mode 23. */
#define LATCH_AT 30U
#define MARKER1_AT 20U
#define MARKER2_AT 80U
#define NAV_AT 20U
#define LATCH_TICKS (VC_S3_LATCH_MS/TICK_MS)
#define REARM_TICKS (VC_S3_REARM_MS/TICK_MS)
#define REVERSE_AT (BR_MISS_AT+BOARD_BARCODE_RETRY_BRAKE_MS/TICK_MS)
/* Mode 22 drives 2 mm a tick centred (both IRs white) to the end. */
_Static_assert(!BOARD_LINE_LOST_MM || (MARKER2_AT+70U)*2U<BOARD_LINE_LOST_MM,"mode 22 line contact");
static bool real_s3(void) { return failure_mode==22U || failure_mode==23U; }
/* Modes 25..27 (START) drive by what reached S2 each tick (kin_step).
 * Mode 25: the right IR touches the line at LL_TOUCH_AT, then the line is
 * gone (both IRs white): BOARD_LINE_LOST_MM later the robot searches on the
 * spot, turning right (the line's side) first, and the right IR finds black
 * LL_FOUND_AFTER ticks into the search: tracking resumes. Mode 26: never
 * found, fault 2 cause 3. Edge tracing has no line-lost distance: the lost
 * line is its own full-turn fault (VC_LINE, cause other). Mode 27: centred
 * throughout with a barcode miss at BR_MISS_AT, as mode 10: the re-read (an
 * 800 mm reverse, then a 950 mm slow pass) never triggers it; the distance
 * runs again from the re-read's end. Mode 28: centred throughout, S3 reads a
 * marker (READING, >= 3 elements) from MR_FROM for 700 mm: the same. */
#define LL_TOUCH_AT 20U
#define LL_FOUND_AFTER 3U
#define MR_FROM 20U
#define MR_UNTIL (MR_FROM+70U)
static bool kinematic(void) { return failure_mode>=25U; }
static unsigned lost_tick; /* first VC_REACQUIRE published after START (0 none) */
static uint32_t kin_counts[2];static int32_t kin_mm[LOG_TICKS]; /* odometry each tick */
/* Forward or a follow 10 mm (a follow's steer splits the wheels), a pivot
 * moves its outer wheel 10 mm, a spin 2 mm a wheel each way (2.3 deg), a
 * reverse 10 mm back, a STOP holds (2 mm a count). */
static void kin_step(void)
{
    if(!kinematic() || iteration>=LOG_TICKS || stopped[iteration])return;
    int32_t l=0,r=0,s=sent_steer[iteration];
    if(sent_kind[iteration]==SENT_FOLLOW) {l=(5*(1000+s)+500)/1000;r=(5*(1000-s)+500)/1000;}
    else if(sent_kind[iteration]==SENT_REVERSE) l=r=-5;
    else if(sent_kind[iteration]==SENT_MOVE) switch(sent_cmd[iteration]) {
        case S2_MOVE_FORWARD: l=r=5;break;
        case S2_MOVE_LEFT: l=-1;r=1;break;
        case S2_MOVE_RIGHT: l=1;r=-1;break;
        case S2_MOVE_PIVOT_RIGHT: l=5;break;
        case S2_MOVE_PIVOT_LEFT: r=5;break;
        default: break;
    }
    kin_counts[0]+=(uint32_t)l;kin_counts[1]+=(uint32_t)r;
}
static s3_core_t s3c;static s3_event_t s3_pending;
static unsigned track_calls,track_tick[16],track_clear[16]; /* TRACK requests */
static bool marker_black(unsigned n)
{
    static const uint16_t chars[3]={0x094U,0x049U,0x094U}; /* *B*, B unmapped */
    const unsigned at[2]={MARKER1_AT,MARKER2_AT};
    for(unsigned m=0;m<2U;++m) {
        if(n<at[m])continue;
        unsigned t=n-at[m];
        for(unsigned e=0;e<29U;++e) {
            unsigned i=e%10U,w=i==9U?1U:(((chars[e/10U]>>(8U-i))&1U)?3U:1U);
            if(t<w)return (e&1U)==0U;
            t-=w;
        }
    }
    return false;
}
static void scenario_dark(unsigned n,int32_t *right,int32_t *left)
{
    const follow_step_t *f=follow_step(n);
    *right=CENTRED_RIGHT;*left=0;
    if(f) {*right=f->right;*left=f->left;}
    else if(failure_mode==13U && n>=SR_FROM && n<SR_UNTIL) {
        *right=(sr_masks[n-SR_FROM]&1U)?1000:0;*left=(sr_masks[n-SR_FROM]&2U)?1000:0;
    }
    else if(hump_cross(n)) *right=*left=1000;
    else if(failure_mode==25U || failure_mode==26U) {
        if(n==LL_TOUCH_AT || (failure_mode==25U && lost_tick && n>=lost_tick+LL_FOUND_AFTER)) *right=1000;
        else if(n>LL_TOUCH_AT) *right=0; /* the line is gone */
    }
}
static bool scenario_lifted(unsigned n) { return failure_mode==16U && n>=LIFT_AT; }
static unsigned scenario_mask(unsigned n)
{
    const follow_step_t *f=follow_step(n);
    if(f)return f->mask;
    if(failure_mode==13U)return n>=SR_FROM && n<SR_UNTIL?sr_masks[n-SR_FROM]:0U;
    if(hump_cross(n))return 3U;
    if(failure_mode==25U || failure_mode==26U) {int32_t r,l;scenario_dark(n,&r,&l);return r==1000?1U:0U;}
    return scenario_lifted(n)?3U:0U; /* otherwise centred */
}
static bool scenario_reading(unsigned n) { return failure_mode==13U && n>=SR_FROM && n<SR_UNTIL; }
static bool scenario_dropout(unsigned n) { const follow_step_t *f=follow_step(n);return f && f->stop; }
static unsigned iterations(void)
{
    switch(failure_mode) {
    case 7U: return SONAR_GAP_AT+SONAR_GAP_TICKS+2U;
    case 10U: return BR_ITERATIONS;
    case 11U: return 20U;
    case 13U: return SR_ITERATIONS;
    case 14U: return FOLLOW_FROM+(unsigned)FOLLOW_STEPS;
    case 15U: return OC_ITERATIONS;
    case 16U: return LIFT_AT+LIFT_TICKS+3U;
    case 17U: return HUMP_ITERATIONS;
    case 18U: return DEC_ITERATIONS;
    case 20U: return 14U;
    case 22U: return MARKER2_AT+70U; /* the decode is 63 ticks after it starts */
    case 23U: return LATCH_AT+LATCH_TICKS+3U;
    case 24U: return REVERSE_AT+LATCH_TICKS+3U;
    case 25U: return 120U;
    case 26U: case 27U: return LOG_TICKS;
    case 28U: return MR_UNTIL+70U;
    default: return 12U;
    }
}

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(failure_mode==10U && iteration>BR_MISS_AT && saved.vehicle.phase!=VC_TRACK)++left_track;
    kin_step();
    if (++iteration >= iterations()) longjmp(done,1);
    clock_ms += TICK_MS;
    return E_OK;
}
void tk_ext_tsk(void) { assert(0); }
void s2_controller_bind(void) {}
ER s3_controller_bind(void) { return E_OK; }
ER s5_controller_bind(void) { return E_OK; }
ER s2_emergency_stop(void)
{
    ++stops;if(iteration<LOG_TICKS)stopped[iteration]=1;
    if(reverse_tick==iteration+1U)++after_reverse;
    return E_OK;
}
/* Positive drive request accounting shared by S2_MOVE_* and the PID follow
 * request (forward with steering). */
static void count_move(s2_move_command_t command,UINT power)
{
    ++positive_requests;
    if(command==S2_MOVE_REVERSE)reverse_tick=iteration+1U;
    else if(reverse_tick==iteration+1U)++after_reverse;
    if(shock_mode() && iteration>SHOCK_AT && command==S2_MOVE_FORWARD && power)++after_shock;
    if(failure_mode==11U && command==S2_MOVE_FORWARD) {
        if(iteration>=READ_FROM && iteration<READ_UNTIL) {++read_count;if(power>read_max)read_max=power;}
        else {++normal_count;if(power<normal_min)normal_min=power;}
    }
    if(command==S2_MOVE_REVERSE) {++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;}
}
ER s2_movement_send(s2_move_command_t command,UINT power)
{
    count_move(command,power);log_send(SENT_MOVE,command,power,0);return E_OK;
}
ER s2_follow_send(UINT power,INT steering)
{
    count_move(S2_MOVE_FORWARD,power);log_send(SENT_FOLLOW,S2_MOVE_FORWARD,power,steering);return E_OK;
}
ER s2_reverse_follow_send(UINT power,INT steering)
{
    ++positive_requests;log_send(SENT_REVERSE,S2_MOVE_REVERSE,power,steering);reverse_tick=iteration+1U;
    ++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;return E_OK;
}
ER s2_motion_get_status(s2_motion_status_t *s)
{ *s=(s2_motion_status_t){0};return E_OK; }
ER s2_encoder_snapshot(uint8_t *p,size_t capacity)
{
    uint32_t stamp=clock_ms;
    if(failure_mode==1U && iteration>=8U)stamp+=100U;
    uint32_t counts=failure_mode==17U?iteration*HUMP_COUNTS_PER_TICK:failure_mode==22U?iteration:0U;
    encoder_snapshot_t e={counts,counts,stamp,ENCODER_SNAPSHOT_VALID};
    if(kinematic()) {
        e.left_counts=kin_counts[0];e.right_counts=kin_counts[1];
        if(iteration<LOG_TICKS)kin_mm[iteration]=(int32_t)(kin_counts[0]+kin_counts[1]-2000U); /* from 1000/1000 */
    }
    assert(encoder_snapshot_encode(&e,p,capacity));
    /* A failed provider deliberately leaves old-looking data behind. */
    return failure_mode==2U && iteration>=8U?E_SYS:E_OK;
}
ER s3_status_get(s3_status_t *s) {
    *s=(s3_status_t){0};
    if(failure_mode>=7U) {s->configured=1;s->sample_time_ms=clock_ms;s->output.steering_allowed=1;}
    if((failure_mode==10U || failure_mode==24U || failure_mode==27U) && iteration>=BR_MISS_AT)s->output.barcode_fail_count=1;
    if(failure_mode==24U && iteration>=BR_MISS_AT)s->output.error=S3_ERROR_STALE;
    if(failure_mode==18U) { /* decode counts one tick after the first turn */
        unsigned ok=(iteration>DEC1_AT?1U:0U)+(iteration>=DEC2_AT?1U:0U);
        s->output.barcode_ok_count=(uint8_t)ok;
        if(ok) {
            s->output.barcode_result=S3_BARCODE_RESULT_DECODED;s->output.barcode_elements=29U;
            s->output.last_symbol=ok==1U?'F':'B';
        }
    }
    /* A marker in progress (bar, gap, bar...): a single crossing line with
     * fewer than 3 elements must not slow the robot (checked at READ_FROM-1). */
    if(failure_mode==11U && iteration>=READ_FROM-1U && iteration<READ_UNTIL) {
        s->output.barcode_result=S3_BARCODE_RESULT_READING;
        s->output.barcode_elements=(uint8_t)(iteration<READ_FROM?2U:3U+iteration-READ_FROM);
    }
    if(failure_mode==28U && iteration>=MR_FROM && iteration<MR_UNTIL) {
        s->output.barcode_result=S3_BARCODE_RESULT_READING;
        s->output.barcode_elements=(uint8_t)(3U+iteration-MR_FROM);
    }
    /* The straddle observer interprets S3's raw scan itself: a complete,
     * fresh sample against the measured GP26/27 endpoints (GP28 digital). */
    if(failure_mode>=7U) {
        static const uint16_t white[S3_SENSOR_COUNT]=BOARD_IR_ADC_WHITE,black[S3_SENSOR_COUNT]=BOARD_IR_ADC_BLACK;
        int32_t dark[2];scenario_dark(iteration,&dark[0],&dark[1]);
        for(unsigned i=0;i<2U;++i) /* rounded up, so S3 normalises back to exactly dark[i] */
            s->sample.raw[i]=(uint16_t)(white[i]+((uint32_t)(black[i]-white[i])*(uint32_t)dark[i]+999U)/1000U);
        s->sample.raw[2]=0U;s->sample.valid_mask=7U;s->sample.time_ms=clock_ms;
        if(scenario_lifted(iteration)) { /* no surface under the IRs, GP28 black */
            s->sample.raw[0]=s->sample.raw[1]=4000U;s->sample.raw[2]=1U;s->output.barcode_black=1;
        }
        if(scenario_reading(iteration)) {
            s->output.barcode_result=S3_BARCODE_RESULT_READING;
            s->output.barcode_elements=(uint8_t)(3U+iteration-SR_FROM);
        }
    }
    if(real_s3()) { /* this sample through S3's own core */
        s->sample.raw[2]=(uint16_t)(marker_black(iteration)?BOARD_BARCODE_DIGITAL_BLACK_LEVEL:
            1U-BOARD_BARCODE_DIGITAL_BLACK_LEVEL);
        if(failure_mode==22U?iteration==LATCH_AT:iteration>=LATCH_AT)
            s3_core_fault(&s3c,failure_mode==22U?S3_ERROR_STALE:S3_ERROR_SENSOR);
        s3_output_t out;s3_core_step(&s3c,&s->sample,&out);
        if(out.event.kind!=S3_EVENT_NONE)s3_pending=out.event;
        s->output=out;s->output.event=(s3_event_t){0};
    }
    return E_OK;
}
ER s5_get_status(s5_status_t *s) {
    if(s5_busy(iteration))return E_TMOUT; /* lock wait timed out: no copy */
    *s=(s5_status_t){0};
    if(failure_mode>=7U) {
        s->state=S5_MONITOR;s->valid=1;s->distance_mm=400;s->time_ms=clock_ms;
        if(iteration>=SONAR_GAP_AT && failure_mode==7U)s->distance_mm=UINT32_MAX;
        if(iteration==BR_GLITCH_AT && failure_mode==10U)s->distance_mm=150; /* one spurious echo */
        if(iteration>=8U && (failure_mode==8U || failure_mode==19U)) {
            s->state=S5_READY;s->distance_mm=100;s->approach_plan_valid=1;s->object_width_valid=1;
            s->planned_rotate_deg=65537;s->planned_approach_mm=65537;s->planned_return_mm=65537;
        }
        if(scenario_dropout(iteration)) {s->angle_deg=s->sample_angle_deg=30;} /* fresh, looking aside */
        if(failure_mode==15U && iteration>=OC_FROM && iteration<OC_ITERATIONS) {
            static uint32_t held_ms;
            switch(oc_sample[iteration-OC_FROM]) {
            case 'H':if(iteration==OC_FROM)held_ms=clock_ms;s->distance_mm=150;s->time_ms=held_ms;break;
            case 'N':s->distance_mm=150;break;
            case 'C':s->distance_mm=40;break;
            case 'D':s->valid=0;s->distance_mm=0;s->echo_result=S5_ECHO_NO_RISE;break;
            case 'I':s->valid=0;s->distance_mm=150;break; /* echo OK, distance unusable */
            default:break;
            }
            oc_s5_publish(s,oc_sample[iteration-OC_FROM]);
        }
    }
    return E_OK;
}
ER s4_sensor_get_record(s4_sensor_cache_t *s)
{
    s4_data_reset(s);s4_data_init_result(s,LSM303_OK);
    lsm303_frame_t frame={0};frame.accel.fresh=frame.accel.valid=1;
    frame.accel.read_time_us=(uint64_t)(clock_ms-(failure_mode==3U?40U:0U))*1000U;
    /* Standing level on the floor with this robot's mounting (the
     * robot_config.h zeros: sensor -Y front, +X left, ~15 deg nose-up), so
     * terrain reads flat and the ramp handling stays idle. */
    frame.accel.raw[0]=-54;frame.accel.raw[1]=-250;frame.accel.raw[2]=909;
    if(scenario_nose_up(iteration)) {frame.accel.raw[1]=-404;frame.accel.raw[2]=852;} /* 10 deg more nose-up */
    frame.accel.overrun=failure_mode==5U;
    if(failure_mode==6U || (failure_mode==12U && iteration==SHOCK_AT))frame.accel.raw[0]=1900;
    if(failure_mode==21U && iteration==SHOCK_AT) { /* what the driver returns for a +2 g left hit */
        frame.accel.raw[0]=2047;frame.accel.valid=0;frame.accel.status=LSM303_DATA_CLIPPED;
    }
    s4_data_update(s,&frame,LSM303_OK);return E_OK;
}

ER s1_command_take(control_message_t *c)
{
    if(failure_mode==0U && iteration==3U && map_step<3U) {
        static const uint8_t frames[3][6]={{0xC1,4,6,1,0,7},{0xC1,4,27,1,0,8},{0xC1,4,1,2,0,9}};
        *c=(control_message_t){.time_ms=map_step==2U?clock_ms-100U:clock_ms};
        for(unsigned i=0;i<6U;++i)c->wire[i]=frames[map_step][i];
        ++map_step;return E_OK;
    }
    if(failure_mode==4U && iteration==9U && !reset_offered) {
        *c=(control_message_t){.time_ms=clock_ms};c->wire[1]=2;
        reset_offered=true;return E_OK;
    }
    if(failure_mode==19U && iteration==AVOID_STOP_AT && reset_offered) {
        *c=(control_message_t){.time_ms=clock_ms};c->wire[1]=0; /* STOP */
        reset_offered=false;return E_OK;
    }
    if(failure_mode>=7U && iteration==7U && !reset_offered) {
        /* START (mission, no lease) or ARM, both at 30%; ARM's lease is 1000 ms. */
        *c=(control_message_t){.time_ms=clock_ms};
        c->wire[1]=starts()?3:1;c->wire[2]=30;c->wire[3]=0xe8;c->wire[4]=3;
        reset_offered=true;return E_OK;
    }
    return E_TMOUT;
}
ER s3_controller_control(s3_mode_t mode,bool clear,UW lease)
{
    (void)lease;
    if(mode==S3_MODE_TRACK && track_calls<16U) {track_tick[track_calls]=iteration;track_clear[track_calls]=clear;}
    if(mode==S3_MODE_TRACK)++track_calls;
    if(real_s3())(void)s3_core_control(&s3c,mode,clear); /* S3 applies it before its next sample */
    return E_OK;
}
ER s3_controller_renew(UW lease) { (void)lease;return E_OK; }
ER s3_controller_barcode_end(void) { if(real_s3())s3_core_barcode_end(&s3c);return E_OK; }
ER s3_controller_set_map(UINT letter,UINT nav)
{
    ++map_calls;map_letter=letter;map_nav=nav;
    return letter>26U || nav>4U || (letter==0U && nav)?E_PAR:E_OK;
}
ER s5_navigation_set(uint32_t id,s5_state_t mode,INT angle)
{
    (void)id;(void)mode;(void)angle;
    if(s5_busy(iteration)) {++nav_set_busy;return E_TMOUT;} /* the same lock */
    if(failure_mode==20U && iteration>=S5_BUSY_UNTIL)++nav_set_after;
    return E_OK;
}
ER s3_event_receive(s3_event_record_t *event)
{
    if(failure_mode==18U && (iteration==DEC1_AT || iteration==DEC2_AT) && nav_given!=iteration) {
        nav_given=iteration;*event=(s3_event_record_t){0};
        event->event.kind=S3_EVENT_NAVIGATION;event->event.command=S3_NAV_STRAIGHT;
        event->event.symbol=iteration==DEC1_AT?'F':'B';event->event.time_ms=clock_ms;
        return E_OK;
    }
    if(failure_mode==23U && iteration==NAV_AT && nav_given!=iteration) { /* F: turn left */
        nav_given=iteration;*event=(s3_event_record_t){0};
        event->event.kind=S3_EVENT_NAVIGATION;event->event.command=S3_NAV_LEFT;
        event->event.symbol='F';event->event.time_ms=clock_ms;
        return E_OK;
    }
    if(real_s3() && s3_pending.kind!=S3_EVENT_NONE) {
        *event=(s3_event_record_t){0};event->event=s3_pending;s3_pending=(s3_event_t){0};
        return E_OK;
    }
    return E_TMOUT;
}
ER app_state_publish(const app_state_t *s)
{
    saved=*s;
    if(kinematic() && !lost_tick && iteration>7U && s->vehicle.phase==VC_REACQUIRE)lost_tick=iteration;
    if(iteration<LOG_TICKS) {
        pub_seen[iteration]=1;pub_follow[iteration]=s->line_follow;pub_mask[iteration]=s->line_mask;
        pub_steer[iteration]=s->line_steer;pub_phase[iteration]=s->vehicle.phase;pub_armed[iteration]=s->vehicle.armed;
    }
    s4_result_t r;
    if(shock_mode() && s4_result_decode(s->s4_result,sizeof(s->s4_result),&r)) {
        if(r.flags&S4_RESULT_SHOCK)++shock_published;
        if(iteration==SHOCK_AT) {shock_flags=r.flags;shock_faults=r.faults;}
    }
    return E_OK;
}
ID app_start_task(FP task,PRI priority,UW core,const char *name)
{ (void)task;(void)priority;(void)core;(void)name;return 1; }
static unsigned codes_seen; /* line_follow values published across the modes */
static unsigned shock_failures; /* modes 12 and 21, reported at the end */
/* (iii) Every tick publishes the follower request that actually reached S2:
 * 1/2 with the steer for a steered follow, 3/4 for a pivot right/left, 0
 * otherwise (straight follow, STOP, reverse, spin), plus the observed mask. */
static void check_published(void)
{
    for(unsigned n=0;n<iterations();++n) {
        unsigned follow=0U;int32_t steer=0;
        if(sent_kind[n]==SENT_FOLLOW && sent_steer[n]) {follow=sent_steer[n]>0?1U:2U;steer=sent_steer[n];}
        else if(sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_RIGHT)follow=3U;
        else if(sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_LEFT)follow=4U;
        assert(pub_seen[n] && pub_follow[n]==follow && pub_steer[n]==steer);
        assert(pub_mask[n]==scenario_mask(n));
        codes_seen|=1U<<pub_follow[n];
    }
}
/* (i) Mode 13: while S3 reads a marker the follower is off, but a single
 * straddle sensor on black steers toward it: right (mask 1) positive, left
 * (mask 2) negative, capped at BOARD_BARCODE_READ_LINE_STEER and never a
 * pivot; both or neither on black is straight. (The Makefile's _read150
 * build sets the cap below BOARD_LINE_STEER_MIN.) */
static void check_read_steer(void)
{
    const int32_t cap=BOARD_BARCODE_READ_LINE_STEER;
    assert(cap>0);
    unsigned right=0,left=0,straight=0;
    for(unsigned n=8U;n<SR_ITERATIONS;++n) {
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW);
        int32_t steer=sent_steer[n];
        if(!scenario_reading(n)) {assert(steer==0 && sent_power[n]==30U);continue;}
#if BOARD_BARCODE_RETRY_DRIVE && BOARD_BARCODE_SLOW_WHILE_READING
        assert(sent_power[n]<30U); /* the marker is read at the slow speed */
#endif
        switch(scenario_mask(n)) {
        case 1U: assert(steer>0 && steer<=cap);++right;break;
        case 2U: assert(steer<0 && steer>=-cap);++left;break;
        default: assert(steer==0);++straight;break;
        }
    }
    assert(right==5U && left==3U && straight==4U);
}
/* (ii) Mode 14: the follower's request every tick, a TRACK STOP inside a
 * touch, and the resume. */
static void check_follower(void)
{
    for(unsigned n=8U;n<iterations();++n) {
        const follow_step_t *f=follow_step(n);
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
        if(!f) {assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW && sent_steer[n]==0 && sent_power[n]==30U);continue;}
        if(f->stop) {assert(stopped[n] && sent_kind[n]==SENT_NONE && pub_mask[n]==f->mask);continue;}
        assert(!stopped[n] && sent_kind[n]==f->kind && sent_cmd[n]==f->cmd);
        assert(sent_power[n]==f->power && sent_steer[n]==f->steer);
        assert(pub_follow[n]==f->follow && pub_mask[n]==f->mask);
    }
}
/* (iv) Modes 22..24: a silent S3 latch while tracking. The START sends
 * TRACK at tick 7 (marker clear); the VC then re-sends it for the latch. */
static void check_latch(void)
{
    unsigned from=failure_mode==24U?REVERSE_AT:LATCH_AT;
    assert(track_calls>=2U && track_tick[0]==7U && track_clear[0]);
    if(failure_mode==22U) {
        /* Once, at once, clearing the marker: S3 re-arms and the next *B*
         * decodes; the robot never stopped or left TRACK. */
        assert(track_calls==2U && track_tick[1]==LATCH_AT && track_clear[1]);
        assert(s3c.barcode_ok_count==1U && s3c.latched_error==S3_ERROR_NONE && saved.barcode==2U);
        for(unsigned n=8U;n<iterations();++n)
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK && !stopped[n] && sent_kind[n]==SENT_FOLLOW);
        assert(saved.vehicle.armed && saved.vehicle.fault==VC_OK);
        return;
    }
    /* From the first tracking tick that sees the latch (mode 24: not while
     * braking), every VC_S3_REARM_MS, holding mode 23's pending turn
     * (no marker clear); VC_S3_LATCH_MS after it, VC_LINE cause event. */
    assert(track_calls==1U+LATCH_TICKS/REARM_TICKS);
    for(unsigned k=1U;k<track_calls;++k)
        assert(track_tick[k]==from+(k-1U)*REARM_TICKS && track_clear[k]==(failure_mode==24U));
    for(unsigned n=8U;n<from+LATCH_TICKS;++n) {
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
        if(failure_mode==23U)assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW);
    }
    assert(!pub_armed[from+LATCH_TICKS] && pub_phase[from+LATCH_TICKS]==VC_FAULT);
    assert(saved.vehicle.fault==VC_LINE && saved.vehicle.fault_cause==VC_LINE_CAUSE_EVENT);
}
/* (v) Modes 25..27: the line lost while tracking. The search starts on the
 * first tick BOARD_LINE_LOST_MM of odometry past the last straddle black (25,
 * 26: LL_TOUCH_AT) or past the barcode re-read's last tick (27). */
static void check_line_lost(void)
{
#if BOARD_LINE_MASK_PIVOT
    unsigned from=LL_TOUCH_AT,due=0;
#if BOARD_BARCODE_RETRY_DRIVE
    if(failure_mode==27U) {
        /* The re-read brakes, reverses, then passes slowly (30% of 30) for
         * longer than BOARD_LINE_LOST_MM: none of it counts. */
        unsigned slow=30U*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U,reverse=0;int32_t back=kin_mm[BR_MISS_AT];
        for(unsigned n=BR_MISS_AT;n<iterations();++n) {
            if(sent_kind[n]==SENT_REVERSE)++reverse;
            if(kin_mm[n]<back)back=kin_mm[n];
            if(sent_kind[n]==SENT_FOLLOW && sent_power[n]==slow)from=n;
        }
        assert(reverse>0U && from>BR_MISS_AT && kin_mm[from]-back>(int32_t)BOARD_LINE_LOST_MM);
    }
#endif
    if(failure_mode==28U) { /* driven on through the 700 mm read, the follower off */
        from=MR_UNTIL-1U;
        assert(kin_mm[from]-kin_mm[MR_FROM]>(int32_t)BOARD_LINE_LOST_MM);
        for(unsigned n=MR_FROM;n<MR_UNTIL;++n)assert(sent_kind[n]==SENT_FOLLOW && !sent_steer[n]);
    }
    for(unsigned n=from+1U;n<iterations() && !due;++n) if(kin_mm[n]-kin_mm[from]>=(int32_t)BOARD_LINE_LOST_MM)due=n;
    assert(due && lost_tick==due);
    for(unsigned n=9U;n<due;++n)assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
    /* Stop, then spin slowly toward the line's side first: right after the
     * right IR's touch, left when no single IR has touched (modes 27, 28). */
    assert(pub_phase[due]==VC_REACQUIRE && stopped[due]);
    assert(sent_kind[due+1U]==SENT_MOVE && sent_power[due+1U]==15U &&
           sent_cmd[due+1U]==(failure_mode>=27U?S2_MOVE_LEFT:S2_MOVE_RIGHT));
    if(failure_mode==25U) {
        /* The right IR on black: the spin settles one tick, then tracking
         * resumes and the follower pivots toward it. */
        unsigned found=due+LL_FOUND_AFTER;bool pivot=false;
        for(unsigned n=due+1U;n<=found;++n)
            assert(pub_phase[n]==VC_REACQUIRE && sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_RIGHT && sent_power[n]==15U);
        for(unsigned n=found+1U;n<iterations();++n) {
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
            pivot=pivot || (sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_RIGHT);
        }
        assert(pivot && saved.vehicle.fault==VC_OK);
    } else if(failure_mode==26U) {
        /* Never found: the bounded sweep both ways at 15%, then fault 2 cause 3. */
        unsigned fault=0,left=0,right=0;
        for(unsigned n=due+1U;n<iterations() && !fault;++n) {
            if(!pub_armed[n]) {fault=n;break;}
            assert(pub_phase[n]==VC_REACQUIRE);
            if(stopped[n])continue;
            assert(sent_kind[n]==SENT_MOVE && sent_power[n]==15U &&
                   (sent_cmd[n]==S2_MOVE_LEFT || sent_cmd[n]==S2_MOVE_RIGHT));
            if(sent_cmd[n]==S2_MOVE_LEFT)++left; else ++right;
        }
        assert(fault && fault-due<=4000U/TICK_MS+1U && pub_phase[fault]==VC_FAULT && left && right);
        assert(saved.vehicle.fault==VC_LINE && saved.vehicle.fault_cause==VC_LINE_CAUSE_NOT_FOUND);
    }
#else
    /* Edge tracing has no line-lost distance (its IR rides the edge, below
     * black): never a search. A lost line is its own full turn with both IRs
     * white (VC_LINE, cause other); the barcode modes track on. */
    assert(!lost_tick);
    if(failure_mode>=27U) for(unsigned n=9U;n<iterations();++n)assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
    else assert(!saved.vehicle.armed && saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_LINE &&
                saved.vehicle.fault_cause==VC_LINE_CAUSE_OTHER);
#endif
}
int main(void)
{
    for(failure_mode=0;failure_mode<MODE_COUNT;++failure_mode) {
        memset(sent_kind,0,sizeof(sent_kind));memset(sent_cmd,0,sizeof(sent_cmd));
        memset(sent_power,0,sizeof(sent_power));memset(sent_steer,0,sizeof(sent_steer));
        memset(stopped,0,sizeof(stopped));memset(pub_seen,0,sizeof(pub_seen));
        assert(iterations()<=LOG_TICKS);
        iteration=stops=positive_requests=0;clock_ms=100;reset_offered=false;
        shock_published=after_shock=0;shock_flags=shock_faults=0;event_count=0;nav_given=0;
        reverse_requests=reverse_after_glitch=left_track=0;saved=(app_state_t){0};
        reverse_tick=after_reverse=0;nav_set_busy=nav_set_after=0;
        read_count=read_max=normal_count=0;normal_min=~0U;
        track_calls=0;s3_pending=(s3_event_t){0};
        lost_tick=0;kin_counts[0]=kin_counts[1]=1000U;memset(kin_mm,0,sizeof(kin_mm));
        if(real_s3()) { /* configured as app_main.c configures S3 */
            s3_config_t c;s3_config_defaults(&c);robot_ir_apply_layout(&c);
            assert(robot_ir_apply_adc_calibration(&c) && s3_core_init(&s3c,&c));
        }
        if(setjmp(done)==0)vehicle_task(0,NULL);
        s4_result_t result={0};assert(s4_result_decode(saved.s4_result,sizeof(saved.s4_result),&result));
        if(failure_mode==0)assert((result.flags&S4_RESULT_TILT) && saved.odometry_valid);
        /* MAP: accepted while disarmed, invalid rejected, stale never applied. */
        if(failure_mode==0)assert(acks==3U && map_calls==2U && map_letter==27U && map_nav==1U &&
            ack_log[0][0]==7U && ack_log[0][2]==0U && ack_log[1][0]==8U && ack_log[1][2]==1U &&
            ack_log[2][0]==9U && ack_log[2][2]==3U && !saved.vehicle.armed);
        if(failure_mode==1 || failure_mode==2)
            assert(!(result.flags&S4_RESULT_TILT) && !saved.odometry_valid &&
                   (result.faults&EDR_FAULT_ENCODER));
        if(failure_mode==3)assert(!(result.flags&S4_RESULT_TILT) && saved.odometry_valid &&
                                  (result.faults&EDR_FAULT_IMU));
        if(failure_mode==4)assert(reset_offered && !result.maximum_mm);
        if(failure_mode==5)assert(!(result.flags&S4_RESULT_TILT) && saved.odometry_valid &&
                                  (result.faults&EDR_FAULT_IMU));
        /* An unclipped impact is reported as a shock, not as an IMU fault. */
        if(failure_mode==6)assert((result.flags&S4_RESULT_SHOCK) &&
                                  !(result.faults&EDR_FAULT_IMU) && !(result.flags&S4_RESULT_TILT));
        assert(stops);
        if(failure_mode>=9U && failure_mode!=19U)assert(positive_requests>0);
        else assert(!positive_requests);
        if(failure_mode==7U) {
            /* No usable sonar sample: tracking holds still (armed, stopped)
             * for VC_SONAR_GAP_MS, then a sonar sensor fault. */
            for(unsigned n=SONAR_GAP_AT;n<SONAR_GAP_AT+SONAR_GAP_TICKS;++n)
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE);
            assert(!pub_armed[SONAR_GAP_AT+SONAR_GAP_TICKS] && pub_phase[SONAR_GAP_AT+SONAR_GAP_TICKS]==VC_FAULT);
            assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SENSOR &&
                   saved.vehicle.fault_cause==VC_CAUSE_SONAR);
        }
        if(shock_mode()) {
            /* The shock is always detected and published (dashboard impact),
             * and counted once so telemetry sends it even after it clears.
             * A clipped sample (mode 21) is the same impact, and it is also
             * an IMU fault; an unclipped one (mode 12) is not. Neither ever
             * gives tilt. Checked without aborting, so the later modes still run. */
            bool clipped=failure_mode==21U;
            bool ok=shock_published==1U && saved.shock_count==1U && (shock_flags&S4_RESULT_SHOCK) &&
                !(shock_flags&S4_RESULT_TILT) && ((shock_faults&EDR_FAULT_IMU)!=0U)==clipped;
#if BOARD_SHOCK_STOP
            ok=ok && !saved.vehicle.armed && saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SHOCK &&
                !after_shock;
#else
            /* Not a stop: tracking continues, and the one-sample tilt gap is
             * inside the grace, so no VC_SENSOR either. */
            ok=ok && saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && saved.vehicle.fault==VC_OK &&
                after_shock>0U;
#endif
            if(!ok) {
                ++shock_failures;
                printf("FAIL %s shock while tracking: published %u, counted %u, flags %u, faults %u, armed %u, phase %u, fault %u, driving after it %u\n",
                       clipped?"clipped":"unclipped",shock_published,(unsigned)saved.shock_count,
                       (unsigned)shock_flags,(unsigned)shock_faults,(unsigned)saved.vehicle.armed,
                       (unsigned)saved.vehicle.phase,(unsigned)saved.vehicle.fault,after_shock);
            }
        }
#if BOARD_BARCODE_RETRY_DRIVE
        /* The glitch arrives while reversing: TRACK holds and the reverse continues. */
        if(failure_mode==10U)assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && !left_track &&
            reverse_requests>reverse_after_glitch && reverse_after_glitch>=BR_ITERATIONS-BR_GLITCH_AT-2U);
#endif
        /* Nothing overrides a reverse request in its tick: no forward request
         * (the policy's TRACK command) and no STOP after it. */
        assert(!after_reverse);
        /* Event log: one EV_BARCODE for the failed first read (attempt 1, no
         * letter), one EV_OBSTACLE with its distance when an obstacle is
         * acquired (mode 8: S5 already READY at 100 mm); none else (a lost
         * line, modes 25 and 26, posts nothing). */
        if(failure_mode==8U)
            assert(event_count==1U && event_id[0]==EV_OBSTACLE && event_tick[0]==8U &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,10U) && saved.vehicle.phase==VC_ACQUIRE);
        else if(failure_mode==10U || failure_mode==27U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,0U,0U) &&
                   event_tick[0]==BR_MISS_AT);
        else if(failure_mode==15U) {
            /* S5 published the third echo as it really does (SCAN_LEFT,
             * turning to 20 deg, invalid); the event has its forward range. */
            assert(oc_status.state==S5_SCAN_LEFT && !oc_status.valid && oc_status.angle_deg==S5_SCAN_STEP_DEG &&
                   oc_status.sample_angle_deg==0 && oc_status.obstacle_distance_mm==150U);
            assert(event_count==1U && event_id[0]==EV_OBSTACLE && event_tick[0]==OC_ACQUIRE_AT &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U) && pub_phase[event_tick[0]]==VC_ACQUIRE &&
                   pub_phase[event_tick[0]-1U]==VC_TRACK);
        }
        else if(failure_mode==19U)
            /* Acquired at tick 8, then STOP: the avoidance ended (how 0, after 0 s). */
            assert(event_count==2U && event_id[0]==EV_OBSTACLE && event_tick[0]==8U &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,10U) &&
                   event_id[1]==EV_OBSTACLE && event_tick[1]==AVOID_STOP_AT &&
                   event_value[1]==ev_obstacle(EV_OBSTACLE_BACK,0U,0U) && !saved.vehicle.armed);
        else if(failure_mode==18U) {
            /* Each attempt (1: the normal pass, 29 elements, F/B) before its instruction. */
            unsigned n=0;
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK);
#if BOARD_BARCODE_RETRY_DRIVE
            assert(event_count==4U);
            assert(event_id[n]==EV_BARCODE && event_value[n]==ev_barcode(1U,29U,6U) && event_tick[n]==DEC1_AT+1U);++n;
#else
            assert(event_count==2U);
#endif
            assert(event_id[n]==EV_INSTRUCTION && event_value[n]==ev_instruction(6U,S3_NAV_STRAIGHT) &&
                   event_tick[n]==DEC1_AT+1U);++n;
#if BOARD_BARCODE_RETRY_DRIVE
            assert(event_id[n]==EV_BARCODE && event_value[n]==ev_barcode(1U,29U,2U) && event_tick[n]==DEC2_AT);++n;
#endif
            assert(event_id[n]==EV_INSTRUCTION && event_value[n]==ev_instruction(2U,S3_NAV_STRAIGHT) &&
                   event_tick[n]==DEC2_AT);
        }
        else if(failure_mode==23U)
            assert(event_count==1U && event_id[0]==EV_INSTRUCTION && event_value[0]==ev_instruction(6U,S3_NAV_LEFT));
#if BOARD_BARCODE_RETRY_DRIVE
        /* Mode 22: only the second marker is read (the normal pass, 29
         * elements, B), 30 mm after its last edge. Mode 24: the miss. */
        else if(failure_mode==22U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,29U,2U) &&
                   event_tick[0]==MARKER2_AT+63U);
        else if(failure_mode==24U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,0U,0U) &&
                   event_tick[0]==BR_MISS_AT);
#endif
#if BOARD_RAMP_SPEED
        else if(failure_mode==17U) {
            assert(event_count==3U && event_id[0]==EV_HUMP && event_id[1]==EV_HUMP && event_id[2]==EV_HUMP);
            assert((event_value[0]&7U)==EV_HUMP_DETECTED && (event_value[1]&7U)==EV_HUMP_CREST &&
                   (event_value[2]&7U)==EV_HUMP_LEVEL);
            /* Detected only after 400 ms of tilt; a ~10 deg climb is reported in 0.1 deg. */
            assert(event_tick[0]>=HUMP_FROM+BOARD_RAMP_HOLD_MS/TICK_MS && event_tick[0]<HUMP_LEVEL);
            assert((event_value[0]>>3U)>=(unsigned)BOARD_RAMP_TILT_CDEG/10U && (event_value[0]>>3U)<=100U);
            assert(event_tick[1]>=HUMP_LEVEL && event_tick[2]>event_tick[1]+BOARD_RAMP_LEVEL_MS/TICK_MS &&
                   event_tick[2]<HUMP_ITERATIONS);
            /* Heights in 2 mm (the robot climbs 10 deg at 10 mm per tick, so
             * ~104 mm by HUMP_LEVEL; S4 sees the smoothed, lagging tilt): the
             * crest reports this hump's height so far (S4 has not completed
             * it), the level its highest point. */
            unsigned crest=event_value[1]>>3U,level=event_value[2]>>3U;
            assert(crest>=20U && crest<=level && level<=60U);
        }
#endif
        else assert(event_count==0U);
        if(failure_mode==11U) {
            unsigned slow=30U;
#if BOARD_BARCODE_RETRY_DRIVE && BOARD_BARCODE_SLOW_WHILE_READING
            /* A marker started at full speed is read on at the re-read speed. */
            slow=30U*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U;
            if(slow<BOARD_BARCODE_RETRY_MIN_POWER)slow=BOARD_BARCODE_RETRY_MIN_POWER;
            assert(slow<30U);
#endif
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK);
            assert(read_count==READ_UNTIL-READ_FROM && read_max==slow);
            assert(normal_count>=6U && normal_min==30U); /* full speed before and after */
        }
        if(failure_mode==20U)
            /* No request on the busy lock (no VC_TRANSPORT); sent once a read succeeds. */
            assert(!nav_set_busy && nav_set_after>0U && saved.vehicle.armed &&
                   saved.vehicle.phase==VC_TRACK && saved.vehicle.fault==VC_OK);
        if(failure_mode>=7U)check_published();
        if(failure_mode==13U)check_read_steer();
        if(failure_mode==14U)check_follower();
        if(failure_mode>=22U && failure_mode<=24U)check_latch();
        if(kinematic())check_line_lost();
        if(failure_mode==15U) {
            _Static_assert(sizeof(oc_expect)==sizeof(oc_sample),"mode 15 table");
            for(unsigned n=10U;n<OC_ITERATIONS;++n) {
                char e=n<OC_FROM?'d':oc_expect[n-OC_FROM];
                assert(pub_armed[n]);
                if(e=='a')assert(pub_phase[n]==VC_ACQUIRE);
                else if(e=='s')assert(pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE);
                else assert(pub_phase[n]==VC_TRACK && !stopped[n] && sent_kind[n]==SENT_FOLLOW);
            }
        }
        if(failure_mode==17U) {
            /* The policy caps a slope (S4 terrain ascent/descent) at 20%;
             * BOARD_RAMP_UP_POWER 0 adds no cap on the way up; from the crest
             * to the level the ramp caps at BOARD_RAMP_DOWN_POWER; level
             * again: normal power. */
            unsigned slope_ticks=0;
            for(unsigned n=8U;n<HUMP_ITERATIONS;++n) {
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && !sent_steer[n]);
#if BOARD_RAMP_SPEED
                if(n>=event_tick[1] && n<event_tick[2]) {assert(sent_power[n]==BOARD_RAMP_DOWN_POWER);continue;}
#endif
                assert(sent_power[n]==30U || sent_power[n]==20U);
                if(sent_power[n]==20U) {++slope_ticks;assert(n>=HUMP_FROM);}
            }
            assert(slope_ticks>0U && sent_power[HUMP_ITERATIONS-1U]==30U);
        }
        if(failure_mode==16U) {
            /* Lifted: still driving (straight, both IRs read black) until it
             * has lasted BOARD_LIFT_FAULT_MS, then VC_LIFTED with a STOP. */
            for(unsigned n=8U;n<LIFT_AT+LIFT_TICKS;++n)
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && !sent_steer[n] && sent_power[n]==30U);
#if BOARD_LIFT_FAULT_MS
            _Static_assert(BOARD_LIFT_FAULT_MS==LIFT_TICKS*TICK_MS,"mode 16 timing");
            for(unsigned n=LIFT_AT+LIFT_TICKS;n<iterations();++n)
                assert(!pub_armed[n] && pub_phase[n]==VC_FAULT && stopped[n] && sent_kind[n]==SENT_NONE);
            assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_LIFTED && !saved.vehicle.fault_cause);
#else
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK); /* lift fault off */
#endif
        }
    }
    assert(codes_seen==0x1FU); /* straight, steer right/left and pivot right/left all published */
    puts("Straddle barcode read: one straddle sensor on black steers toward it (capped, no pivot), both/neither straight PASS");
#if BOARD_LINE_MASK_PIVOT
    puts("Mask-pivot follower: right/left IR black pivots that way at once, both/neither straight; a TRACK STOP resumes on the current sample PASS");
#else
    puts("Edge tracing: proportional steer with power drop, far-side pivot, crossing straight, deep-edge pivot after 250 ms PASS");
#endif
    puts("Straddle app_state line_follow/line_mask/line_steer match the S2 request every tick PASS");
    puts("Obstacle while tracking: re-reads count once, no-echo is no reading, 5 cm/invalidated/no-echo-after-near hold; 3 consecutive new near echoes acquire PASS");
    puts("Obstacle detected through S5's real step (third echo published turning, invalid): EV_OBSTACLE carries its 15 cm PASS");
    puts("Sonar gap: tracking holds still, a sonar fault only after VC_SONAR_GAP_MS PASS");
    puts("S5 lock busy across a phase change: the last snapshot is reused, the sensor mode is requested after a fresh read PASS");
    puts("Avoidance ended by a STOP: EV_OBSTACLE back with how 0 PASS");
    printf("Lifted (no surface, GP28 black): %s PASS\n",
           BOARD_LIFT_FAULT_MS?"VC_LIFTED after BOARD_LIFT_FAULT_MS":"no fault (BOARD_LIFT_FAULT_MS 0)");
    puts("Event log: EV_BARCODE for a failed first read, EV_OBSTACLE on acquisition, each EV_BARCODE attempt before its EV_INSTRUCTION PASS");
#if BOARD_RAMP_SPEED
    puts("Hump: EV_HUMP detected after the 400 ms hold, crest caps the power to BOARD_RAMP_DOWN_POWER (this hump's height so far), level restores it (its height) PASS");
#else
    puts("Hump with BOARD_RAMP_SPEED 0: no events, no power cap PASS");
#endif
    puts("INT-S4-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS");
    puts("Barcode retry: a one-sample forward sonar glitch does not abort the reverse re-read; no forward request or STOP follows a reverse in its tick PASS");
    puts("Barcode first pass: S3 READING caps the forward power to the re-read speed PASS");
    puts("MAP command: applied in any phase, invalid rejected, stale ACK 3 PASS");
    puts("Silent S3 latch (STALE/SENSOR) while tracking: TRACK re-sent at once, at most every 100 ms, keeping a pending turn's hold; the robot drives on and the next marker decodes; a latch still returning after 1 s is VC_LINE cause event; no re-send while a re-read brakes PASS");
#if BOARD_LINE_MASK_PIVOT
    printf("Line lost while tracking: %u mm with both IRs white spot-searches toward the line's side; found -> tracking, never -> fault 2 cause 3; a barcode read or re-read never counts PASS\n",
           (unsigned)BOARD_LINE_LOST_MM);
#else
    puts("Line lost with edge tracing: no line-lost distance, its own full-turn VC_LINE PASS");
#endif
    if(shock_failures) {
        puts("FAIL Shock while tracking: a >1.8 g or clipped (+/-2 g) sample is not detected/published/counted in this build");
        return 1;
    }
    printf("Shock while tracking, unclipped and clipped at +/-2 g (also an IMU fault): published, counted, %s PASS\n",
           BOARD_SHOCK_STOP?"stops the run (VC_SHOCK)":"run continues (BOARD_SHOCK_STOP 0)");
    return 0;
}
