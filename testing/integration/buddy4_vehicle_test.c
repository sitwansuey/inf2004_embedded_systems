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
/* Mode 12: one >1.8 g accelerometer sample while armed and tracking. */
#define SHOCK_AT 9U
static unsigned shock_published, after_shock;
static bool reset_offered;
/* Modes 7 and 13..20 START the mission (no lease) so they can run past one
 * second; the others ARM at 30% with a 1000 ms lease. */
static bool starts(void) { return failure_mode==7U || failure_mode>=13U; }
#define MODE_COUNT 21U
#define TICK_MS 20U /* tk_dly_tsk below */
/* Per-tick record of what reached S2 and what the task published. */
#define LOG_TICKS 128U
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
 * reading; C and I hold; acquisition only on the third near echo. */
#define OC_FROM 12U
static const char oc_sample[]="HHHHHFNNDFDCFINNDNNN";
static const char oc_expect[]="ddddddddsddsdsddsaaa";
#define OC_ITERATIONS (OC_FROM+sizeof(oc_sample)-1U)
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
static void scenario_dark(unsigned n,int32_t *right,int32_t *left)
{
    const follow_step_t *f=follow_step(n);
    *right=CENTRED_RIGHT;*left=0;
    if(f) {*right=f->right;*left=f->left;}
    else if(failure_mode==13U && n>=SR_FROM && n<SR_UNTIL) {
        *right=(sr_masks[n-SR_FROM]&1U)?1000:0;*left=(sr_masks[n-SR_FROM]&2U)?1000:0;
    }
}
static bool scenario_lifted(unsigned n) { return failure_mode==16U && n>=LIFT_AT; }
static unsigned scenario_mask(unsigned n)
{
    const follow_step_t *f=follow_step(n);
    if(f)return f->mask;
    if(failure_mode==13U)return n>=SR_FROM && n<SR_UNTIL?sr_masks[n-SR_FROM]:0U;
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
    default: return 12U;
    }
}

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(failure_mode==10U && iteration>BR_MISS_AT && saved.vehicle.phase!=VC_TRACK)++left_track;
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
    if(failure_mode==12U && iteration>SHOCK_AT && command==S2_MOVE_FORWARD && power)++after_shock;
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
    uint32_t counts=failure_mode==17U?iteration*HUMP_COUNTS_PER_TICK:0U;
    encoder_snapshot_t e={counts,counts,stamp,ENCODER_SNAPSHOT_VALID};
    assert(encoder_snapshot_encode(&e,p,capacity));
    /* A failed provider deliberately leaves old-looking data behind. */
    return failure_mode==2U && iteration>=8U?E_SYS:E_OK;
}
ER s3_status_get(s3_status_t *s) {
    *s=(s3_status_t){0};
    if(failure_mode>=7U) {s->configured=1;s->sample_time_ms=clock_ms;s->output.steering_allowed=1;}
    if(failure_mode==10U && iteration>=BR_MISS_AT)s->output.barcode_fail_count=1;
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
{ (void)mode;(void)clear;(void)lease;return E_OK; }
ER s3_controller_renew(UW lease) { (void)lease;return E_OK; }
ER s3_controller_barcode_end(void) { return E_OK; }
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
    return E_TMOUT;
}
ER app_state_publish(const app_state_t *s)
{
    saved=*s;
    if(iteration<LOG_TICKS) {
        pub_seen[iteration]=1;pub_follow[iteration]=s->line_follow;pub_mask[iteration]=s->line_mask;
        pub_steer[iteration]=s->line_steer;pub_phase[iteration]=s->vehicle.phase;pub_armed[iteration]=s->vehicle.armed;
    }
    s4_result_t r;
    if(failure_mode==12U && s4_result_decode(s->s4_result,sizeof(s->s4_result),&r) &&
       (r.flags&S4_RESULT_SHOCK))++shock_published;
    return E_OK;
}
ID app_start_task(FP task,PRI priority,UW core,const char *name)
{ (void)task;(void)priority;(void)core;(void)name;return 1; }
static unsigned codes_seen; /* line_follow values published across the modes */
static unsigned shock_failures; /* mode 12, reported at the end */
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
int main(void)
{
    for(failure_mode=0;failure_mode<MODE_COUNT;++failure_mode) {
        memset(sent_kind,0,sizeof(sent_kind));memset(sent_cmd,0,sizeof(sent_cmd));
        memset(sent_power,0,sizeof(sent_power));memset(sent_steer,0,sizeof(sent_steer));
        memset(stopped,0,sizeof(stopped));memset(pub_seen,0,sizeof(pub_seen));
        assert(iterations()<=LOG_TICKS);
        iteration=stops=positive_requests=0;clock_ms=100;reset_offered=false;
        shock_published=after_shock=0;event_count=0;nav_given=0;
        reverse_requests=reverse_after_glitch=left_track=0;saved=(app_state_t){0};
        reverse_tick=after_reverse=0;nav_set_busy=nav_set_after=0;
        read_count=read_max=normal_count=0;normal_min=~0U;
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
        if(failure_mode==5)assert(!(result.flags&S4_RESULT_TILT) && saved.odometry_valid);
        if(failure_mode==6)assert((result.flags&S4_RESULT_SHOCK) &&
                                  (result.faults&EDR_FAULT_IMU) && !(result.flags&S4_RESULT_TILT));
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
        if(failure_mode==12U) {
            /* The shock is always detected and published (dashboard impact),
             * and counted once so telemetry sends it even after it clears.
             * Checked without aborting, so the later modes still run. */
            bool ok=shock_published==1U && saved.shock_count==1U;
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
                printf("FAIL shock while tracking: published %u, counted %u, armed %u, phase %u, fault %u, driving after it %u\n",
                       shock_published,(unsigned)saved.shock_count,(unsigned)saved.vehicle.armed,
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
         * acquired (mode 8: S5 already READY at 100 mm); none else. */
        if(failure_mode==8U)
            assert(event_count==1U && event_id[0]==EV_OBSTACLE && event_tick[0]==8U &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,10U) && saved.vehicle.phase==VC_ACQUIRE);
        else if(failure_mode==10U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,0U,0U) &&
                   event_tick[0]==BR_MISS_AT);
        else if(failure_mode==15U)
            assert(event_count==1U && event_id[0]==EV_OBSTACLE &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U) && pub_phase[event_tick[0]]==VC_ACQUIRE &&
                   pub_phase[event_tick[0]-1U]==VC_TRACK);
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
    if(shock_failures) {
        puts("FAIL Shock while tracking: a >1.8 g sample is not detected/published/counted in this build");
        return 1;
    }
    printf("Shock while tracking: published, %s PASS\n",
           BOARD_SHOCK_STOP?"stops the run (VC_SHOCK)":"run continues (BOARD_SHOCK_STOP 0)");
    return 0;
}
