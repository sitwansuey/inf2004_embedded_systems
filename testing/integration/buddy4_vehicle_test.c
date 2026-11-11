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
/* Mode 0 also offers MAP frames: valid, invalid letter, stale. */
static unsigned acks,ack_log[4][3],map_calls,map_letter,map_nav,map_step;
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault)
{
    (void)phase;(void)fault;
    if(opcode==4U && acks<4U) {ack_log[acks][0]=seq;ack_log[acks][1]=opcode;ack_log[acks++][2]=result;}
}
static jmp_buf done;
static app_state_t saved;
static uint32_t clock_ms, iteration;
static unsigned failure_mode, stops, positive_requests;
/* Mode 10: barcode miss -> retry reverse, with one forward sonar glitch. */
#define BR_MISS_AT 10U
#define BR_GLITCH_AT 25U
#define BR_ITERATIONS 40U
static unsigned reverse_requests, reverse_after_glitch, left_track;
/* Mode 11: S3 starts reading a marker during normal tracking (first pass). */
#define READ_FROM 12U
#define READ_UNTIL 16U
static unsigned read_count, read_max, normal_count, normal_min;
/* Mode 12: one >1.8 g accelerometer sample while armed and tracking. */
#define SHOCK_AT 9U
static unsigned shock_published, after_shock;
static bool reset_offered;
/* Modes 13..16 exist only in the straddle build (BOARD_LINE_STRADDLE): they
 * feed real GP26/27 ADC samples to the straddle course and START the mission
 * (no lease), see the scenario block below. */
#define STRADDLE_FIRST 13U
#if BOARD_LINE_STRADDLE
#define MODE_COUNT 17U
#else
#define MODE_COUNT 13U
#endif
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
#if BOARD_LINE_STRADDLE
#define TICK_MS 20U /* tk_dly_tsk below */
/* Mode 13: S3 reads a marker (READING, >= 3 elements) while the straddle
 * sensors see right, left, both, neither, then right again. */
#define SR_FROM 12U
#define SR_UNTIL 24U
#define SR_ITERATIONS 28U
static const uint8_t sr_masks[SR_UNTIL-SR_FROM]={1,1,1,2,2,2,3,3,0,0,1,1};
/* Modes 14/15: the right sensor stays on black from TOUCH_AT; the sonar
 * looking aside (front unknown: not clear, no obstacle) is a TRACK STOP from
 * STOP_AT for 300 ms (14) / 700 ms (15) with the touch still held. Mode 14
 * then holds right, then left, for HOLD_TICKS each: long enough to pivot. */
#define TOUCH_AT 10U
#define STOP_AT 13U
#define HOLD_TICKS 40U
static unsigned stop_ticks(void) { return failure_mode==14U?15U:35U; }
static unsigned resume_at(void) { return STOP_AT+stop_ticks(); }
static unsigned left_from(void) { return resume_at()+HOLD_TICKS; }
static unsigned scenario_mask(unsigned n)
{
    if(failure_mode==13U)return n>=SR_FROM && n<SR_UNTIL?sr_masks[n-SR_FROM]:0U;
    if(failure_mode==14U || failure_mode==15U)
        return n<TOUCH_AT?0U:failure_mode==14U && n>=left_from()?2U:1U;
    return 0U; /* modes 7..12: centred, both sensors WHITE */
}
static bool scenario_reading(unsigned n) { return failure_mode==13U && n>=SR_FROM && n<SR_UNTIL; }
static bool scenario_dropout(unsigned n)
{ return (failure_mode==14U || failure_mode==15U) && n>=STOP_AT && n<resume_at(); }
/* Mode 16: forward sonar samples while tracking a centred line, one per
 * tick from OC_FROM (H: one near echo re-read for several ticks, F far,
 * N near 150 mm, D no-echo, C close 40 mm, I invalidated by S5 re-centring)
 * and what TRACK must do (d drive, s hold still, a VC_ACQUIRE): the held
 * echo counts once; a no-echo holds only right after a near echo and is no
 * reading; C and I hold; acquisition only on the third near echo. */
#define OC_FROM 12U
static const char oc_sample[]="HHHHHFNNDFDCFINNDNNN";
static const char oc_expect[]="ddddddddsddsdsddsaaa";
#define OC_ITERATIONS (OC_FROM+sizeof(oc_sample)-1U)
#endif
static unsigned iterations(void)
{
#if BOARD_LINE_STRADDLE
    if(failure_mode==13U)return SR_ITERATIONS;
    if(failure_mode==14U)return left_from()+HOLD_TICKS;
    if(failure_mode==15U)return resume_at()+4U;
    if(failure_mode==16U)return OC_ITERATIONS;
#endif
    return failure_mode==10U?BR_ITERATIONS:failure_mode==11U?20U:12U;
}

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(failure_mode==10U && iteration>BR_MISS_AT && saved.vehicle.phase!=VC_TRACK)++left_track;
    if (++iteration >= iterations()) longjmp(done,1);
    clock_ms += 20U;
    return E_OK;
}
void tk_ext_tsk(void) { assert(0); }
void s2_controller_bind(void) {}
ER s3_controller_bind(void) { return E_OK; }
ER s5_controller_bind(void) { return E_OK; }
ER s2_emergency_stop(void) { ++stops;if(iteration<LOG_TICKS)stopped[iteration]=1;return E_OK; }
/* Positive drive request accounting shared by S2_MOVE_* and the PID follow
 * request (forward with steering), which the straddle/PID build uses. */
static void count_move(s2_move_command_t command,UINT power)
{
    ++positive_requests;
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
    ++positive_requests;log_send(SENT_REVERSE,S2_MOVE_REVERSE,power,steering);
    ++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;return E_OK;
}
ER s2_motion_get_status(s2_motion_status_t *s)
{ *s=(s2_motion_status_t){0};return E_OK; }
ER s2_encoder_snapshot(uint8_t *p,size_t capacity)
{
    uint32_t stamp=clock_ms;
    if(failure_mode==1U && iteration>=8U)stamp+=100U;
    encoder_snapshot_t e={0,0,stamp,ENCODER_SNAPSHOT_VALID};
    assert(encoder_snapshot_encode(&e,p,capacity));
    /* A failed provider deliberately leaves old-looking data behind. */
    return failure_mode==2U && iteration>=8U?E_SYS:E_OK;
}
ER s3_status_get(s3_status_t *s) {
    *s=(s3_status_t){0};
    if(failure_mode>=7U) {s->configured=1;s->sample_time_ms=clock_ms;s->output.steering_allowed=1;}
    if(failure_mode==10U && iteration>=BR_MISS_AT)s->output.barcode_fail_count=1;
    /* A marker in progress (bar, gap, bar...): a single crossing line with
     * fewer than 3 elements must not slow the robot (checked at READ_FROM-1). */
    if(failure_mode==11U && iteration>=READ_FROM-1U && iteration<READ_UNTIL) {
        s->output.barcode_result=S3_BARCODE_RESULT_READING;
        s->output.barcode_elements=(uint8_t)(iteration<READ_FROM?2U:3U+iteration-READ_FROM);
    }
#if BOARD_LINE_STRADDLE
    /* The straddle observer interprets S3's raw scan itself: a complete,
     * fresh sample at the measured GP26/27 endpoints (GP28 digital WHITE). */
    if(failure_mode>=7U) {
        static const uint16_t white[S3_SENSOR_COUNT]=BOARD_IR_ADC_WHITE,black[S3_SENSOR_COUNT]=BOARD_IR_ADC_BLACK;
        unsigned mask=scenario_mask(iteration);
        s->sample.raw[0]=(mask&1U)?black[0]:white[0]; /* GP26 right */
        s->sample.raw[1]=(mask&2U)?black[1]:white[1]; /* GP27 left */
        s->sample.raw[2]=0U;s->sample.valid_mask=7U;s->sample.time_ms=clock_ms;
        if(scenario_reading(iteration)) {
            s->output.barcode_result=S3_BARCODE_RESULT_READING;
            s->output.barcode_elements=(uint8_t)(3U+iteration-SR_FROM);
        }
    }
#endif
    return E_OK;
}
ER s5_get_status(s5_status_t *s) {
    *s=(s5_status_t){0};
    if(failure_mode>=7U) {
        s->state=S5_MONITOR;s->valid=1;s->distance_mm=400;s->time_ms=clock_ms;
        if(iteration>=8U && failure_mode==7U)s->distance_mm=UINT32_MAX;
        if(iteration==BR_GLITCH_AT && failure_mode==10U)s->distance_mm=150; /* one spurious echo */
        if(iteration>=8U && failure_mode==8U) {
            s->state=S5_READY;s->distance_mm=100;s->approach_plan_valid=1;s->object_width_valid=1;
            s->planned_rotate_deg=65537;s->planned_approach_mm=65537;s->planned_return_mm=65537;
        }
#if BOARD_LINE_STRADDLE
        if(scenario_dropout(iteration)) {s->angle_deg=s->sample_angle_deg=30;} /* fresh, looking aside */
        if(failure_mode==16U && iteration>=OC_FROM && iteration<OC_ITERATIONS) {
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
#endif
    }
    return E_OK;
}
ER s4_sensor_get_record(s4_sensor_cache_t *s)
{
    s4_data_reset(s);s4_data_init_result(s,LSM303_OK);
    lsm303_frame_t frame={0};frame.accel.fresh=frame.accel.valid=1;
    frame.accel.read_time_us=(uint64_t)(clock_ms-(failure_mode==3U?40U:0U))*1000U;
    frame.accel.raw[2]=1000;
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
    if(failure_mode>=7U && iteration==7U && !reset_offered) {
        *c=(control_message_t){.time_ms=clock_ms};
        /* ARM at 30% with a 1000 ms lease; the straddle modes START instead
         * (mission, no lease) so they can run past one second. */
        c->wire[1]=failure_mode>=STRADDLE_FIRST?3:1;c->wire[2]=30;c->wire[3]=0xe8;c->wire[4]=3;
        reset_offered=true;return E_OK;
    }
    return E_TMOUT;
}
ER s1_command_offer(const uint8_t *wire,UINT length)
{ (void)wire;(void)length;return E_OK; }
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
{ (void)id;(void)mode;(void)angle;return E_OK; }
ER s3_event_receive(s3_event_record_t *event) { (void)event;return E_TMOUT; }
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
#if BOARD_LINE_STRADDLE
static unsigned codes_seen; /* line_follow values published across the straddle modes */
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
/* (i) Mode 13: while S3 reads a marker the course is off, but a single
 * straddle sensor on black steers toward it: right (mask 1) positive, left
 * (mask 2) negative, capped at BOARD_BARCODE_READ_LINE_STEER and never a
 * pivot; both or neither on black is straight. (The course, reset on every
 * reading tick, would steer BOARD_LINE_STEER_MIN: the Makefile's _read150
 * build sets the cap below that, so only the read path passes the cap.) */
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
/* (ii) Modes 14/15: a TRACK STOP inside a held right touch. The resume tick
 * steers as if the standstill never happened (one tick of ramp past the
 * pre-stop steer): no jump to full steer (300 ms stop) or a pivot (700 ms). */
static void check_stop_resume(void)
{
    const unsigned r=resume_at();
    const int32_t step=(BOARD_LINE_STEER_MAX-BOARD_LINE_STEER_MIN)*(int32_t)TICK_MS/(int32_t)BOARD_LINE_STEER_RAMP_MS;
    int32_t before=sent_steer[STOP_AT-1U],after=sent_steer[r];
    for(unsigned n=TOUCH_AT;n<STOP_AT;++n)assert(sent_kind[n]==SENT_FOLLOW && sent_steer[n]>0);
    assert(before>=BOARD_LINE_STEER_MIN && before<BOARD_LINE_STEER_MAX); /* still ramping */
    /* Standing still in TRACK with the right sensor still on black. */
    for(unsigned n=STOP_AT;n<r;++n)
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE && pub_mask[n]==1U);
    assert(sent_kind[r]==SENT_FOLLOW && after>=before && after<=before+step && after<BOARD_LINE_STEER_MAX);
    if(failure_mode==14U) {
        /* Driving time only: each way the pivot comes BOARD_LINE_PIVOT_AFTER_MS
         * of moving touch after the touch began, the stop excluded. */
        const unsigned pivot_ticks=(BOARD_LINE_PIVOT_AFTER_MS+TICK_MS-1U)/TICK_MS;
        unsigned first_right=0,first_left=0;
        for(unsigned n=r;n<iterations();++n) {
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]!=SENT_NONE);
            if(!first_right && sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_RIGHT)first_right=n;
            if(!first_left && sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_LEFT)first_left=n;
        }
        assert(first_right==TOUCH_AT+stop_ticks()+pivot_ticks && first_right<left_from());
        assert(first_left==left_from()+pivot_ticks && sent_kind[left_from()]==SENT_FOLLOW &&
               sent_steer[left_from()]<0);
    }
}
#endif
int main(void)
{
    for(failure_mode=0;failure_mode<MODE_COUNT;++failure_mode) {
        memset(sent_kind,0,sizeof(sent_kind));memset(sent_cmd,0,sizeof(sent_cmd));
        memset(sent_power,0,sizeof(sent_power));memset(sent_steer,0,sizeof(sent_steer));
        memset(stopped,0,sizeof(stopped));memset(pub_seen,0,sizeof(pub_seen));
        assert(iterations()<=LOG_TICKS);
        iteration=stops=positive_requests=0;clock_ms=100;reset_offered=false;
        shock_published=after_shock=0;
        reverse_requests=reverse_after_glitch=left_track=0;saved=(app_state_t){0};
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
        if(failure_mode>=9U)assert(positive_requests>0);
        else assert(!positive_requests);
        if(failure_mode==7U)assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SENSOR &&
            saved.vehicle.fault_cause==VC_CAUSE_SONAR);
        if(failure_mode==12U) {
            /* The shock is always detected and published (dashboard impact),
             * and counted once so telemetry sends it even after it clears. */
            assert(shock_published==1U && saved.shock_count==1U);
#if BOARD_SHOCK_STOP
            assert(!saved.vehicle.armed && saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SHOCK &&
                   !after_shock);
#else
            /* Not a stop: tracking continues, and the one-sample tilt gap is
             * inside the grace, so no VC_SENSOR either. */
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && saved.vehicle.fault==VC_OK &&
                   after_shock>0U);
#endif
        }
#if BOARD_BARCODE_RETRY_DRIVE
        /* The glitch arrives while reversing: TRACK holds and the reverse continues. */
        if(failure_mode==10U)assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && !left_track &&
            reverse_requests>reverse_after_glitch && reverse_after_glitch>=BR_ITERATIONS-BR_GLITCH_AT-2U);
#endif
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
#if BOARD_LINE_STRADDLE
        if(failure_mode>=7U)check_published();
        if(failure_mode==13U)check_read_steer();
        if(failure_mode==14U || failure_mode==15U)check_stop_resume();
        if(failure_mode==16U) {
            _Static_assert(sizeof(oc_expect)==sizeof(oc_sample),"mode 16 table");
            for(unsigned n=10U;n<OC_ITERATIONS;++n) {
                char e=n<OC_FROM?'d':oc_expect[n-OC_FROM];
                assert(pub_armed[n]);
                if(e=='a')assert(pub_phase[n]==VC_ACQUIRE);
                else if(e=='s')assert(pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE);
                else assert(pub_phase[n]==VC_TRACK && !stopped[n] && sent_kind[n]==SENT_FOLLOW);
            }
        }
#endif
    }
#if BOARD_LINE_STRADDLE
    assert(codes_seen==0x1FU); /* straight, steer right/left and pivot right/left all published */
    puts("Straddle barcode read: one straddle sensor on black steers toward it (capped, no pivot), both/neither straight PASS");
    puts("Straddle TRACK STOP inside a held side touch: resume steers one tick on, not full steer or a pivot PASS");
    puts("Straddle app_state line_follow/line_mask/line_steer match the S2 request every tick PASS");
    puts("Obstacle while tracking: re-reads count once, no-echo is no reading, 5 cm/invalidated/no-echo-after-near hold; 3 consecutive new near echoes acquire PASS");
#endif
    puts("INT-S4-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS");
    puts("Barcode retry: a one-sample forward sonar glitch does not abort the reverse re-read PASS");
    puts("Barcode first pass: S3 READING caps the forward power to the re-read speed PASS");
    puts("MAP command: applied in any phase, invalid rejected, stale ACK 3 PASS");
    printf("Shock while tracking: published, %s PASS\n",
           BOARD_SHOCK_STOP?"stops the run (VC_SHOCK)":"run continues (BOARD_SHOCK_STOP 0)");
    return 0;
}
