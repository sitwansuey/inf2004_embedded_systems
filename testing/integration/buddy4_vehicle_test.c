/* Run the actual controller task with synthetic owner snapshots. */
#include "s5_hardware_host.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
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

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(failure_mode==10U && iteration>BR_MISS_AT && saved.vehicle.phase!=VC_TRACK)++left_track;
    if (++iteration >= (failure_mode==10U?BR_ITERATIONS:failure_mode==11U?20U:12U)) longjmp(done,1);
    clock_ms += 20U;
    return E_OK;
}
void tk_ext_tsk(void) { assert(0); }
void s2_controller_bind(void) {}
ER s3_controller_bind(void) { return E_OK; }
ER s5_controller_bind(void) { return E_OK; }
ER s2_emergency_stop(void) { ++stops; return E_OK; }
ER s2_movement_send(s2_move_command_t command,UINT power)
{
    ++positive_requests;
    if(failure_mode==12U && iteration>SHOCK_AT && command==S2_MOVE_FORWARD && power)++after_shock;
    if(failure_mode==11U && command==S2_MOVE_FORWARD) {
        if(iteration>=READ_FROM && iteration<READ_UNTIL) {++read_count;if(power>read_max)read_max=power;}
        else {++normal_count;if(power<normal_min)normal_min=power;}
    }
    if(command==S2_MOVE_REVERSE) {++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;}
    return E_OK;
}
ER s2_reverse_follow_send(UINT power,INT steering)
{
    (void)power;(void)steering;++positive_requests;
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
        c->wire[1]=1;c->wire[2]=30;c->wire[3]=0xe8;c->wire[4]=3;
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
    s4_result_t r;
    if(failure_mode==12U && s4_result_decode(s->s4_result,sizeof(s->s4_result),&r) &&
       (r.flags&S4_RESULT_SHOCK))++shock_published;
    return E_OK;
}
ID app_start_task(FP task,PRI priority,UW core,const char *name)
{ (void)task;(void)priority;(void)core;(void)name;return 1; }
int main(void)
{
    for(failure_mode=0;failure_mode<13;++failure_mode) {
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
    }
    puts("INT-S4-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS");
    puts("Barcode retry: a one-sample forward sonar glitch does not abort the reverse re-read PASS");
    puts("Barcode first pass: S3 READING caps the forward power to the re-read speed PASS");
    puts("MAP command: applied in any phase, invalid rejected, stale ACK 3 PASS");
    printf("Shock while tracking: published, %s PASS\n",
           BOARD_SHOCK_STOP?"stops the run (VC_SHOCK)":"run continues (BOARD_SHOCK_STOP 0)");
    return 0;
}
