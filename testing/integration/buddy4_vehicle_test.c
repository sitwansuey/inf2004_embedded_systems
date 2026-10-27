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
static jmp_buf done;
static app_state_t saved;
static uint32_t clock_ms, iteration;
static unsigned failure_mode, stops, positive_requests;
static bool reset_offered;

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if (++iteration >= 12U) longjmp(done,1);
    clock_ms += 20U;
    return E_OK;
}
void tk_ext_tsk(void) { assert(0); }
void s2_controller_bind(void) {}
ER s3_controller_bind(void) { return E_OK; }
ER s5_controller_bind(void) { return E_OK; }
ER s2_emergency_stop(void) { ++stops; return E_OK; }
ER s2_movement_send(s2_move_command_t command,UINT power)
{ (void)command;(void)power;++positive_requests;return E_OK; }
ER s2_motion_get_status(s2_motion_status_t *s)
{ *s=(s2_motion_status_t){0};return E_OK; }
ER s2_encoder_encode(uint8_t *p,size_t capacity)
{
    uint32_t stamp=clock_ms;
    if(failure_mode==1U && iteration>=8U)stamp+=100U;
    edr_encoder_t e={0,0,stamp,EDR_ENCODER_VALID};
    assert(edr_encoder_encode(&e,p,capacity));
    /* A failed provider deliberately leaves old-looking data behind. */
    return failure_mode==2U && iteration>=8U?E_SYS:E_OK;
}
ER s3_status_get(s3_status_t *s) {
    *s=(s3_status_t){0};
    if(failure_mode>=7U) {s->configured=1;s->sample_time_ms=clock_ms;s->output.steering_allowed=1;}
    return E_OK;
}
ER s5_get_status(s5_status_t *s) {
    *s=(s5_status_t){0};
    if(failure_mode>=7U) {
        s->state=S5_MONITOR;s->valid=1;s->distance_mm=400;s->time_ms=clock_ms;
        if(iteration>=8U && failure_mode==7U)s->distance_mm=UINT32_MAX;
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
    if(failure_mode==6U)frame.accel.raw[0]=1900;
    s4_data_update(s,&frame,LSM303_OK);return E_OK;
}

ER s1_command_take(control_message_t *c)
{
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
ER s5_navigation_set(uint32_t id,s5_state_t mode,INT angle)
{ (void)id;(void)mode;(void)angle;return E_OK; }
ER s3_event_receive(s3_event_record_t *event) { (void)event;return E_TMOUT; }
ER app_state_publish(const app_state_t *s) { saved=*s;return E_OK; }
ID app_start_task(FP task,PRI priority,UW core,const char *name)
{ (void)task;(void)priority;(void)core;(void)name;return 1; }
int main(void)
{
    for(failure_mode=0;failure_mode<10;++failure_mode) {
        iteration=stops=positive_requests=0;clock_ms=100;reset_offered=false;
        if(setjmp(done)==0)vehicle_task(0,NULL);
        s4_result_t result={0};assert(s4_result_decode(saved.s4_result,sizeof(saved.s4_result),&result));
        if(failure_mode==0)assert((result.flags&S4_RESULT_TILT) && saved.odometry_valid);
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
        if(failure_mode==9U)assert(positive_requests>0);
        else assert(!positive_requests);
        if(failure_mode==7U)assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SENSOR);
    }
    puts("INT-S4-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS");
    return 0;
}
