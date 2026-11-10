/* Run the production telemetry task against snapshots published during a wait.
 * The real 54-byte codec feeds a fake transport; no network connection is made. */
#include "s5_hardware_host.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define REQUIRE(c) do { if(!(c)) { \
    fprintf(stderr,"Snapshot time failed line %d: %s\n",__LINE__,#c);exit(2); } } while(0)
INT tm_printf(UB *format, ...);
#include "../../app_program/subsystem1/task_telemetry.c"
static jmp_buf finished;
static UW clock_ms, read_delay, sample_age, line_stamp;
static unsigned clock_reads, packets;
static BOOL copy_failure, clock_failure;
static BOOL bad_s4_record;
static BOOL bad_s5_fields;
static int32_t test_speed;
static uint8_t test_fault,test_cause;
static int64_t decoded[TEL_FIELD_COUNT];
/* Optional extra task loops (10 ms apart) with a per-loop shock count. */
static unsigned extra_loops, loop_index;
static uint8_t shock_counts[4];
static int64_t qualities[4];
/* Line follower view published by the vehicle task (app_state line_*), and
 * an S3 edge-follow line output that differs from it. */
static uint8_t test_follow,test_mask;
static int16_t test_steer;
static BOOL s3_line_output;

ER tk_get_otm(SYSTIM *time)
{
    if(++clock_reads>1 && clock_failure)return E_SYS;
    time->hi=0;time->lo=clock_ms;return E_OK;
}
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(loop_index<extra_loops) {++loop_index;clock_ms+=10;return E_OK;}
    longjmp(finished,1);return E_OK;
}
INT tm_printf(UB *format, ...) { (void)format;return 0; }
ER app_get_state(app_state_t *state) {
    *state=(app_state_t){0};
    state->speed_bits=(uint16_t)((uint32_t)test_speed & UINT32_C(0xfff));
    state->odometry_valid=1;state->vehicle.fault=test_fault;state->vehicle.fault_cause=test_cause;
    state->shock_count=shock_counts[loop_index<4U?loop_index:3U];
    state->line_follow=test_follow&7U;state->line_mask=test_mask&3U;state->line_steer=test_steer;
    s4_result_t result={.calculated_ms=clock_ms,.pitch_cdeg=1234,.roll_cdeg=-5678,
        .height_mm=123,.maximum_mm=456,.terrain=0,.flags=27};
    REQUIRE(s4_result_encode(&result,state->s4_result,sizeof(state->s4_result)));
    if(bad_s4_record)state->s4_result[11]|=0x80;
    return E_OK;
}
ER s2_motion_get_status(s2_motion_status_t *state)
{ *state=(s2_motion_status_t){.state=S2_MOTION_STOPPED};return E_OK; }
ER s2_encoder_get_status(s2_encoder_status_t *state)
{ *state=(s2_encoder_status_t){.valid=1};return E_OK; }
ER s3_status_get(s3_status_t *state)
{
    *state=(s3_status_t){.configured=1,.acquisition_error=E_OK,.sample_time_ms=line_stamp};
    if(s3_line_output) {
        state->output.line.state=S3_LINE_TRACKING;state->output.line.black_mask=2U;state->output.line.error=-300;
    }
    return E_OK;
}
ER s4_sensor_get_record(s4_sensor_cache_t *state) {
    s4_data_reset(state);s4_data_init_result(state,LSM303_OK);
    lsm303_frame_t frame={0};frame.accel.fresh=frame.accel.valid=1;
    frame.accel.read_time_us=(uint64_t)clock_ms*1000U;frame.accel.raw[2]=1000;
    s4_data_update(state,&frame,LSM303_OK);return E_OK;
}
ER s5_get_status(s5_status_t *state)
{
    if(copy_failure)return E_TMOUT;
    clock_ms+=read_delay;
    *state=(s5_status_t){.state=S5_MONITOR,.time_ms=clock_ms-sample_age,.valid=1,.distance_mm=400};
    if(bad_s5_fields){state->distance_mm=UINT32_MAX;state->object_width_valid=1;state->object_width_mm=UINT32_MAX;}
    return E_OK;
}
uint16_t s1_drop_count(void) { return 0; }
ER s1_binary_send(const uint8_t data[TELEMETRY_BYTES])
{
    REQUIRE(telemetry_decode(data,decoded));
    if(packets<4U)qualities[packets]=decoded[TEL_QUALITY];
    ++packets;return E_OK;
}
static void capture(void)
{
    clock_ms=1000;clock_reads=packets=0;memset(decoded,0,sizeof(decoded));loop_index=0;
    if(setjmp(finished)==0)telemetry_task(0,NULL);
}
int main(void)
{
    read_delay=10;sample_age=0;line_stamp=1000;
    capture();
    REQUIRE(packets==1 && decoded[TEL_TIME_MS]==1010);
    REQUIRE((decoded[TEL_AVAILABLE]&16)!=0 && decoded[TEL_DISTANCE_MM]==400);
    REQUIRE((decoded[TEL_AVAILABLE]&96)==96 && decoded[TEL_PITCH_CDEG]==1234 &&
        decoded[TEL_ROLL_CDEG]==-5678 && decoded[TEL_HEIGHT_MM]==123 && decoded[TEL_HIGHEST_MM]==456);
    const int32_t speeds[]={-2000,-1,0,1,2000};
    for(unsigned n=0;n<sizeof(speeds)/sizeof(speeds[0]);++n) {
        test_speed=speeds[n];capture();
        REQUIRE(packets==1 && decoded[TEL_SPEED_MM_S]==speeds[n]);
        REQUIRE((decoded[TEL_QUALITY]&16)!=0);
    }
    REQUIRE(decoded[TEL_FAULT_CAUSE]==0);
    test_fault=VC_SENSOR;test_cause=VC_CAUSE_ODOM;capture();
    REQUIRE(decoded[TEL_FAULT]==VC_SENSOR && decoded[TEL_FAULT_CAUSE]==VC_CAUSE_ODOM);
    test_fault=VC_LINE;capture(); /* cause only describes a sensor fault */
    REQUIRE(decoded[TEL_FAULT]==VC_LINE && decoded[TEL_FAULT_CAUSE]==0);
    test_fault=VC_OK;test_cause=0;
    bad_s5_fields=TRUE;capture();
    REQUIRE(packets==1 && (decoded[TEL_AVAILABLE]&16)==0 && (decoded[TEL_QUALITY]&1)==0);
    bad_s5_fields=FALSE;
    bad_s4_record=TRUE;capture();REQUIRE((decoded[TEL_AVAILABLE]&96)==0);
    bad_s4_record=FALSE;read_delay=61;capture();
    REQUIRE((decoded[TEL_AVAILABLE]&96)==0 && decoded[TEL_HIGHEST_MM]==456);
    read_delay=20;line_stamp=990;
    capture();
    REQUIRE((decoded[TEL_AVAILABLE]&4)==0); /* line became stale during wait */
    sample_age=500;
    capture();
    REQUIRE((decoded[TEL_AVAILABLE]&16)==0 && decoded[TEL_DISTANCE_MM]==0);
    copy_failure=TRUE;
    capture();
    REQUIRE((decoded[TEL_AVAILABLE]&16)==0 && decoded[TEL_DISTANCE_MM]==0);
    copy_failure=FALSE;clock_failure=TRUE;
    capture();
    REQUIRE(packets==0); /* no publication with an unusable age reference */
    /* A shock lasts one 20 ms sample and no longer changes the fault: a new
     * shock count is sent at once (inside the 200 ms period) as impact. */
    clock_failure=FALSE;read_delay=0;sample_age=0;line_stamp=1000;
    extra_loops=3;shock_counts[0]=shock_counts[1]=0;shock_counts[2]=shock_counts[3]=1;
    capture();
    REQUIRE(packets==2 && (qualities[0]&2)==0 && (qualities[1]&2)!=0);
    extra_loops=0;
    /* Line fields. Straddle builds send the vehicle task's follower (0
     * straight, 1/2 steer right/left, 3/4 pivot right/left; straddle mask;
     * steer bounded to the field's +-1000); edge-follow builds S3's view. */
    s3_line_output=TRUE;
    static const struct { uint8_t follow,mask;int16_t steer;int64_t error; } lines[]={
        {4,3,-400,-400},{1,1,250,250},{2,2,-250,-250},{3,1,0,0},{0,0,0,0},{1,1,1500,1000},{2,2,-1500,-1000}};
    for(unsigned n=0;n<sizeof(lines)/sizeof(lines[0]);++n) {
        test_follow=lines[n].follow;test_mask=lines[n].mask;test_steer=lines[n].steer;
        capture();
        REQUIRE(packets==1 && (decoded[TEL_AVAILABLE]&4)!=0);
#if BOARD_LINE_STRADDLE
        REQUIRE(decoded[TEL_LINE_STATE]==lines[n].follow && decoded[TEL_BLACK_MASK]==lines[n].mask &&
            decoded[TEL_LINE_ERROR]==lines[n].error);
#else
        REQUIRE(decoded[TEL_LINE_STATE]==S3_LINE_TRACKING && decoded[TEL_BLACK_MASK]==2 &&
            decoded[TEL_LINE_ERROR]==-300);
#endif
    }
    line_stamp=960;capture(); /* 40 ms old line sample: no line fields in either build */
    REQUIRE((decoded[TEL_AVAILABLE]&4)==0 && decoded[TEL_LINE_STATE]==0 &&
        decoded[TEL_BLACK_MASK]==0 && decoded[TEL_LINE_ERROR]==0);
    line_stamp=1000;s3_line_output=FALSE;test_follow=test_mask=0;test_steer=0;
#if BOARD_LINE_STRADDLE
    puts("Straddle line telemetry: TEL_LINE_STATE/BLACK_MASK/LINE_ERROR from app_state line_follow/mask/steer (+-1000) PASS");
#else
    puts("Edge-follow line telemetry: TEL_LINE_STATE/BLACK_MASK/LINE_ERROR from S3, not app_state PASS");
#endif
    puts("Shock count: a new shock is sent at once with the impact bit PASS");
    puts("INT-CR1-10: post-snapshot clock, freshness/errors, packed signed speed, actual binary codec PASS");
    return 0;
}
