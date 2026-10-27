/* Run the production telemetry task against snapshots published during a wait.
 * The real 50-byte codec feeds a fake transport; no network connection is made. */
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
static int64_t decoded[TEL_FIELD_COUNT];

ER tk_get_otm(SYSTIM *time)
{
    if(++clock_reads>1 && clock_failure)return E_SYS;
    time->hi=0;time->lo=clock_ms;return E_OK;
}
ER tk_dly_tsk(UW delay) { (void)delay;longjmp(finished,1);return E_OK; }
INT tm_printf(UB *format, ...) { (void)format;return 0; }
ER app_get_state(app_state_t *state) { *state=(app_state_t){0};return E_OK; }
ER s2_motion_get_status(s2_motion_status_t *state)
{ *state=(s2_motion_status_t){.state=S2_MOTION_STOPPED};return E_OK; }
ER s2_encoder_get_status(s2_encoder_status_t *state)
{ *state=(s2_encoder_status_t){.valid=1};return E_OK; }
ER s3_status_get(s3_status_t *state)
{
    *state=(s3_status_t){.configured=1,.acquisition_error=E_OK,.sample_time_ms=line_stamp};
    return E_OK;
}
ER s4_sensor_get_status(s4_sensor_status_t *state)
{ *state=(s4_sensor_status_t){0};return E_OK; }
ER s5_get_status(s5_status_t *state)
{
    if(copy_failure)return E_TMOUT;
    clock_ms+=read_delay;
    *state=(s5_status_t){.state=S5_MONITOR,.time_ms=clock_ms-sample_age,.valid=1,.distance_mm=400};
    return E_OK;
}
uint16_t s1_drop_count(void) { return 0; }
ER s1_binary_send(const uint8_t data[TELEMETRY_BYTES])
{
    REQUIRE(telemetry_decode(data,decoded));++packets;return E_OK;
}
static void capture(void)
{
    clock_ms=1000;clock_reads=packets=0;memset(decoded,0,sizeof(decoded));
    if(setjmp(finished)==0)telemetry_task(0,NULL);
}
int main(void)
{
    read_delay=10;sample_age=0;line_stamp=1000;
    capture();
    REQUIRE(packets==1 && decoded[TEL_TIME_MS]==1010);
    REQUIRE((decoded[TEL_AVAILABLE]&16)!=0 && decoded[TEL_DISTANCE_MM]==400);
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
    puts("INT-CR1-10: post-snapshot clock, fresh/new and stale data, failed copies/clock, actual binary codec PASS");
    return 0;
}
