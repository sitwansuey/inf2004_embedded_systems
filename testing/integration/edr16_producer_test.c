/* Exercise the real sensor producer and copied EDR16 queue in one task pass. */
#include "s5_hardware_host.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define TM_NET_LWIP 1
#define TM_MQTT 1
#define BOARD_IR_POSITIONS {-1,1,0}
#define BOARD_BARCODE_SENSOR 2
INT tm_sprintf(UB *output, const UB *format, ...);
#include "../../app_program/subsystem1/telemetry.c"
#include "../../app_program/subsystem1/task_edr16.c"

static jmp_buf finished;
static unsigned scenario, waits, queued, head;
static unsigned char messages[S1_TELEMETRY_CAPACITY][256];
static INT lengths[S1_TELEMETRY_CAPACITY];

ID tk_cre_mbf(const T_CMBF *config)
{ assert(config->maxmsz<=256);return 1; }
ER tk_get_otm(SYSTIM *now) { now->hi=0;now->lo=1000;return E_OK; }
ER tk_dly_tsk(UW delay)
{ assert(delay==250);if(++waits==2)longjmp(finished,1);return E_OK; }
INT tm_sprintf(UB *output,const UB *format,...)
{
    va_list args;va_start(args,format);
    INT result=vsprintf((char *)output,(const char *)format,args);
    va_end(args);return result;
}
ER tk_snd_mbf(ID id,const void *message,INT length,TMO timeout)
{
    assert(id==1 && timeout==TMO_POL && length>0 && length<=256);
    if(queued==S1_TELEMETRY_CAPACITY)return E_TMOUT;
    unsigned slot=(head+queued)%S1_TELEMETRY_CAPACITY;
    memcpy(messages[slot],message,(size_t)length);lengths[slot]=length;++queued;return E_OK;
}
INT tk_rcv_mbf(ID id,void *message,TMO timeout)
{
    assert(id==1 && timeout==TMO_POL);
    if(!queued)return E_TMOUT;
    INT length=lengths[head];memcpy(message,messages[head],(size_t)length);
    head=(head+1U)%S1_TELEMETRY_CAPACITY;--queued;return length;
}
ER s2_encoder_encode(uint8_t *output,size_t capacity)
{
    /* Control uses a distinct 13-byte snapshot; the producer requests only
     * the two independent external records. This catches buffer-size drift. */
    assert(capacity==4);
    if(scenario==4)return E_OBJ;
    assert(edr_encode(EDR_ENCODER_LEFT,4095,output,2));
    assert(edr_encode(EDR_ENCODER_RIGHT,0,output+2,2));return E_OK;
}
ER s2_motion_encode(uint8_t *output,size_t capacity)
{
    assert(capacity==4);
    if(scenario==4)return E_TMOUT;
    assert(edr_encode(EDR_MOTOR_STATUS,1265,output,2));
    assert(edr_encode(EDR_MOTOR_FAULTS,0,output+2,2));return E_OK;
}
ER s3_status_get(s3_status_t *status)
{
    *status=(s3_status_t){.configured=1,.acquisition_error=E_OK,.sample_time_ms=990};
    status->sample.valid_mask=7;
    for(unsigned i=0;i<S3_SENSOR_COUNT;++i)status->sample.raw[i]=(uint16_t)(100U+i);
    if(scenario==1)status->sample_time_ms=1000U-S3_STATUS_MAX_AGE_MS;
    if(scenario==2)status->sample_time_ms=1001;
    if(scenario==3)status->acquisition_error=E_IO;
    return E_OK;
}
ER s4_sensor_get_status(s4_sensor_status_t *status)
{
    *status=(s4_sensor_status_t){.configured=scenario!=4,.mag_usable=1,.accel_usable=1};
    const int32_t readings[3]={-2048,0,2047};
    for(unsigned i=0;i<3;++i) {
        status->latest.mag.milli_units[i]=readings[i];
        status->latest.accel.milli_units[i]=readings[i];
    }
    return E_OK;
}
int main(void)
{
    assert(s1_telemetry_init()==E_OK);
    for(scenario=0;scenario<5;++scenario) {
        waits=0;unsigned seen[16]={0},values[16]={0},total=0;
        if(setjmp(finished)==0)s1_sensor_telemetry_task(0,NULL);
        char topic[S1_TOPIC_MAX];unsigned char payload[S1_PAYLOAD_MAX];unsigned length;
        while(s1_telemetry_take_bytes(topic,payload,&length)==1) {
            unsigned id,data;
            assert(!strcmp(topic,EDR_MQTT_TOPIC) && length==2);
            assert(edr_decode(payload,length,&id,&data));
            ++seen[id];values[id]=data;++total;
        }
        if(scenario==4) {
            assert(total==3 && seen[5]==1 && seen[6]==1 && seen[7]==1);
        } else {
            assert(total==(scenario==0?13U:10U));
            assert(seen[1]==1 && seen[2]==1 && values[1]==4095 && values[2]==0);
            assert(seen[11]==1 && seen[12]==1 && values[11]==1265 && values[12]==0);
            assert(seen[5]==(scenario==0) && seen[6]==(scenario==0) && seen[7]==(scenario==0));
            for(unsigned id=8;id<=10;++id)assert(seen[id]==1);
            for(unsigned id=13;id<=15;++id)assert(seen[id]==1);
            assert(values[8]==0 && values[9]==2048 && values[10]==4095);
            assert(values[13]==0 && values[14]==2048 && values[15]==4095);
        }
        assert(!seen[3] && !seen[4]); /* No invented servo/sonar observations. */
    }
    puts("EDR16 producer: exact record buffers, copied queue, offsets, operating-clock freshness and owner errors PASS");
    return 0;
}
