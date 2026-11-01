/* Actual handoff code with deterministic message-buffer and clock substitutes. */
#include "host.h"
#include <assert.h>
#include "../../app_program/app_state.h"
#include "../../app_program/subsystem2/s2_data.h"
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem1/telemetry.c"
#include "../../app_program/subsystem3/event_codec.h"
static unsigned now_ms, created;
static unsigned counts[3],sizes[3][4];static unsigned char queue[3][4][256];
ID tk_cre_mbf(const T_CMBF *c) {assert(c->maxmsz<256);return (ID)++created;}
ER tk_del_mbf(ID id) {(void)id;return E_OK;}
ER tk_get_otm(SYSTIM *t) {t->hi=0;t->lo=now_ms;return E_OK;}
ER tk_snd_mbf(ID id,const void *p,INT size,TMO timeout) {
    assert(id>0 && id<3 && timeout==TMO_POL && size>0 && size<256);
    unsigned capacity=id==1?2:4;if(counts[id]==capacity)return E_TMOUT;
    sizes[id][counts[id]]=(unsigned)size;memcpy(queue[id][counts[id]++],p,(size_t)size);return E_OK;
}
INT tk_rcv_mbf(ID id,void *p,TMO timeout) {
    assert(id>0 && id<3 && timeout==TMO_POL);if(!counts[id])return E_TMOUT;
    unsigned size=sizes[id][0];memcpy(p,queue[id][0],size);--counts[id];
    memmove(queue[id],queue[id]+1,counts[id]*sizeof(queue[id][0]));
    memmove(sizes[id],sizes[id]+1,counts[id]*sizeof(sizes[id][0]));return (INT)size;
}
int main(void) {
    app_state_t state={0};
    for(unsigned symbol=0;symbol<=5;++symbol) {
        state.barcode=(uint8_t)symbol;
        assert(state.barcode==symbol);
        int64_t values[TEL_FIELD_COUNT]={0}, decoded_values[TEL_FIELD_COUNT];
        uint8_t packed[TELEMETRY_BYTES], record[2];unsigned id,data;
        values[TEL_VERSION]=1;values[TEL_BARCODE]=state.barcode;
        assert(telemetry_encode(packed,values) && telemetry_decode(packed,decoded_values));
        assert(telemetry_edr_record(decoded_values,16,record));
        assert(edr_decode(record,2,&id,&data) && id==0 && (data&7U)==symbol);
    }
    assert(edr_motor_fault_flags(s2_motor_error_code(E_OK))==0);
    assert(edr_motor_fault_flags(s2_motor_error_code(E_TMOUT))==32);
    assert(edr_motor_fault_flags(s2_motor_error_code(E_SYS))==64);
    assert(s1_binary_init()==E_OK);assert(s1_binary_init()==E_OBJ);
    int64_t v[TEL_FIELD_COUNT]={0};v[TEL_VERSION]=1;uint8_t p[TELEMETRY_BYTES],copy[TELEMETRY_BYTES];
    assert(telemetry_encode(p,v));assert(s1_binary_send(p)==E_OK);p[1]=123;
    assert(s1_telemetry_take(copy)==TELEMETRY_BYTES && copy[1]==0);
    p[1]=0;assert(s1_binary_send(p)==E_OK);assert(s1_binary_send(p)==E_OK);
    assert(s1_binary_send(p)==E_TMOUT);now_ms=500;assert(s1_telemetry_take(copy)==E_TMOUT);
    unsigned char arm[6]={0xC1,1,30,0xF4,1,1},stop[6]={0xC1,0,0,0,0,2};control_message_t c;
    for(unsigned n=0;n<4;++n)assert(s1_command_offer(arm,6)==E_OK);
    assert(s1_command_offer(arm,6)==E_TMOUT);assert(s1_command_offer(stop,6)==E_OK);
    assert(s1_command_take(&c)==E_OK && c.wire[1]==0);
    assert(s1_command_take(&c)==E_TMOUT); /* every pre-STOP ARM was invalidated */
    assert(s1_command_offer(arm,6)==E_OK);arm[2]=99;
    assert(s1_command_take(&c)==E_OK && c.wire[2]==30 && c.time_ms==500);
    assert(s1_command_offer(arm,6)==E_PAR);assert(s1_command_offer(arm,5)==E_PAR);
    arm[2]=30;arm[3]=0xFF;arm[4]=0xFF;assert(s1_command_offer(arm,6)==E_PAR);
    s3_event_record_t event={.event={.kind=S3_EVENT_FAULT,.command=S3_NAV_UTURN,.symbol='Z',.sequence=UINT32_MAX,.time_ms=UINT32_MAX},.generation=UINT32_MAX};
    uint8_t wire[S3_EVENT_BYTES];s3_event_record_t decoded;
    assert(s3_event_pack(wire,&event));assert(wire[0]==(4U|(4U<<3)) && wire[1]==5U);
    assert(s3_event_unpack(wire,&decoded));assert(decoded.event.kind==event.event.kind && decoded.generation==UINT32_MAX);
    for(unsigned n=1;n<=4;++n) {
        event.event.symbol=(char)('A'+n-1U);event.event.command=n;
        assert(s3_event_pack(wire,&event) && s3_event_unpack(wire,&decoded));
        assert(decoded.event.symbol==event.event.symbol && decoded.event.command==n);
    }
    unsigned char start[6]={0xC1,3,30,0,0,0};
    assert(s1_command_offer(start,6)==E_OK && s1_command_take(&c)==E_OK && c.wire[1]==3);
    start[3]=1;assert(s1_command_offer(start,6)==E_PAR);
    wire[0]=0xFF;assert(!s3_event_unpack(wire,&decoded));
    puts("INT-CR1-06 actual queues: copy ownership, full queue, urgent STOP, stale telemetry, validation and 14-byte events PASS");
    return 0;
}
