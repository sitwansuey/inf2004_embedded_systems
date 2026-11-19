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
/* Buffers in creation order: 1 telemetry, 2 commands, 3 acks, 4 events. */
static unsigned counts[5],sizes[5][S1_EVENT_CAPACITY];static unsigned char queue[5][S1_EVENT_CAPACITY][256];
static const unsigned capacity_of[5]={0,2,4,8,S1_EVENT_CAPACITY};
ID tk_cre_mbf(const T_CMBF *c) {assert(c->maxmsz<256);return (ID)++created;}
ER tk_del_mbf(ID id) {(void)id;return E_OK;}
ER tk_get_otm(SYSTIM *t) {t->hi=0;t->lo=now_ms;return E_OK;}
ER tk_snd_mbf(ID id,const void *p,INT size,TMO timeout) {
    assert(id>0 && id<5 && timeout==TMO_POL && size>0 && size<256);
    unsigned capacity=capacity_of[id];if(counts[id]==capacity)return E_TMOUT;
    sizes[id][counts[id]]=(unsigned)size;memcpy(queue[id][counts[id]++],p,(size_t)size);return E_OK;
}
INT tk_rcv_mbf(ID id,void *p,TMO timeout) {
    assert(id>0 && id<5 && timeout==TMO_POL);if(!counts[id])return E_TMOUT;
    unsigned size=sizes[id][0];memcpy(p,queue[id][0],size);--counts[id];
    memmove(queue[id],queue[id]+1,counts[id]*sizeof(queue[id][0]));
    memmove(sizes[id],sizes[id]+1,counts[id]*sizeof(sizes[id][0]));return (INT)size;
}
int main(void) {
    app_state_t state={0};
    for(unsigned symbol=0;symbol<=26;++symbol) {
        state.barcode=(uint8_t)symbol;
        assert(state.barcode==symbol);
        int64_t values[TEL_FIELD_COUNT]={0}, decoded_values[TEL_FIELD_COUNT];
        uint8_t packed[TELEMETRY_BYTES], record[2]={0};unsigned id=0,data=0;
        values[TEL_VERSION]=1;values[TEL_BARCODE]=state.barcode;
        assert(telemetry_encode(packed,values) && telemetry_decode(packed,decoded_values));
        assert(telemetry_edr_record(decoded_values,16,record));
        assert(edr_decode(record,2,&id,&data) && id==0 && (data&31U)==symbol);
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
    assert(s3_event_pack(wire,&event));assert(wire[0]==(4U|(4U<<3)) && wire[1]==26U);
    assert(s3_event_unpack(wire,&decoded));assert(decoded.event.kind==event.event.kind && decoded.generation==UINT32_MAX);
    /* Every letter A..Z with every turn round-trips: any letter can be
     * mapped from the dashboard (an F used to fail to pack and its turn was
     * lost). Non-letters and out-of-range codes are rejected. */
    for(unsigned n=1;n<=26U;++n) for(unsigned cmd=1;cmd<=4U;++cmd) {
        event.event.kind=S3_EVENT_NAVIGATION;event.event.symbol=(char)('A'+n-1U);event.event.command=(uint8_t)cmd;
        assert(s3_event_pack(wire,&event) && wire[1]==n && s3_event_unpack(wire,&decoded));
        assert(decoded.event.symbol==event.event.symbol && decoded.event.command==cmd &&
               decoded.event.kind==S3_EVENT_NAVIGATION);
    }
    event.event.symbol='a';assert(!s3_event_pack(wire,&event));
    event.event.symbol='@';assert(!s3_event_pack(wire,&event));
    event.event.symbol=0;assert(s3_event_pack(wire,&event) && wire[1]==0U && s3_event_unpack(wire,&decoded) && !decoded.event.symbol);
    wire[1]=27U;assert(!s3_event_unpack(wire,&decoded));
    unsigned char start[6]={0xC1,3,30,0,0,0};
    assert(s1_command_offer(start,6)==E_OK && s1_command_take(&c)==E_OK && c.wire[1]==3);
    start[3]=1;assert(s1_command_offer(start,6)==E_PAR);
    /* MAP: letter 1..26, action 0..4, byte 4 zero; 0/0 restores the codebook. */
    unsigned char map[6]={0xC1,4,6,1,0,7};
    assert(s1_command_offer(map,6)==E_OK && s1_command_take(&c)==E_OK &&
           c.wire[1]==4 && c.wire[2]==6 && c.wire[3]==1 && c.wire[5]==7);
    map[2]=0;map[3]=0;assert(s1_command_offer(map,6)==E_OK && s1_command_take(&c)==E_OK && !c.wire[2]);
    map[3]=1;assert(s1_command_offer(map,6)==E_PAR);
    map[2]=27;assert(s1_command_offer(map,6)==E_PAR);
    map[2]=26;map[3]=5;assert(s1_command_offer(map,6)==E_PAR);
    map[3]=4;map[4]=1;assert(s1_command_offer(map,6)==E_PAR);
    map[4]=0;assert(s1_command_offer(map,6)==E_OK && s1_command_take(&c)==E_OK && c.wire[3]==4);
    map[1]=5;assert(s1_command_offer(map,6)==E_PAR);
    wire[0]=0xFF;assert(!s3_event_unpack(wire,&decoded));
    /* Robot events (shared/events.h): one two-byte EDR16-format word each
     * (value<<4 | id), FIFO, S1_EVENT_CAPACITY deep, never blocking; a full
     * queue drops (and counts) the new event; bad ids/values are ignored. */
    {
        unsigned char record[EDR_RECORD_SIZE];
        #define EVENT_WORD(r) ((unsigned)(r)[0]|((unsigned)(r)[1]<<8U))
        assert(s1_event_take(record)==E_TMOUT && s1_event_take(NULL)==E_OBJ);
        uint16_t before=s1_drop_count();
        s1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U));
        s1_event_post(0U,1U);s1_event_post(16U,1U);s1_event_post(EV_LINK,EDR_DATA_MAX+1U);
        assert(s1_event_take(record)==(int)EDR_RECORD_SIZE &&
               EVENT_WORD(record)==((ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U)<<4U)|EV_OBSTACLE));
        assert(s1_event_take(record)==E_TMOUT && s1_drop_count()==before);
        for(unsigned n=0;n<S1_EVENT_CAPACITY;++n)s1_event_post(EV_BARCODE,ev_barcode(1U,n,n+1U));
        s1_event_post(EV_HUMP,ev_hump(EV_HUMP_DETECTED,50U));
        assert(s1_drop_count()==before+1U);
        for(unsigned n=0;n<S1_EVENT_CAPACITY;++n)
            assert(s1_event_take(record)==(int)EDR_RECORD_SIZE &&
                   EVENT_WORD(record)==((ev_barcode(1U,n,n+1U)<<4U)|EV_BARCODE));
        assert(s1_event_take(record)==E_TMOUT);
        /* Field packing: saturating values, letters 1..26 only. */
        assert(ev_barcode(3U,40U,27U)==(3U|(31U<<2U)) && ev_instruction(26U,4U)==(26U|(4U<<5U)));
        assert(ev_hump(EV_HUMP_CREST,600U)==(EV_HUMP_CREST|(511U<<3U)) && ev_link(EV_LINK_BOOT,2000U)==(EV_LINK_BOOT|(1023U<<2U)));
        assert(ev_obstacle(EV_OBSTACLE_BACK,3U,300U)==(EV_OBSTACLE_BACK|(3U<<2U)|(255U<<4U)));
        #undef EVENT_WORD
    }
    /* The drop counter wraps instead of sticking: status subtype 12 sends it
     * modulo 256, like the sequence (11), and receivers take differences. */
    {
        static const struct { int64_t drops; unsigned sent; } wraps[]={{0,0},{255,255},{256,0},{300,44},{65535,255}};
        int64_t values[TEL_FIELD_COUNT]={0};uint8_t record[2];unsigned id=0,data=0;
        __atomic_store_n(&drops,65535U,__ATOMIC_RELAXED);
        dropped();assert(s1_drop_count()==0U);dropped();assert(s1_drop_count()==1U);
        values[TEL_VERSION]=1;
        for(unsigned n=0;n<sizeof(wraps)/sizeof(wraps[0]);++n) {
            values[TEL_DROPPED]=wraps[n].drops;
            assert(telemetry_edr_record(values,15U+12U,record) && edr_decode(record,2,&id,&data));
            assert(id==0U && data==((12U<<8U)|wraps[n].sent));
        }
    }
    puts("INT-CR1-06 actual queues: copy ownership, full queue, urgent STOP, stale telemetry, validation and 14-byte events PASS");
    puts("Robot event queue: EDR16-format words, FIFO, 16 deep, full queue drops and counts, bad ids ignored PASS");
    puts("Drop counter: wraps past 65535, status subtype 12 is drops modulo 256 PASS");
    return 0;
}
