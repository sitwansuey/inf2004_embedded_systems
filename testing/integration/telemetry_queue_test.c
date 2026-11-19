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
/* tk/smp.h atomics (one core here): calls counted, so the drop counter and
 * the STOP word are shown to use them. */
static unsigned atomic_calls;
UW atomic_inc(UW *addr) {++atomic_calls;return ++*addr;}
UW atomic_cmpxchg(UW *addr,UW val,UW cmp) {++atomic_calls;UW old=*addr;if(old==cmp)*addr=val;return old;}
/* Last published VC state, for replies the VC did not make. */
static unsigned test_phase, test_fault;
ER app_get_state(app_state_t *p) {
    *p=(app_state_t){0};p->vehicle.phase=test_phase&15U;p->vehicle.fault=test_fault&15U;return E_OK;
}
static bool ack_is(const unsigned char *a,unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault) {
    return a[0]==0xC2 && a[1]==seq && a[2]==opcode && a[3]==result && a[4]==phase && a[5]==fault;
}
static void snapshot_with(uint8_t out[TELEMETRY_BYTES],unsigned sequence,unsigned time_ms,unsigned quality,unsigned imu) {
    int64_t v[TEL_FIELD_COUNT]={0};v[TEL_VERSION]=1;v[TEL_SEQUENCE]=sequence;v[TEL_TIME_MS]=time_ms;
    v[TEL_QUALITY]=quality;v[TEL_IMU_FAULT]=imu;assert(telemetry_encode(out,v));
}
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
    /* A full queue drops its oldest snapshot (counted) and queues the newest. */
    {uint16_t d=s1_drop_count();assert(s1_binary_send(p)==E_OK && s1_drop_count()==d+1U && counts[1]==2);}
    now_ms=500;{uint16_t d=s1_drop_count();assert(s1_telemetry_take(copy)==E_TMOUT && s1_drop_count()==d+2U);}
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
    /* Snapshots: the network takes the newest; an older or stale one still
     * queued is dropped (counted) and hands on its one-snapshot flags
     * (impact = quality bit 1, IMU fault), so neither is lost. */
    {
        uint8_t a[TELEMETRY_BYTES],b[TELEMETRY_BYTES],c[TELEMETRY_BYTES],out[TELEMETRY_BYTES];
        int64_t got[TEL_FIELD_COUNT];
        snapshot_with(a,1,now_ms,2|16,1);snapshot_with(b,2,now_ms,16,0);snapshot_with(c,3,now_ms,16,0);
        uint16_t d=s1_drop_count();
        assert(s1_binary_send(a)==E_OK && s1_binary_send(b)==E_OK);
        assert(s1_telemetry_take(out)==(int)TELEMETRY_BYTES && s1_drop_count()==d+1U && telemetry_decode(out,got));
        assert(got[TEL_SEQUENCE]==2 && got[TEL_QUALITY]==(2|16) && got[TEL_IMU_FAULT]==1);
        assert(s1_telemetry_take(out)==E_TMOUT);
        /* Full at send: a goes, its flags ride c; b is then dropped at take. */
        assert(s1_binary_send(a)==E_OK && s1_binary_send(b)==E_OK && s1_binary_send(c)==E_OK && s1_drop_count()==d+2U);
        assert(s1_telemetry_take(out)==(int)TELEMETRY_BYTES && s1_drop_count()==d+3U && telemetry_decode(out,got));
        assert(got[TEL_SEQUENCE]==3 && got[TEL_QUALITY]==(2|16) && got[TEL_IMU_FAULT]==1);
        /* A stale one (500 ms) is dropped too, and its flags still go on. */
        snapshot_with(a,4,now_ms-500U,2,1);
        assert(s1_binary_send(a)==E_OK && s1_binary_send(b)==E_OK);
        assert(s1_telemetry_take(out)==(int)TELEMETRY_BYTES && s1_drop_count()==d+4U && telemetry_decode(out,got));
        assert(got[TEL_SEQUENCE]==2 && got[TEL_QUALITY]==(2|16) && got[TEL_IMU_FAULT]==1);
        /* No older snapshot: nothing added. */
        assert(s1_binary_send(b)==E_OK && s1_telemetry_take(out)==(int)TELEMETRY_BYTES && telemetry_decode(out,got));
        assert(got[TEL_QUALITY]==16 && got[TEL_IMU_FAULT]==0 && s1_drop_count()==d+4U);
        /* Only a stale one queued: nothing to send, its flags wait for the next. */
        assert(s1_binary_send(a)==E_OK && s1_telemetry_take(out)==E_TMOUT && s1_drop_count()==d+5U);
        assert(s1_binary_send(b)==E_OK && s1_telemetry_take(out)==(int)TELEMETRY_BYTES && telemetry_decode(out,got));
        assert(got[TEL_QUALITY]==(2|16) && got[TEL_IMU_FAULT]==1);
        assert(s1_binary_send(b)==E_OK && s1_telemetry_take(out)==(int)TELEMETRY_BYTES && telemetry_decode(out,got));
        assert(got[TEL_QUALITY]==16 && got[TEL_IMU_FAULT]==0);
        d=(uint16_t)(d+1U);
        /* A snapshot the network abandoned part-way is counted too. */
        s1_telemetry_dropped();assert(s1_drop_count()==d+5U);
        /* Every drop is one atomic increment (both cores count drops). */
        unsigned calls=atomic_calls;s1_telemetry_dropped();assert(atomic_calls==calls+1U);
    }
    /* Command replies. Commands queued before a STOP are answered "rejected"
     * (1) when the VC drains them; a reply the VC does not make carries the
     * last published phase and fault. */
    {
        unsigned char ack[CSC02_ACK_BYTES];control_message_t c;
        while(s1_command_ack_take(ack)>0) {}
        test_phase=12;test_fault=8;
        unsigned char start[6]={0xC1,3,30,0,0,20},stop[6]={0xC1,0,0,0,0,21};
        assert(s1_command_offer(start,6)==E_OK && s1_command_offer(stop,6)==E_OK);
        assert(s1_command_take(&c)==E_OK && c.wire[1]==0 && c.wire[5]==21);
        assert(s1_command_ack_take(ack)==E_TMOUT); /* the VC answers its STOP itself */
        assert(s1_command_take(&c)==E_TMOUT);
        assert(s1_command_ack_take(ack)==(int)CSC02_ACK_BYTES && ack_is(ack,20,3,1,12,8));
        assert(s1_command_ack_take(ack)==E_TMOUT);
        /* Two STOPs before the VC takes one: it stops once; the first STOP is
         * answered at once (accepted), the second by the VC. */
        stop[5]=30;assert(s1_command_offer(stop,6)==E_OK && s1_command_ack_take(ack)==E_TMOUT);
        stop[5]=31;assert(s1_command_offer(stop,6)==E_OK);
        assert(s1_command_ack_take(ack)==(int)CSC02_ACK_BYTES && ack_is(ack,30,0,0,12,8));
        assert(s1_command_take(&c)==E_OK && c.wire[1]==0 && c.wire[5]==31);
        assert(s1_command_take(&c)==E_TMOUT && s1_command_ack_take(ack)==E_TMOUT);
        /* A command offered after the STOP is not superseded. */
        start[5]=22;assert(s1_command_offer(start,6)==E_OK && s1_command_take(&c)==E_OK && c.wire[5]==22);
        s1_command_reply(5,2,2);
        assert(s1_command_ack_take(ack)==(int)CSC02_ACK_BYTES && ack_is(ack,5,2,2,12,8));
        /* A full reply buffer (no broker) drops its oldest reply, counted:
         * the newest is the one a waiting dashboard can use. */
        for(unsigned n=0;n<8U;++n)s1_command_ack(100U+n,4,0,0,0);
        uint16_t d=s1_drop_count();s1_command_ack(108,4,0,0,0);assert(s1_drop_count()==d+1U);
        for(unsigned n=1;n<=8U;++n)assert(s1_command_ack_take(ack)==(int)CSC02_ACK_BYTES && ack[1]==100U+n);
        assert(s1_command_ack_take(ack)==E_TMOUT);
        test_phase=test_fault=0;
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
    puts("Snapshots: newest first, older/stale and full-queue drops counted, impact and IMU fault handed on PASS");
    puts("Command replies: superseded commands rejected with the VC state, two STOPs both answered, full reply buffer drops the oldest PASS");
    return 0;
}
