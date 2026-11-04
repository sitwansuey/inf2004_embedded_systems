/* Copied nonblocking handoff, derived from Buddy 1's telemetry inbox design. */
#include "subsystem1.h"
#include "control.h"
static ID telemetry_buffer, command_buffer;
static UW drops, stop_epoch, seen_stop;
static void dropped(void) { UW n=__atomic_load_n(&drops,__ATOMIC_RELAXED);
    __atomic_store_n(&drops,n<65535?n+1:n,__ATOMIC_RELAXED); }
uint16_t s1_drop_count(void) { return (uint16_t)__atomic_load_n(&drops,__ATOMIC_RELAXED); }
ER s1_binary_init(void) {
    if(telemetry_buffer>0 || command_buffer>0)return E_OBJ;
    T_CMBF c={0};c.mbfatr=TA_TFIFO;c.maxmsz=TELEMETRY_BYTES;
    c.bufsz=2*((TELEMETRY_BYTES+3U)/4U*4U+4U);
    telemetry_buffer=tk_cre_mbf(&c);if(telemetry_buffer<0)return telemetry_buffer;
    c.maxmsz=sizeof(control_message_t);c.bufsz=4*(sizeof(control_message_t)+4);
    command_buffer=tk_cre_mbf(&c);
    if(command_buffer<0) { ER er=command_buffer;tk_del_mbf(telemetry_buffer);telemetry_buffer=0;command_buffer=0;return er; }
    return E_OK;
}
ER s1_binary_send(const uint8_t data[TELEMETRY_BYTES]) {
    if(!data)return E_PAR;
    if(telemetry_buffer<=0)return E_OBJ;
    ER er=tk_snd_mbf(telemetry_buffer,data,TELEMETRY_BYTES,TMO_POL);
    if(er<0)dropped();
    return er;
}
int s1_telemetry_take(unsigned char *out) {
    if(!out || telemetry_buffer<=0)return E_OBJ;
    for(unsigned n=0;n<2;++n) {
        INT size=tk_rcv_mbf(telemetry_buffer,out,TMO_POL);
        if(size<0)return size;
        if(size==TELEMETRY_BYTES && s1_telemetry_current(out))return size;
        dropped();
    }
    return E_TMOUT;
}
int s1_telemetry_current(const unsigned char *data) {
    if(!data)return 0;
    /* Version(4) + sequence(16), followed by the full 32-bit kernel time. */
    uint32_t stamp=0;
    for(unsigned n=0;n<32;++n)stamp|=(uint32_t)((data[(20+n)/8]>>((20+n)%8))&1U)<<n;
    SYSTIM now;
    return tk_get_otm(&now)==E_OK && now.lo-stamp<500U;
}
int s1_command_offer(const unsigned char *data,unsigned length) {
    if(!data || length!=CSC02_COMMAND_BYTES || data[0]!=0xC1 || data[1]>2)return E_PAR;
    if(data[1]==0) { /* STOP survives a full queue and is handled before ARM. */
        UW n=__atomic_load_n(&stop_epoch,__ATOMIC_RELAXED);
        __atomic_store_n(&stop_epoch,n+1U,__ATOMIC_RELEASE);return E_OK;
    }
    unsigned lease=(unsigned)data[3]|((unsigned)data[4]<<8);
    if(data[1]==1 && (data[2]==0 || data[2]>50 || lease==0 || lease>1000))return E_PAR;
    if(data[1]==2 && (data[2] || lease))return E_PAR;
    if(command_buffer<=0)return E_OBJ;
    SYSTIM now;ER er=tk_get_otm(&now);if(er<0)return er;
    control_message_t m={.time_ms=now.lo,.epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE)};
    for(unsigned i=0;i<CSC02_COMMAND_BYTES;++i)m.wire[i]=data[i];
    er=tk_snd_mbf(command_buffer,&m,sizeof(m),TMO_POL);if(er<0)dropped();return er;
}
ER s1_command_take(control_message_t *p) {
    if(!p)return E_PAR;
    *p=(control_message_t){0};
    UW epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE);
    if(epoch!=seen_stop) { seen_stop=epoch;p->wire[0]=0xC1;return E_OK; }
    if(command_buffer<=0)return E_OBJ;
    for(unsigned n=0;n<4;++n) {
        INT size=tk_rcv_mbf(command_buffer,p,TMO_POL);if(size<0)return size;
        if(size==sizeof(*p) && p->epoch==epoch)return E_OK;
    }
    return E_TMOUT;
}

#if BOARD_IR_MOTOR_SCAN_BENCH
#include "../ir_motor_scan.h"
int s1_scan_command_offer(const uint8_t wire[6]) {
    SYSTIM now;
    if(!ir_scan_frame_valid(wire)) return E_PAR;
    if(command_buffer<=0) return E_OBJ;
    ER er=tk_get_otm(&now);if(er<E_OK)return er;
    control_message_t m={.time_ms=now.lo,.epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE)};
    for(unsigned i=0;i<6U;++i)m.wire[i]=wire[i];
    er=tk_snd_mbf(command_buffer,&m,sizeof(m),TMO_POL);
    return er;
}
#endif
