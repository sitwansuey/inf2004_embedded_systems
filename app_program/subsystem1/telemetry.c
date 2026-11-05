/* Copied nonblocking handoff, derived from Buddy 1's telemetry inbox design. */
#include "subsystem1.h"
#include "control.h"
#if BOARD_WHEEL_CAL_BENCH
#include "../wheel_calibration_policy.h"
#define COMMAND_MAX_ARM_LEASE_MS WHEEL_CAL_MAX_LEASE_MS
#else
#define COMMAND_MAX_ARM_LEASE_MS 1000U
#endif
static ID telemetry_buffer, command_buffer, ack_buffer, diagnostic_buffer;
static uint32_t diagnostic_drops;
static UW stop_sequence;
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
    c.maxmsz=CSC02_ACK_BYTES;c.bufsz=8*(CSC02_ACK_BYTES+8U);
    ack_buffer=tk_cre_mbf(&c);
    if(ack_buffer<0) {
        ER er=ack_buffer;tk_del_mbf(command_buffer);tk_del_mbf(telemetry_buffer);
        command_buffer=telemetry_buffer=ack_buffer=0;return er;
    }
    c.maxmsz=BARCODE_DIAGNOSTIC_BYTES;c.bufsz=4*(BARCODE_DIAGNOSTIC_BYTES+4U);
    diagnostic_buffer=tk_cre_mbf(&c);
    if(diagnostic_buffer<E_OK) {
        ER er=diagnostic_buffer;tk_del_mbf(ack_buffer);tk_del_mbf(command_buffer);tk_del_mbf(telemetry_buffer);
        diagnostic_buffer=ack_buffer=command_buffer=telemetry_buffer=0;return er;
    }
    return E_OK;
}
int s1_diagnostic_offer(const unsigned char *data,unsigned length) {
    if(!data || length!=BARCODE_DIAGNOSTIC_BYTES || data[0]!=0xBC || data[1]!=1)return E_PAR;
    if(diagnostic_buffer<=0)return E_OBJ;
    uint8_t copy[BARCODE_DIAGNOSTIC_BYTES];memcpy(copy,data,sizeof(copy));
    barcode_diag_put32(copy+44U,diagnostic_drops);
    ER er=tk_snd_mbf(diagnostic_buffer,copy,sizeof(copy),TMO_POL);
    if(er<E_OK && diagnostic_drops!=UINT32_MAX)++diagnostic_drops;
    return er;
}
int s1_diagnostic_take(unsigned char *out) {
    if(!out || diagnostic_buffer<=0)return E_OBJ;
    return tk_rcv_mbf(diagnostic_buffer,out,TMO_POL);
}
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault) {
    unsigned char ack[CSC02_ACK_BYTES]={0xC2,(unsigned char)seq,(unsigned char)opcode,
        (unsigned char)result,(unsigned char)phase,(unsigned char)fault};
    if(ack_buffer>0)(void)tk_snd_mbf(ack_buffer,ack,sizeof(ack),TMO_POL);
}
int s1_command_ack_take(unsigned char *out) {
    if(!out || ack_buffer<=0)return E_OBJ;
    return tk_rcv_mbf(ack_buffer,out,TMO_POL);
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
    if(!data || length!=CSC02_COMMAND_BYTES || data[0]!=0xC1 || data[1]>3)return E_PAR;
    if(data[1]==0) { __atomic_store_n(&stop_sequence,data[5],__ATOMIC_RELEASE); /* STOP survives a full queue and is handled before ARM. */
        UW n=__atomic_load_n(&stop_epoch,__ATOMIC_RELAXED);
        __atomic_store_n(&stop_epoch,n+1U,__ATOMIC_RELEASE);return E_OK;
    }
    unsigned lease=(unsigned)data[3]|((unsigned)data[4]<<8);
    if(data[1]==1 && (data[2]==0 || data[2]>50 || lease==0 || lease>COMMAND_MAX_ARM_LEASE_MS))return E_PAR;
    if(data[1]==2 && (data[2] || lease))return E_PAR;
    if(data[1]==3 && (data[2]==0 || data[2]>50 || lease))return E_PAR;
    if(command_buffer<=0)return E_OBJ;
    SYSTIM now;ER er=tk_get_otm(&now);if(er<0)return er;
    control_message_t m={.time_ms=now.lo,.epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE)};
    for(unsigned i=0;i<CSC02_COMMAND_BYTES;++i)m.wire[i]=data[i];
    er=tk_snd_mbf(command_buffer,&m,sizeof(m),TMO_POL);if(er<0)dropped();return er;
}
/* Copied through the same bounded command queue; only the vehicle owner may
 * install a map and acknowledge its application. A mapping never arms motors. */
int s1_barcode_map_offer(const unsigned char letters[BARCODE_MAP_BYTES]) {
    if (!barcode_map_valid(letters)) return E_PAR;
    if (command_buffer<=0) return E_OBJ;
    SYSTIM now;ER er=tk_get_otm(&now);if(er<E_OK)return er;
    control_message_t m={.time_ms=now.lo,.epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE)};
    m.wire[0]=0xC3;
    for(unsigned n=0;n<BARCODE_MAP_BYTES;++n)m.wire[2+n]=letters[n];
    return tk_snd_mbf(command_buffer,&m,sizeof(m),TMO_POL);
}
void s1_barcode_map_ack(unsigned result,const unsigned char letters[BARCODE_MAP_BYTES]) {
    unsigned char ack[CSC02_ACK_BYTES]={0xC4,(unsigned char)result,0,0,0,0};
    if(letters)for(unsigned n=0;n<BARCODE_MAP_BYTES;++n)ack[2+n]=letters[n];
    if(ack_buffer>0)(void)tk_snd_mbf(ack_buffer,ack,sizeof(ack),TMO_POL);
}
ER s1_command_take(control_message_t *p) {
    if(!p)return E_PAR;
    *p=(control_message_t){0};
    UW epoch=__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE);
    if(epoch!=seen_stop) { seen_stop=epoch;p->wire[0]=0xC1;p->wire[5]=(uint8_t)__atomic_load_n(&stop_sequence,__ATOMIC_ACQUIRE);return E_OK; }
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
