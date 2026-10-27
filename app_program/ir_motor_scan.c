/* Explicit USB-only timed bench policy; no GPIO, kernel or navigation calls. */
#include "ir_motor_scan.h"
#include <stddef.h>
bool ir_scan_frame_valid(const uint8_t w[6]) {
    unsigned duration;
    if(w==NULL || w[0]!=0xD3U || w[5]>63U) return false;
    /* ASCII s is an urgent USB STOP at every framing position. */
    for(unsigned n=0U;n<6U;++n) if(w[n]==(uint8_t)'s') return false;
    duration=(unsigned)w[3]|((unsigned)w[4]<<8);
    if(w[1]>=IR_SCAN_RENEW && w[1]<=IR_SCAN_QUERY)
        return w[2]==0U && duration==0U;
    if(w[1]!=IR_SCAN_PULSE && w[1]!=IR_SCAN_RUN) return false;
    return w[2]>0U && w[2]<=IR_SCAN_MAX_POWER && duration>0U &&
           duration<=(w[1]==IR_SCAN_PULSE?IR_SCAN_PULSE_MAX_MS:IR_SCAN_MAX_MS);
}
void ir_scan_stop(ir_scan_state_t *s,ir_scan_reason_t reason) {
    s->active=0U;s->prepared=0U;s->power=0U;s->sequence_valid=0U;
    s->reason=(uint8_t)((unsigned)reason&15U);
}
void ir_scan_step(ir_scan_state_t *s,const ir_scan_input_t *i) {
    bool late=s->have_tick && i->now_ms-s->last_tick_ms>=30U;
    s->last_tick_ms=i->now_ms;s->have_tick=1U;
    if(s->prepared && i->now_ms-s->prepared_ms>=1000U) ir_scan_stop(s,IR_SCAN_USB_LOST);
    if(!s->active) return;
    if(late) ir_scan_stop(s,IR_SCAN_CLOCK);
    else if(!i->motor_ok) ir_scan_stop(s,IR_SCAN_MOTOR_FAULT);
    else if(!i->adc_ok) ir_scan_stop(s,IR_SCAN_ADC_LOST);
    else if(s->scan && !i->front_clear) ir_scan_stop(s,IR_SCAN_OBSTACLE);
    else if(i->now_ms-s->started_ms>=s->duration_ms) ir_scan_stop(s,IR_SCAN_COMPLETE);
    else if(i->now_ms-s->renewed_ms>=IR_SCAN_LEASE_MS) ir_scan_stop(s,IR_SCAN_USB_LOST);
}
bool ir_scan_command(ir_scan_state_t *s,const ir_scan_input_t *i,
                     const uint8_t w[6],uint32_t issued) {
    if(!ir_scan_frame_valid(w) || i->now_ms-issued>=100U) {
        ir_scan_stop(s,IR_SCAN_BAD_COMMAND);return false;
    }
    if(w[1]==IR_SCAN_QUERY) return false;
    if(w[1]==IR_SCAN_PREPARE) {
        if(s->active || !i->motor_ok || !i->adc_ok) {
            ir_scan_stop(s,IR_SCAN_BAD_COMMAND);return false;
        }
        s->prepared=1U;s->prepared_ms=i->now_ms;s->sequence=w[5]&63U;
        s->sequence_valid=1U;s->reason=IR_SCAN_IDLE;return false;
    }
    if(!s->sequence_valid || (!s->prepared && !s->active)) return false;
    if(w[5]==s->sequence) return false; /* Duplicate heartbeat cannot extend a lease. */
    if(w[5]!=(uint8_t)((s->sequence+1U)&63U)) {
        ir_scan_stop(s,IR_SCAN_BAD_COMMAND);return false;
    }
    s->sequence=w[5]&63U;
    if(w[1]==IR_SCAN_RENEW) {
        if(s->active && i->now_ms-s->renewed_ms<IR_SCAN_LEASE_MS) s->renewed_ms=i->now_ms;
        return false;
    }
    if(!s->prepared || s->active || i->now_ms-s->prepared_ms>=1000U ||
       !i->motor_ok || !i->adc_ok || (w[1]==IR_SCAN_RUN && (!i->white || !i->front_clear))) {
        ir_scan_stop(s,IR_SCAN_BAD_COMMAND);return false;
    }
    s->prepared=0U;s->active=1U;s->scan=w[1]==IR_SCAN_RUN;
    s->power=w[2]&IR_SCAN_POWER_MASK;s->duration_ms=(uint32_t)w[3]|((uint32_t)w[4]<<8);
    s->started_ms=i->now_ms;s->renewed_ms=i->now_ms;s->reason=IR_SCAN_IDLE;
    return true;
}
