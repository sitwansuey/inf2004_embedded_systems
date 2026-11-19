#include "control.h"
/* Called only by the existing USB owner. No TinyUSB calls from this module.
 * Six-byte command frames share the exact MQTT decoder. USB sequences have
 * bit 7 set (CSC02_USB_SEQUENCE): their ACKs, also published on MQTT, never
 * match a dashboard command, whose sequences stay 0..127. */
void app_usb_rx_byte(unsigned char byte) {
    static unsigned used;static unsigned char frame[CSC02_COMMAND_BYTES];
    static uint8_t sequence;
    if(used==0 && byte!=0xC1) {
        if(byte=='s' || byte=='a' || byte=='r' || byte=='g') {
            unsigned char c[6]={0xC1,0,0,0,0,0};c[5]=(unsigned char)(CSC02_USB_SEQUENCE|(sequence++&0x7FU));
            if(byte=='a') {c[1]=1;c[2]=30;c[3]=0xF4;c[4]=1;} /* 500 ms lease */
            if(byte=='r')c[1]=2;
            if(byte=='g') {c[1]=3;c[2]=30;} /* one-shot autonomous START */
            (void)s1_command_offer(c,sizeof(c));
        }
        return;
    }
    frame[used++]=byte;
    if(used==sizeof(frame)) {used=0;frame[5]=(unsigned char)(frame[5]|CSC02_USB_SEQUENCE);(void)s1_command_offer(frame,sizeof(frame));}
}
