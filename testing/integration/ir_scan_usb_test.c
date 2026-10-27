/* Actual USB parser with a fake inbox; no kernel, serial port or motor calls. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define CSC02_CONTROL_H
#define ROBOT_CONFIG_H
#define BOARD_IR_MOTOR_SCAN_BENCH 1
#include "../../app_program/ir_motor_scan.h"
static unsigned offered,stops;static int fail_offer;
static uint8_t copied[6];
int s1_command_offer(const unsigned char *w,unsigned n) {
    assert(n==6U && w[0]==0xC1U && w[1]==0U);stops++;return 0;
}
int s1_scan_command_offer(const uint8_t w[6]) {
    if(!ir_scan_frame_valid(w) || fail_offer)return -1;
    offered++;memcpy(copied,w,6U);return 0;
}
#include "../../app_program/subsystem1/usb_control.c"
static void feed(const uint8_t *p,unsigned n){for(unsigned i=0U;i<n;i++)app_usb_rx_byte(p[i]);}
int main(void) {
    const uint8_t prepare[6]={0xD3U,IR_SCAN_PREPARE,0U,0U,0U,0U};
    const uint8_t pulse[6]={0xD3U,IR_SCAN_PULSE,15U,200U,0U,1U};
    const uint8_t arm[6]={0xC1U,1U,30U,0xF4U,1U,1U};
    feed((const uint8_t *)"agvr?",5U);feed(arm,6U);assert(offered==0U && stops==0U);
    feed(prepare,6U);assert(offered==1U && memcmp(copied,prepare,6U)==0);
    feed(pulse,6U);assert(offered==2U && memcmp(copied,pulse,6U)==0);
    feed(prepare,3U);app_usb_rx_byte('s');assert(stops==1U);
    feed(prepare,6U);assert(offered==3U); /* Partial frame cannot trap STOP. */
    uint8_t bad[6]={0xD3U,IR_SCAN_RUN,100U,0x88U,0x13U,2U};
    feed(bad,6U);assert(offered==3U && stops==2U);
    fail_offer=1;feed(pulse,6U);assert(stops==3U && offered==3U);
    fail_offer=0;app_usb_rx_byte('s');assert(stops==4U);
    puts("IR scan USB: actual framing, old ARM rejection, partial-frame STOP, bad power and queue-failure STOP PASS");
    return 0;
}
