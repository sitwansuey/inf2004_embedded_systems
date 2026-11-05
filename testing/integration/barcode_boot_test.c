#include "../../app_program/barcode_boot.h"
#include "../../app_program/vehicle_policy.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    barcode_boot_t b={0};
    for(uint32_t t=0;t<10000;t+=10)assert(!barcode_boot_step(&b,t,false,true));
    for(uint32_t t=10000;t<13000;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    assert(barcode_boot_step(&b,13000,true,true));
    for(uint32_t t=13010;t<20000;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    b=(barcode_boot_t){0};assert(!barcode_boot_step(&b,0,true,true));
    barcode_boot_cancel(&b);
    for(uint32_t t=10;t<10000;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    b=(barcode_boot_t){0};
    for(uint32_t t=0;t<2990;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    assert(!barcode_boot_step(&b,2990,true,false));
    for(uint32_t t=3000;t<6000;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    assert(barcode_boot_step(&b,6000,true,true));
    b=(barcode_boot_t){0};
    for(uint32_t t=0;t<2990;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    assert(!barcode_boot_step(&b,3020,true,true)); /* a clock gap restarts the delay */
    for(uint32_t t=3030;t<6020;t+=10)assert(!barcode_boot_step(&b,t,true,true));
    assert(barcode_boot_step(&b,6020,true,true));
    b=(barcode_boot_t){0};uint32_t start=UINT32_MAX-1000U;
    for(uint32_t n=0;n<3000;n+=10)assert(!barcode_boot_step(&b,start+n,true,true));
    assert(barcode_boot_step(&b,start+3000,true,true));
    /* The demo deliberately does not require PID/terrain/sonar, but retains
     * real line, motor and encoder validity and all explicit STOP/fault rules. */
    for(unsigned mask=0;mask<4;++mask)for(unsigned scanning=0;scanning<2;++scanning) {
        vc_state_t s={0};assert(vc_start(&s,0,30));
        vc_input_t i={.now_ms=10,.motor_ok=1,.odom_valid=1,.transport_ok=1,
            .line_fresh=1,.steering=1,.barcode_demo=1,.barcode_scanning=scanning,
            .line_black_mask=(uint8_t)mask};
        vc_step(&s,&i);assert(s.armed && s.power==30);
        assert(s.command==(scanning?1:mask==1?5:mask==2?6:1));
        i.line_fresh=0;vc_step(&s,&i);assert(!s.armed && !s.command);
    }
    vc_state_t s={0};assert(vc_start(&s,0,30));
    vc_input_t i={.now_ms=10,.motor_ok=1,.odom_valid=0,.transport_ok=1,
        .line_fresh=1,.steering=1,.barcode_demo=1};
    vc_step(&s,&i);assert(s.phase==VC_FAULT && !s.command);
    puts("BarcodeDemo boot/map/ready/delay/STOP/wrap, open-loop steering and straight capture, stale line/encoder STOP PASS");
    return 0;
}
