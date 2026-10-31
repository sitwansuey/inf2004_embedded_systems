/* Reuse the real handoff fixture, then test USB-only bench records in that queue. */
#define main ordinary_queue_tests
#include "telemetry_queue_test.c"
#undef main
int main(void) {
    assert(ordinary_queue_tests()==0);
    uint8_t scan[6]={0xD3U,IR_SCAN_RUN,15U,0x88U,0x13U,1U};
    uint8_t stop[6]={0xC1U,0U,0U,0U,0U,0U};control_message_t c;
    assert(s1_command_offer(scan,6U)==E_PAR); /* MQTT/shared C1 decoder cannot start it. */
    assert(s1_scan_command_offer(scan)==E_OK);scan[2]=20U;
    assert(s1_command_take(&c)==E_OK && c.wire[0]==0xD3U && c.wire[2]==15U && c.time_ms==500U);
    for(unsigned n=0U;n<4U;n++)assert(s1_scan_command_offer(scan)==E_OK);
    assert(s1_scan_command_offer(scan)==E_TMOUT);
    assert(s1_command_offer(stop,6U)==E_OK);
    assert(s1_command_take(&c)==E_OK && c.wire[1]==0U);
    assert(s1_command_take(&c)==E_TMOUT); /* STOP epoch discards every queued START/renewal. */
    scan[2]=21U;assert(s1_scan_command_offer(scan)==E_PAR);
    puts("IR scan actual inbox: USB-only discriminator, copies/timestamps, saturation and urgent STOP epoch PASS");
    return 0;
}
