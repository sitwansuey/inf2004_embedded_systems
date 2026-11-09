/* Actual MQTT port with a fake transport: no network, radio or broker claim. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"
static mqtt_client_t fake_client;
static unsigned connects,timers,publishes,offers,queued,publish_size;
static uint64_t seen_fields;
static int publish_error, fresh=1;
static unsigned char sent[TELEMETRY_BYTES], offered[6];
int tm_printf(const UB *format,...) {(void)format;return 0;}
mqtt_client_t *mqtt_client_new(void) {return &fake_client;}
int mqtt_client_is_connected(mqtt_client_t *c) {return c->connected;}
int ip4addr_aton(const char *s,ip_addr_t *ip) {(void)s;ip->value=0;return 1;}
err_t mqtt_client_connect(mqtt_client_t *c,const void *ip,unsigned port,void (*cb)(mqtt_client_t *,void *,mqtt_connection_status_t),void *arg,const struct mqtt_connect_client_info_t *info) {
    (void)c;(void)ip;(void)port;(void)cb;(void)arg;assert(info->keep_alive==60);++connects;return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t *c,const char *topic,const void *data,u16_t size,unsigned qos,unsigned retain,done_t cb,void *arg) {
    (void)c;(void)cb;(void)arg;assert(!strcmp(topic,EDR_MQTT_TOPIC) && qos==0 && retain==0);
    unsigned id=0,value=0;assert(size==2 && edr_decode(data,size,&id,&value));
    /* IDs 1..15, status subtypes 0..15, then paged subtype 8 left/right halves. */
    unsigned bit=id?id-1U:(value>>8U)==8U && (value&255U)>=128U?31U+((value>>6U)&1U):15U+(value>>8U);
    if(publish_error==ERR_OK)seen_fields|=UINT64_C(1)<<bit;
    ++publishes;publish_size=size;memcpy(sent,data,size);return publish_error;
}
err_t mqtt_subscribe(mqtt_client_t *c,const char *topic,unsigned qos,done_t cb,void *arg) {
    (void)c;(void)qos;(void)cb;(void)arg;assert(!strcmp(topic,"csc02/command"));return ERR_OK;
}
void mqtt_set_inpub_callback(mqtt_client_t *c,void (*start)(void *,const char *,u32_t),void (*data)(void *,const u8_t *,u16_t,u8_t),void *arg) {(void)c;(void)start;(void)data;(void)arg;}
void sys_timeout(unsigned ms,void (*cb)(void *),void *arg) {(void)cb;(void)arg;assert(ms==3000 && timers==0);++timers;}
void sys_untimeout(void (*cb)(void *),void *arg) {(void)cb;(void)arg;timers=0;}
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault)
{ (void)seq;(void)opcode;(void)result;(void)phase;(void)fault; }
int s1_command_ack_take(unsigned char *out){(void)out;return -50;}
int s1_command_offer(const unsigned char *data,unsigned size) {assert(size==6);memcpy(offered,data,6);++offers;return 0;}
int s1_telemetry_take(unsigned char *data) {
    if(!queued)return -50;
    int64_t v[TEL_FIELD_COUNT]={0};v[TEL_VERSION]=1;v[TEL_AVAILABLE]=255;v[TEL_QUALITY]=63;
    assert(telemetry_encode(data,v));--queued;fresh=1;return TELEMETRY_BYTES;
}
int s1_telemetry_current(const unsigned char *data) {(void)data;return fresh;}
static void deliver(const unsigned char *cmd) {
    incoming_publish_cb(NULL,CSC02_COMMAND_TOPIC,6);
    incoming_data_cb(NULL,cmd,2,0);incoming_data_cb(NULL,cmd+2,4,MQTT_DATA_FLAG_LAST);
}
int main(void) {
    mqtt_utk_start();mqtt_utk_start();assert(connects==1);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);
    unsigned char arm[6]={0xC1,1,30,0xF4,1,0},reset[6]={0xC1,2,0,0,0,0};
    deliver(arm);assert(offers==0); /* retained ARM alone cannot start after reconnect */
    deliver(reset);assert(offers==1);deliver(arm);assert(offers==2 && offered[1]==1);
    incoming_publish_cb(NULL,CSC02_COMMAND_TOPIC,7);incoming_data_cb(NULL,arm,6,MQTT_DATA_FLAG_LAST);assert(offers==2);
    incoming_publish_cb(NULL,CSC02_COMMAND_TOPIC,6);incoming_data_cb(NULL,arm,2,MQTT_DATA_FLAG_LAST);assert(offers==2);
    incoming_publish_cb(NULL,"unrelated",6);incoming_data_cb(NULL,arm,6,MQTT_DATA_FLAG_LAST);assert(offers==2);
    queued=1;
    for(unsigned n=0;n<9;++n)mqtt_utk_poll_telemetry();
    assert(publishes==TELEMETRY_EDR_RECORDS && publish_size==2); /* every sensor and status, no aggregate */
    queued=1;publish_error=ERR_MEM;mqtt_utk_poll_telemetry();assert(publishes==TELEMETRY_EDR_RECORDS+1U);
    unsigned char blocked[2];memcpy(blocked,sent,2);
    publish_error=0;mqtt_utk_poll_telemetry();assert(publishes==TELEMETRY_EDR_RECORDS+5U);
    /* The pending snapshot expires before later records can be sent. */
    fresh=0;unsigned before=publishes;mqtt_utk_poll_telemetry();assert(publishes==before);
    (void)blocked;
    /* Expire every partially sent snapshot, then provide a fresh replacement.
     * All fields, including late width/quality records, must still arrive. */
    seen_fields=0;
    for(unsigned n=0;n<16U;++n){queued=1;fresh=0;mqtt_utk_poll_telemetry();}
    assert(seen_fields==(UINT64_C(1)<<TELEMETRY_EDR_RECORDS)-1U);
    fake_client.connected=0;connection_cb(client,NULL,2);assert(offers==2 && timers==1);
    for(unsigned n=0;n<20;++n)mqtt_utk_start();
    assert(connects==1 && timers==1);
    timers=0;retry_timeout(NULL);assert(connects==2);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);deliver(arm);assert(offers==2);
    puts("INT-CR1-07 actual MQTT port: binary length/NULs, fragments, malformed input, backpressure, autonomous link-loss independence and retry guard PASS");
    return 0;
}
