/* Actual MQTT port with a fake transport: no network, radio or broker claim. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"
static mqtt_client_t fake_client;
static unsigned connects,timers,publishes,offers,queued,publish_size;
static unsigned map_offers,map_acks,command_acks,map_subscriptions;
static int map_offer_error;
static unsigned diagnostic_available,diagnostic_publishes;
static done_t diagnostic_callback;
static void *diagnostic_argument;
static uint32_t fields_seen;
int s1_diagnostic_take(unsigned char *out) {
    if(!diagnostic_available)return -50;
    memset(out,0,BARCODE_DIAGNOSTIC_BYTES);out[0]=0xBC;out[1]=1;
    --diagnostic_available;return BARCODE_DIAGNOSTIC_BYTES;
}
static uint8_t mapped[4],map_ack[6],ack_pending;
void s1_command_ack(unsigned seq,unsigned op,unsigned result,unsigned phase,unsigned fault) {
    (void)seq;(void)op;(void)result;(void)phase;(void)fault;++command_acks;
}
int s1_command_ack_take(unsigned char *out) {
    if(!ack_pending)return -50;
    memcpy(out,map_ack,6);ack_pending=0;return 6;
}
int s1_barcode_map_offer(const unsigned char letters[4]) {
    assert(barcode_map_valid(letters));memcpy(mapped,letters,4);++map_offers;return map_offer_error;
}
void s1_barcode_map_ack(unsigned result,const unsigned char letters[4]) {
    map_ack[0]=0xC4;map_ack[1]=(uint8_t)result;
    memset(map_ack+2,0,4);if(letters)memcpy(map_ack+2,letters,4);ack_pending=1;
}
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
    (void)c;(void)cb;(void)arg;
    if(!strcmp(topic,CSC02_BARCODE_DIAGNOSTIC_TOPIC)) {
        assert(size==BARCODE_DIAGNOSTIC_BYTES && qos==1 && !retain && ((const uint8_t *)data)[0]==0xBC);
        ++diagnostic_publishes;
        if(!publish_error){diagnostic_callback=cb;diagnostic_argument=arg;}
        return publish_error;
    }
    if(!strcmp(topic,CSC02_BARCODE_MAP_ACK_TOPIC)) {
        assert(size==6 && qos==0 && retain==0 && ((const uint8_t *)data)[0]==0xC4);
        ++map_acks;return publish_error;
    }
    assert(!strcmp(topic,EDR_MQTT_TOPIC) && qos==0 && retain==0);
    unsigned id,value;assert(size==2 && edr_decode(data,size,&id,&value));
    if(!publish_error)fields_seen|=UINT32_C(1)<<(id?id-1U:15U+(value>>8));
    ++publishes;publish_size=size;memcpy(sent,data,size);return publish_error;
}
err_t mqtt_subscribe(mqtt_client_t *c,const char *topic,unsigned qos,done_t cb,void *arg) {
    (void)c;(void)cb;(void)arg;
    if(!strcmp(topic,CSC02_BARCODE_MAP_TOPIC)){assert(qos==1);++map_subscriptions;}
    else assert(!strcmp(topic,"csc02/command") && qos==0);
    return ERR_OK;
}
void mqtt_set_inpub_callback(mqtt_client_t *c,void (*start)(void *,const char *,u32_t),void (*data)(void *,const u8_t *,u16_t,u8_t),void *arg) {(void)c;(void)start;(void)data;(void)arg;}
void sys_timeout(unsigned ms,void (*cb)(void *),void *arg) {(void)cb;(void)arg;assert(ms==3000 && timers==0);++timers;}
void sys_untimeout(void (*cb)(void *),void *arg) {(void)cb;(void)arg;timers=0;}
int s1_command_offer(const unsigned char *data,unsigned size) {assert(size==6);memcpy(offered,data,6);++offers;return 0;}
int s1_telemetry_take(unsigned char *data) {
    if(!queued)return -50;
    int64_t v[TEL_FIELD_COUNT]={0};v[TEL_VERSION]=1;v[TEL_AVAILABLE]=255;v[TEL_QUALITY]=63;
    assert(telemetry_encode(data,v));--queued;return TELEMETRY_BYTES;
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
    for(unsigned n=0;n<8;++n)mqtt_utk_poll_telemetry();
    assert(publishes==31 && publish_size==2); /* every sensor and status, no aggregate */
    queued=1;publish_error=ERR_MEM;mqtt_utk_poll_telemetry();assert(publishes==32);
    unsigned char blocked[2];memcpy(blocked,sent,2);
    publish_error=0;mqtt_utk_poll_telemetry();assert(publishes==36);
    /* The pending snapshot expires before later records can be sent. */
    fresh=0;unsigned before=publishes;mqtt_utk_poll_telemetry();assert(publishes==before);
    /* Slow transport: every poll outlives a snapshot. Refresh values without
     * restarting the field cursor; all31 fields must still get a turn. */
    fields_seen=0;
    for(unsigned n=0;n<16;++n){queued=1;mqtt_utk_poll_telemetry();}
    assert(fields_seen==UINT32_C(0x7fffffff));fresh=1;
    (void)blocked;
    fake_client.connected=0;connection_cb(client,NULL,2);assert(offers==2 && timers==1);
    for(unsigned n=0;n<20;++n)mqtt_utk_start();
    assert(connects==1 && timers==1);
    timers=0;retry_timeout(NULL);assert(connects==2);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);deliver(arm);assert(offers==2);
    assert(map_subscriptions==2);
    const char *map="{\"right\":\"Z\",\"forward\":\"E\",\"left\":\"Q\",\"uturn\":\"M\"}";
    for(unsigned split=0;split<=strlen(map);++split) {
        incoming_publish_cb(NULL,CSC02_BARCODE_MAP_TOPIC,(u32_t)strlen(map));
        incoming_data_cb(NULL,(const u8_t *)map,(u16_t)split,0);
        incoming_data_cb(NULL,(const u8_t *)map+split,(u16_t)(strlen(map)-split),MQTT_DATA_FLAG_LAST);
        assert(!memcmp(mapped,"EMQZ",4));
    }
    assert(map_offers==strlen(map)+1);
    incoming_publish_cb(NULL,CSC02_BARCODE_MAP_TOPIC,129);
    incoming_data_cb(NULL,(const u8_t *)map,1,MQTT_DATA_FLAG_LAST);
    assert(map_offers==strlen(map)+1);
    incoming_publish_cb(NULL,CSC02_BARCODE_MAP_TOPIC,2);
    incoming_data_cb(NULL,(const u8_t *)"{}",2,MQTT_DATA_FLAG_LAST);
    assert(map_ack[1]==BARCODE_MAP_INVALID);
    mqtt_utk_poll_telemetry();assert(map_acks==1);
    map_offer_error=-50;
    incoming_publish_cb(NULL,CSC02_BARCODE_MAP_TOPIC,(u32_t)strlen(map));
    incoming_data_cb(NULL,(const u8_t *)map,(u16_t)strlen(map),MQTT_DATA_FLAG_LAST);
    assert(map_ack[1]==BARCODE_MAP_QUEUE_FULL);
    mqtt_utk_poll_telemetry();assert(map_acks==2);
    /* A disconnect invalidates a partial publication; its suffix cannot install a map. */
    unsigned previous_maps=map_offers;
    incoming_publish_cb(NULL,CSC02_BARCODE_MAP_TOPIC,(u32_t)strlen(map));
    incoming_data_cb(NULL,(const u8_t *)map,1,0);
    connection_cb(client,NULL,2);
    incoming_data_cb(NULL,(const u8_t *)map+1,(u16_t)(strlen(map)-1),MQTT_DATA_FLAG_LAST);
    assert(map_offers==previous_maps);
    fake_client.connected=1;diagnostic_turn=1;diagnostic_available=2;
    queued=8;mqtt_utk_poll_telemetry();assert(diagnostic_publishes==1 && diagnostic_inflight);
    before=publishes;mqtt_utk_poll_telemetry();assert(publishes>before && diagnostic_publishes==1);
    diagnostic_callback(diagnostic_argument,ERR_OK);
    mqtt_utk_poll_telemetry();assert(diagnostic_publishes==2 && diagnostic_inflight);
    diagnostic_callback(diagnostic_argument,ERR_MEM); /* Failed delivery keeps the exact frame. */
    mqtt_utk_poll_telemetry();mqtt_utk_poll_telemetry();assert(diagnostic_publishes==3);
    diagnostic_callback(diagnostic_argument,ERR_OK);
    puts("INT-CR1-07 actual MQTT port: binary length/NULs, fragments, malformed input, backpressure, autonomous link-loss independence and retry guard PASS");
    return 0;
}
