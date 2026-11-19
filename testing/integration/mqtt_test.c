/* Actual MQTT port with a fake transport: no network, radio or broker claim. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lwip/apps/mqtt.h"
/* lwIP API the mock header lacks, used by mqtt_utk_link_lost(). */
enum {MQTT_CONNECT_DISCONNECTED=256};
void mqtt_disconnect(mqtt_client_t *c);
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"
/* The Wi-Fi re-join watchdog cyw43_utk.c runs on every poll (pure, inline). */
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
static mqtt_client_t fake_client;
static unsigned connects,timers,publishes,offers,queued,publish_size,disconnects;
static err_t connect_error;
static uint64_t seen_fields;
static int publish_error, fresh=1;
static unsigned char sent[TELEMETRY_BYTES], offered[6];
/* lwIP clock and the robot event queue (subsystem1/telemetry.c packs one
 * EDR16-format word per event: value<<4 | id). */
static u32_t now_ms=7500;
#define EVENT_MAX 16U
static unsigned event_ids[EVENT_MAX],event_values[EVENT_MAX],events_posted,events_taken,events_published,event_attempts;
static unsigned char event_log[EVENT_MAX][2];
u32_t sys_now(void) {return now_ms;}
void s1_event_post(unsigned id,unsigned value) {assert(events_posted<EVENT_MAX);event_ids[events_posted]=id;event_values[events_posted++]=value;}
int s1_event_take(unsigned char *out) {
    if(events_taken==events_posted)return -50;
    unsigned word=(event_values[events_taken]<<4U)|event_ids[events_taken];++events_taken;
    out[0]=(unsigned char)(word&255U);out[1]=(unsigned char)(word>>8U);return 2;
}
static bool event_was(unsigned n,unsigned id,unsigned value) {
    unsigned word=(unsigned)event_log[n][0]|((unsigned)event_log[n][1]<<8U);
    return n<events_published && (word&15U)==id && (word>>4U)==value;
}
int tm_printf(const UB *format,...) {(void)format;return 0;}
mqtt_client_t *mqtt_client_new(void) {return &fake_client;}
int mqtt_client_is_connected(mqtt_client_t *c) {return c->connected;}
int ip4addr_aton(const char *s,ip_addr_t *ip) {(void)s;ip->value=0;return 1;}
err_t mqtt_client_connect(mqtt_client_t *c,const void *ip,unsigned port,void (*cb)(mqtt_client_t *,void *,mqtt_connection_status_t),void *arg,const struct mqtt_connect_client_info_t *info) {
    (void)c;(void)ip;(void)port;(void)cb;(void)arg;assert(info->keep_alive==60);++connects;return connect_error;
}
void mqtt_disconnect(mqtt_client_t *c) {c->connected=0;++disconnects;}
err_t mqtt_publish(mqtt_client_t *c,const char *topic,const void *data,u16_t size,unsigned qos,unsigned retain,done_t cb,void *arg) {
    (void)c;(void)cb;(void)arg;
    if(!strcmp(topic,CSC02_EVENT_TOPIC)) { /* robot events: own topic, one record each */
        assert(size==2 && qos==0 && retain==0);++event_attempts;
        if(publish_error==ERR_OK) {assert(events_published<EVENT_MAX);memcpy(event_log[events_published++],data,2);}
        return publish_error;
    }
    assert(!strcmp(topic,EDR_MQTT_TOPIC) && qos==0 && retain==0);
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
/* One 20 ms cyw43_utk.c poll of the re-join watchdog at a CYW43_LINK_* status;
 * a leave drops the broker session exactly as cyw43_utk.c does. */
static T_WIFI_REJOIN wifi;
static uint64_t wifi_us;
static unsigned leaves,joins;
static int wifi_poll(int status) {
    wifi_us+=20000U;now_ms=(u32_t)(wifi_us/1000U);
    int r=wifi_rejoin_step(&wifi,status==3,status<0,wifi_us);
    if(r==WIFI_REJOIN_LEAVE){++leaves;mqtt_utk_link_lost();}
    if(r==WIFI_REJOIN_JOIN)++joins;
    return r;
}
static void wifi_hold(int status,unsigned polls) {for(unsigned n=0;n<polls;++n)assert(wifi_poll(status)==WIFI_REJOIN_WAIT);}
int main(void) {
    mqtt_utk_start();mqtt_utk_start();assert(connects==1);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);
    /* First broker connection: EV_LINK boot event with the seconds since boot. */
    assert(events_posted==1 && event_ids[0]==EV_LINK && event_values[0]==ev_link(EV_LINK_BOOT,7U));
    unsigned char arm[6]={0xC1,1,30,0xF4,1,0},reset[6]={0xC1,2,0,0,0,0};
    deliver(arm);assert(offers==0); /* retained ARM alone cannot start after reconnect */
    deliver(reset);assert(offers==1);deliver(arm);assert(offers==2 && offered[1]==1);
    incoming_publish_cb(NULL,CSC02_COMMAND_TOPIC,7);incoming_data_cb(NULL,arm,6,MQTT_DATA_FLAG_LAST);assert(offers==2);
    incoming_publish_cb(NULL,CSC02_COMMAND_TOPIC,6);incoming_data_cb(NULL,arm,2,MQTT_DATA_FLAG_LAST);assert(offers==2);
    incoming_publish_cb(NULL,"unrelated",6);incoming_data_cb(NULL,arm,6,MQTT_DATA_FLAG_LAST);assert(offers==2);
    queued=1;
    for(unsigned n=0;n<9;++n)mqtt_utk_poll_telemetry();
    assert(publishes==TELEMETRY_EDR_RECORDS && publish_size==2); /* every sensor and status, no aggregate */
    assert(events_published==1 && event_was(0,EV_LINK,ev_link(EV_LINK_BOOT,7U))); /* on the event topic */
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
    /* Events raised while the link is down wait in the queue. */
    now_ms+=4000U;s1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U));
    mqtt_utk_poll_telemetry();assert(events_published==1 && events_taken==1);
    timers=0;retry_timeout(NULL);assert(connects==2);
    now_ms+=8400U; /* 12.4 s after the loss */
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);deliver(arm);assert(offers==2);
    assert(events_posted==3 && event_ids[2]==EV_LINK && event_values[2]==ev_link(EV_LINK_RECONNECTED,12U));
    /* One event per poll, in order; backpressure keeps the exact record. */
    publish_error=ERR_MEM;mqtt_utk_poll_telemetry();assert(events_published==1 && event_attempts==2);
    publish_error=ERR_OK;mqtt_utk_poll_telemetry();
    assert(events_published==2 && event_was(1,EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U)));
    mqtt_utk_poll_telemetry();assert(events_published==3 && event_was(2,EV_LINK,ev_link(EV_LINK_RECONNECTED,12U)));
    mqtt_utk_poll_telemetry();assert(events_published==3 && events_taken==3);
    /* A drop right after connecting still reports the reconnection (0 s). */
    fake_client.connected=0;connection_cb(client,NULL,2);timers=0;retry_timeout(NULL);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);
    assert(events_posted==4 && event_values[3]==ev_link(EV_LINK_RECONNECTED,0U));
    connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);assert(events_posted==4); /* no loss in between: no event */
    unsigned char map[6]={0xC1,4,6,1,0,9};deliver(map);assert(offers==3 && offered[1]==4); /* MAP needs no RESET */
    mqtt_utk_poll_telemetry();assert(events_published==4 && event_was(3,EV_LINK,ev_link(EV_LINK_RECONNECTED,0U)));
    /* Wi-Fi lost while connected: lwIP closes the connection only when its
     * keep-alive watchdog expires, ~90 s later. Events stay queued (not handed
     * to lwIP, which drops them unsent on the close) once the broker has
     * acknowledged nothing for 3 s; the outage counts from the first publish
     * it never acknowledged. */
    publish_done_cb(NULL,ERR_OK); /* everything so far acknowledged */
    now_ms+=10000U;s1_event_post(EV_HUMP,ev_hump(EV_HUMP_DETECTED,52U));
    mqtt_utk_poll_telemetry();assert(events_published==5 && event_was(4,EV_HUMP,ev_hump(EV_HUMP_DETECTED,52U)));
    now_ms+=2900U;s1_event_post(EV_HUMP,ev_hump(EV_HUMP_CREST,40U));
    mqtt_utk_poll_telemetry();assert(events_published==6); /* 2.9 s unacknowledged: still sent */
    publish_done_cb(NULL,ERR_OK); /* the broker's last acknowledgement */
    now_ms+=100U;queued=1;mqtt_utk_poll_telemetry(); /* never acknowledged from here */
    now_ms+=3000U;s1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_DETECTED,0U,20U));
    for(unsigned n=0;n<92U;++n) {mqtt_utk_poll_telemetry();now_ms+=1000U;}
    assert(events_published==6 && events_taken==6); /* held in subsystem 1 */
    /* An acknowledgement while still connected lets it go. */
    publish_done_cb(NULL,ERR_OK);mqtt_utk_poll_telemetry();
    assert(events_published==7 && event_was(6,EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_DETECTED,0U,20U)));
    /* Unacknowledged again; the watchdog closes the connection 95 s later. */
    const u32_t silent_at=now_ms;
    now_ms+=3000U;s1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_SCANNED,1U,12U));
    for(unsigned n=0;n<92U;++n) {mqtt_utk_poll_telemetry();now_ms+=1000U;}
    assert(events_published==7 && events_taken==7);
    fake_client.connected=0;connection_cb(client,NULL,2);timers=0;retry_timeout(NULL);
    now_ms+=5000U;fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);
    assert(events_posted==9 && event_ids[8]==EV_LINK &&
           event_values[8]==ev_link(EV_LINK_RECONNECTED,(now_ms-silent_at)/1000U) && now_ms-silent_at==100000U);
    mqtt_utk_poll_telemetry();mqtt_utk_poll_telemetry();
    assert(events_published==9 && event_was(7,EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_SCANNED,1U,12U)) &&
           event_was(8,EV_LINK,ev_link(EV_LINK_RECONNECTED,100U)));
    /* Wi-Fi lost mid-run (cyw43_utk.c). A dropped association reads DOWN (0)
     * or JOIN (1), never the negative statuses the 5 s join retry handles.
     * Once the link has been UP, 10 s without UP leaves the AP and joins 1 s
     * later, for ever; the broker session goes with it, so the 3 s retry
     * reconnects once the link is back and EV_LINK reports the outage. */
    wifi_us=(uint64_t)now_ms*1000U;
    wifi_hold(1,1000U);wifi_hold(-2,500U);wifi_hold(0,1000U); /* boot: the first join is never cut short */
    wifi_hold(3,10U);
    publish_done_cb(NULL,ERR_OK);queued=1;mqtt_utk_poll_telemetry(); /* never acknowledged */
    const u32_t wifi_lost_at=now_ms;const unsigned links=connects;
    assert(unacked && unacked_since_ms==wifi_lost_at);
    wifi_hold(1,499U);assert(wifi_poll(1)==WIFI_REJOIN_LEAVE); /* JOIN for 10 s */
    assert(leaves==1 && disconnects==1 && !fake_client.connected && timers==1);
    for(unsigned n=0;n<20;++n)mqtt_utk_start();
    assert(connects==links); /* waits for the 3 s retry */
    wifi_hold(0,49U);assert(wifi_poll(0)==WIFI_REJOIN_JOIN && joins==1); /* 1 s after the leave */
    /* AP still off: NONET belongs to the 5 s retry, never a leave. */
    wifi_hold(-2,250U);
    timers=0;connect_error=-4;retry_timeout(NULL); /* ERR_RTE: no route while down */
    assert(connects==links+1U && timers==1);
    wifi_rejoin_joined(&wifi,wifi_us); /* the 5 s retry joins */
    /* Stuck in JOIN: the next leave is 10 s after that join, not sooner. */
    wifi_hold(1,499U);assert(wifi_poll(1)==WIFI_REJOIN_LEAVE && leaves==2 && timers==1);
    wifi_hold(0,49U);assert(wifi_poll(0)==WIFI_REJOIN_JOIN && joins==2);
    wifi_hold(1,100U);wifi_hold(3,1U); /* AP back */
    timers=0;connect_error=ERR_OK;retry_timeout(NULL);assert(connects==links+2U);
    fake_client.connected=1;connection_cb(client,NULL,MQTT_CONNECT_ACCEPTED);
    assert(events_posted==10 && event_ids[9]==EV_LINK && now_ms-wifi_lost_at==29020U &&
           event_values[9]==ev_link(EV_LINK_RECONNECTED,29U));
    /* A short drop the radio rides out by itself keeps the session. */
    wifi_hold(1,250U);wifi_hold(3,1U);assert(leaves==2 && disconnects==2 && fake_client.connected);
    /* NOIP (DHCP stalled) after UP counts as lost too; UP during the settle
     * cancels the join. */
    wifi_hold(2,499U);assert(wifi_poll(2)==WIFI_REJOIN_LEAVE && leaves==3 && disconnects==3 && timers==1);
    wifi_hold(3,1U);wifi_hold(0,499U);assert(joins==2 && wifi_poll(0)==WIFI_REJOIN_LEAVE);
    puts("INT-CR1-07 actual MQTT port: binary length/NULs, fragments, malformed input, backpressure, autonomous link-loss independence and retry guard PASS");
    puts("MQTT robot events: EV_LINK boot/reconnected, queued through a link drop, one per poll on their own topic, backpressure keeps the record, held after 3 s without a broker ACK, outage dated from the first unacknowledged publish PASS");
    puts("Wi-Fi re-join: DOWN/JOIN/NOIP 10 s after UP leaves and re-joins 1 s later for ever, NONET left to the 5 s retry, session dropped and 3 s retry reconnects with EV_LINK PASS");
    return 0;
}
