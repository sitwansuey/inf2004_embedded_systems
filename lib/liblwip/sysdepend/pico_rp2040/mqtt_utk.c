/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/network_bridge.h"
#include <string.h>
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Keep existing local secrets files compatible. */
#ifndef MQTT_USERNAME
#define MQTT_USERNAME NULL
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD NULL
#endif

/* Commands arrive in fragments; accept exactly one six-byte command on the
 * command topic. Observations are independently framed by MQTT at two bytes. */
static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u8_t retry_pending;
static unsigned char incoming[BARCODE_MAP_JSON_MAX];
static unsigned incoming_used;
static unsigned incoming_total;
static unsigned incoming_valid;
static unsigned command_enabled;
static unsigned telemetry_cursor,telemetry_loaded;
static unsigned char diagnostic_wire[BARCODE_DIAGNOSTIC_BYTES];
static unsigned diagnostic_pending,diagnostic_inflight,diagnostic_turn;

static void publish_done_cb(void *arg,err_t result);
static void retry_timeout(void *arg);


/* All callbacks and publishing remain on cyw43_utk_task. Binary-safe and
 * length-delimited: a NUL is a payload byte, never a terminator. */
static void incoming_publish_cb(void *arg,const char *topic,u32_t length) {
    (void)arg;incoming_used=0;incoming_total=length;
    incoming_valid=0;
    if(strcmp(topic,CSC02_COMMAND_TOPIC)==0 && length==CSC02_COMMAND_BYTES)incoming_valid=1;
    else if(strcmp(topic,CSC02_BARCODE_MAP_TOPIC)==0 && length>0 && length<=sizeof(incoming))incoming_valid=2;
}
static void incoming_data_cb(void *arg,const u8_t *data,u16_t length,u8_t flags) {
    (void)arg;
    if(!incoming_valid)return;
    if(length>incoming_total-incoming_used || incoming_used+length>sizeof(incoming)) {incoming_valid=0;return;}
    memcpy(incoming+incoming_used,data,length);incoming_used+=length;
    if(flags & MQTT_DATA_FLAG_LAST) {
        if(incoming_used==incoming_total && incoming_valid==2) {
            uint8_t letters[BARCODE_MAP_BYTES]={0};
            if(!barcode_map_parse(incoming,incoming_used,letters))
                s1_barcode_map_ack(BARCODE_MAP_INVALID,NULL);
            else if(s1_barcode_map_offer(letters)!=0)
                s1_barcode_map_ack(BARCODE_MAP_QUEUE_FULL,letters);
        } else if(incoming_used==incoming_total && incoming_valid==1) {
            /* Reconnect requires a fresh RESET before ARM. A retained ARM
             * alone must never restart a vehicle after boot or link loss. */
            if(incoming[1]==0)command_enabled=0;
            if((incoming[1]==1 || incoming[1]==3) && !command_enabled) {
                s1_command_ack(incoming[5],incoming[1],2,0,0);
            } else {
                int offered=s1_command_offer(incoming,incoming_used);
                if(offered==0 && incoming[1]==2)command_enabled=1;
                if(offered!=0)s1_command_ack(incoming[5],incoming[1],1,0,0);
            }
        }
        incoming_valid=0;
    }
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: command/map subscription acknowledged\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}


static void diagnostic_done(void *arg,err_t result) {
    (void)arg;diagnostic_inflight=0;
    if(result==ERR_OK)diagnostic_pending=0; /* Preserve exact frame on failure. */
}
void mqtt_utk_poll_telemetry(void) {
    static unsigned char snapshot[TELEMETRY_BYTES];
    static int64_t values[TEL_FIELD_COUNT];
    /* Every field gets a turn even if backpressure outlives a snapshot.
     * Critical state is first in each cycle; refreshing never rewinds it. */
    static const uint8_t order[TELEMETRY_EDR_RECORDS]={15,16,25,24,10,11,0,1,6,4,5,17,29,28,26,27,
        7,8,9,12,13,14,2,3,18,19,20,21,22,23,30};
    if(!client || !mqtt_client_is_connected(client))return;
    /* Command replies take priority over the sensor stream. */
    static unsigned char ack[CSC02_ACK_BYTES];
    static unsigned ack_pending;
    if(!ack_pending && s1_command_ack_take(ack)==CSC02_ACK_BYTES)ack_pending=1;
    if(ack_pending) {
        if(mqtt_publish(client,ack[0]==0xC4?CSC02_BARCODE_MAP_ACK_TOPIC:CSC02_ACK_TOPIC,ack,CSC02_ACK_BYTES,
                       0,0,publish_done_cb,NULL)!=ERR_OK)return;
        ack_pending=0;
    }
    if(!diagnostic_pending && s1_diagnostic_take(diagnostic_wire)==BARCODE_DIAGNOSTIC_BYTES)diagnostic_pending=1;
    /* Alternate diagnostic and EDR opportunities under a full 256-byte ring.
     * At most one QoS1 diagnostic awaits PUBACK; motors never wait for it. */
    if(diagnostic_pending && !diagnostic_inflight && diagnostic_turn) {
        diagnostic_turn=0;
        if(mqtt_publish(client,CSC02_BARCODE_DIAGNOSTIC_TOPIC,diagnostic_wire,BARCODE_DIAGNOSTIC_BYTES,
                        1,0,diagnostic_done,NULL)==ERR_OK) {
            diagnostic_inflight=1;return;
        }
    } else diagnostic_turn=1;

    if(!telemetry_loaded || !s1_telemetry_current(snapshot)) {
        if(s1_telemetry_take(snapshot)!=TELEMETRY_BYTES || !telemetry_decode(snapshot,values))return;
        telemetry_loaded=1;
    }
    /* At most four EDR records per poll. A blocked field retains its turn;
     * a refreshed snapshot supplies current values at that same cursor. */
    for(unsigned budget=0,visited=0;budget<4 && visited<TELEMETRY_EDR_RECORDS;++visited) {
        uint8_t record[EDR_RECORD_SIZE];
        unsigned index=order[telemetry_cursor];
        if(!telemetry_edr_record(values,index,record)) {
            telemetry_cursor=(telemetry_cursor+1U)%TELEMETRY_EDR_RECORDS;continue;
        }
        if(mqtt_publish(client,CSC02_TELEMETRY_TOPIC,record,EDR_RECORD_SIZE,
                        0,0,publish_done_cb,NULL)!=ERR_OK)return;
        telemetry_cursor=(telemetry_cursor+1U)%TELEMETRY_EDR_RECORDS;++budget;
        if(!telemetry_cursor){telemetry_loaded=0;break;}
    }
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	retry_pending = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;
    incoming_valid=0;incoming_used=0;incoming_total=0;
    telemetry_loaded=0;telemetry_cursor=0;diagnostic_inflight=0;diagnostic_turn=0;

	if(status == MQTT_CONNECT_ACCEPTED) {
		command_enabled=0;
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		retry_pending = 0;
		sys_untimeout(retry_timeout, NULL);

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Commands use a separate inbound topic; observations use EDR16 only. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, CSC02_COMMAND_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

        err=mqtt_subscribe(client,CSC02_BARCODE_MAP_TOPIC,1,subscribe_done_cb,NULL);
        if(err!=ERR_OK)tm_printf((UB*)"mqtt: barcode map subscribe failed, err=%d\n",(int)err);

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		/* The application snapshot is the heartbeat; no extra text publication. */
	} else {
		command_enabled=0;
		/* Timed ARM expires locally. Autonomous START continues onboard. */
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		connecting = 0; retry_pending = 1;
		sys_untimeout(retry_timeout, NULL);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(retry_pending || connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;
#ifdef MQTT_USERNAME
    client_info.client_user=MQTT_USERNAME;
#endif
#ifdef MQTT_PASSWORD
    client_info.client_pass=MQTT_PASSWORD;
#endif

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;retry_pending = 1;
		sys_untimeout(retry_timeout,NULL);
		sys_timeout(MQTT_UTK_RETRY_MS,retry_timeout,NULL);
	}
}
