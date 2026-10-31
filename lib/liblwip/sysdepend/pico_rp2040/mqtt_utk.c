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
<<<<<<< HEAD
#include "../../../../app_program/subsystem1/telemetry_topics.h"
=======
#include "../../../../app_program/subsystem1/network_bridge.h"
#include <string.h>
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Keep existing local secrets files compatible. */
#ifndef MQTT_USERNAME
#define MQTT_USERNAME NULL
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD NULL
#endif

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * heartbeat this demo publishes; anything longer is truncated and flagged
 * rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u8_t retry_pending;
static unsigned char incoming[CSC02_COMMAND_BYTES];
static unsigned incoming_used;
static unsigned incoming_total;
static unsigned incoming_valid;
static unsigned command_enabled;

static void publish_done_cb(void *arg,err_t result);
static void retry_timeout(void *arg);

<<<<<<< HEAD
static void heartbeat_timeout(void *arg);
static void publish_done_cb(void *arg, err_t result);

void mqtt_utk_poll_telemetry(void)
{
    static char topic[S1_TOPIC_MAX];
    static unsigned char payload[S1_PAYLOAD_MAX];
    static unsigned int length;
    static int pending;
    unsigned int budget;
    err_t error;
    if (client == NULL || !mqtt_client_is_connected(client)) return;
    /* Bound each visit so network maintenance cannot be starved by producers.
     * Keep a message staged on lwIP backpressure rather than losing it.
     */
    for (budget = 0; budget < 4; ++budget) {
        if (!pending) {
            int result = s1_telemetry_take_bytes(topic, payload, &length);
            if (result <= 0) return;
            pending = 1;
        }
        error = mqtt_publish(client, topic, payload, (u16_t)length,
                             0 /* QoS 0 */, 0 /* not retained */,
                             publish_done_cb, NULL);
        if (error != ERR_OK) return;
        pending = 0;
    }
}
=======
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e

/* All callbacks and publishing remain on cyw43_utk_task. Binary-safe and
 * length-delimited: a NUL is a payload byte, never a terminator. */
static void incoming_publish_cb(void *arg,const char *topic,u32_t length) {
    (void)arg;incoming_used=0;incoming_total=length;
    incoming_valid=strcmp(topic,CSC02_COMMAND_TOPIC)==0 && length==CSC02_COMMAND_BYTES;
}
static void incoming_data_cb(void *arg,const u8_t *data,u16_t length,u8_t flags) {
    (void)arg;
    if(!incoming_valid)return;
    if(incoming_used+length>sizeof(incoming)) {incoming_valid=0;return;}
    memcpy(incoming+incoming_used,data,length);incoming_used+=length;
    if(flags & MQTT_DATA_FLAG_LAST) {
        if(incoming_used==incoming_total) {
            /* Reconnect requires a fresh RESET before ARM. A retained ARM
             * alone must never restart a vehicle after boot or link loss. */
            if(incoming[1]==0)command_enabled=0;
            if(incoming[1]==2 && s1_command_offer(incoming,incoming_used)==0)command_enabled=1;
            else if(incoming[1]!=1 || command_enabled)(void)s1_command_offer(incoming,incoming_used);
        }
        incoming_valid=0;
    }
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " CSC02_COMMAND_TOPIC "\n");
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


void mqtt_utk_poll_telemetry(void) {
    unsigned char payload[TELEMETRY_BYTES];
    if(!client || !mqtt_client_is_connected(client))return;
    /* Latest snapshot wins across backpressure/reconnection; never replay
     * a stale dashboard backlog as if it were current sensor evidence. */
    static unsigned char pending_payload[TELEMETRY_BYTES];
    static unsigned pending;
    for(unsigned n=0;n<2;++n) {
        int length=s1_telemetry_take(payload);
        if(length<0)break;
        if(length==TELEMETRY_BYTES) {memcpy(pending_payload,payload,TELEMETRY_BYTES);pending=1;}
    }
    if(pending && !s1_telemetry_current(pending_payload))pending=0;
    if(pending && mqtt_publish(client,CSC02_TELEMETRY_TOPIC,pending_payload,
            TELEMETRY_BYTES,0,0,publish_done_cb,NULL)==ERR_OK)pending=0;
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

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, CSC02_COMMAND_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		/* The application snapshot is the heartbeat; no extra text publication. */
	} else {
		command_enabled=0;
		const unsigned char stop[CSC02_COMMAND_BYTES]={0xC1,0,0,0,0,0};
		(void)s1_command_offer(stop,sizeof(stop));
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
<<<<<<< HEAD
	client_info.client_user = MQTT_USERNAME;
	client_info.client_pass = MQTT_PASSWORD;
=======
#ifdef MQTT_USERNAME
    client_info.client_user=MQTT_USERNAME;
#endif
#ifdef MQTT_PASSWORD
    client_info.client_pass=MQTT_PASSWORD;
#endif
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e

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
