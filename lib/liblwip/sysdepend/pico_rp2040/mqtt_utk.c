/*
 *    mqtt_utk.c
 *    MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *    Runs entirely on the single processor-1-owned task that already owns
 *    the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *    cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *    time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *    timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *    polling that task already does for DHCP and TCP -- nothing extra runs
 *    on any other task or core.
 *
 *    This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *    (unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *    file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/network_bridge.h"
#include <string.h>
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S 60
#define MQTT_UTK_RETRY_MS 3000

/* Optional broker login (config/wifi_secrets.h). NULL, the default, connects
 * anonymously as before. With a broker ACL that lets only the dashboard
 * publish csc02/command, other clients cannot command the robot
 * (app_program/external_systems/Dashboard/README.md, "Broker access"). */
#ifndef MQTT_USERNAME
#define MQTT_USERNAME NULL
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD NULL
#endif

/* Commands arrive in fragments; accept exactly one six-byte command on the
 * command topic. Observations are independently framed by MQTT at two bytes. */
static mqtt_client_t * client;
static ip_addr_t       broker_addr;
static u8_t            connecting;
static u8_t            retry_pending;
static unsigned char   incoming[CSC02_COMMAND_BYTES];
static unsigned        incoming_used;
static unsigned        incoming_total;
static unsigned        incoming_valid;
static unsigned        command_enabled;
/* Broker link history for EV_LINK: connected once since boot; lost since. */
static u8_t  ever_connected, link_lost;
static u32_t link_lost_ms;
/* lwIP completes a QoS 0 publish (publish_done_cb, ERR_OK) when the broker's
 * TCP ACK arrives. After a Wi-Fi loss lwIP closes the connection only on its
 * keep-alive watchdog, about 90 s later, so an outage is dated from the first
 * publish the broker has not acknowledged. Once nothing has been acknowledged
 * for MQTT_UTK_EVENT_HOLD_MS, robot events stay queued in subsystem 1: lwIP
 * would drop them, unsent, when it closes the connection. */
#define MQTT_UTK_EVENT_HOLD_MS 3000U
/* telemetry_edr_record() indexes: ID 12 is index 11, status subtype 9 is 15+9.
 */
#define MQTT_UTK_ID12_INDEX 11U
#define MQTT_UTK_QUALITY_INDEX 24U
static u8_t  unacked;
static u32_t unacked_since_ms;
/* EV_LINK waits here, not in subsystem 1's event queue: after an outage that
 * queue can be full of the VC's events, and a full queue drops the newest.
 * It is published before them. A later connection replaces an unsent one. */
static unsigned char link_event[EDR_RECORD_SIZE];
static u8_t          link_pending;
static void
link_event_set (unsigned value)
{
    unsigned word = (value << 4U) | EV_LINK;
    link_event[0] = (unsigned char) (word & 255U);
    link_event[1] = (unsigned char) (word >> 8U);
    link_pending  = 1;
}

static void publish_done_cb (void * arg, err_t result);
static void retry_timeout (void * arg);
static void
published (void)
{
    if (!unacked)
    {
        unacked          = 1;
        unacked_since_ms = sys_now();
    }
}


/* All callbacks and publishing remain on cyw43_utk_task. Binary-safe and
 * length-delimited: a NUL is a payload byte, never a terminator. */
static void
incoming_publish_cb (void * arg, const char * topic, u32_t length)
{
    (void) arg;
    incoming_used  = 0;
    incoming_total = length;
    incoming_valid = strcmp(topic, CSC02_COMMAND_TOPIC) == 0 &&
                     length == CSC02_COMMAND_BYTES;
}
static void
incoming_data_cb (void * arg, const u8_t * data, u16_t length, u8_t flags)
{
    (void) arg;
    if (!incoming_valid)
    {
        return;
    }
    if (incoming_used + length > sizeof(incoming))
    {
        incoming_valid = 0;
        return;
    }
    memcpy(incoming + incoming_used, data, length);
    incoming_used += length;
    if (flags & MQTT_DATA_FLAG_LAST)
    {
        if (incoming_used == incoming_total)
        {
            /* Reconnect requires a fresh RESET before ARM. A retained ARM
             * alone must never restart a vehicle after boot or link loss. */
            if (incoming[1] == 0)
            {
                command_enabled = 0;
            }
            /* These replies carry the VC's last published phase and fault. */
            if ((incoming[1] == 1 || incoming[1] == 3) && !command_enabled)
            {
                s1_command_reply(incoming[5], incoming[1], 2);
            }
            else
            {
                int offered = s1_command_offer(incoming, incoming_used);
                if (offered == 0 && incoming[1] == 2)
                {
                    command_enabled = 1;
                }
                if (offered != 0)
                {
                    s1_command_reply(incoming[5], incoming[1], 1);
                }
            }
        }
        incoming_valid = 0;
    }
}

static void
subscribe_done_cb (void * arg, err_t result)
{
    (void) arg;
    if (result == ERR_OK)
    {
        tm_printf((UB *) "mqtt: subscribed to " CSC02_COMMAND_TOPIC "\n");
    }
    else
    {
        tm_printf((UB *) "mqtt: subscribe failed, err=%d\n", (int) result);
    }
}

static void
publish_done_cb (void * arg, err_t result)
{
    (void) arg;
    if (result == ERR_OK)
    {
        unacked = 0; /* the broker acknowledged */
    }
    else
    {
        tm_printf((UB *) "mqtt: publish failed, err=%d\n", (int) result);
    }
}


void
mqtt_utk_poll_telemetry (void)
{
    static unsigned char snapshot[TELEMETRY_BYTES];
    static int64_t       values[TEL_FIELD_COUNT];
    static unsigned      next = TELEMETRY_EDR_RECORDS;
    static unsigned      resume;
    if (!client || !mqtt_client_is_connected(client))
    {
        return;
    }
    /* Command replies take priority over the sensor stream. */
    static unsigned char ack[CSC02_ACK_BYTES];
    static unsigned      ack_pending;
    if (!ack_pending && s1_command_ack_take(ack) == CSC02_ACK_BYTES)
    {
        ack_pending = 1;
    }
    if (ack_pending &&
        mqtt_publish(client, CSC02_ACK_TOPIC, ack, CSC02_ACK_BYTES, 0, 0,
                     publish_done_cb, NULL) == ERR_OK)
    {
        ack_pending = 0;
        published();
    }
    /* Then one robot event per poll, EV_LINK first, then the queued ones
     * (events survive a link drop): none while the broker has not
     * acknowledged anything for MQTT_UTK_EVENT_HOLD_MS. */
    static unsigned char event[EDR_RECORD_SIZE];
    static unsigned      event_pending;
    int                  hold =
        unacked && sys_now() - unacked_since_ms >= MQTT_UTK_EVENT_HOLD_MS;
    if (!hold && link_pending)
    {
        if (mqtt_publish(client, CSC02_EVENT_TOPIC, link_event, EDR_RECORD_SIZE,
                         0, 0, publish_done_cb, NULL) == ERR_OK)
        {
            link_pending = 0;
            published();
        }
    }
    else
    {
        if (!hold && !event_pending && s1_event_take(event) == EDR_RECORD_SIZE)
        {
            event_pending = 1;
        }
        if (!hold && event_pending &&
            mqtt_publish(client, CSC02_EVENT_TOPIC, event, EDR_RECORD_SIZE, 0,
                         0, publish_done_cb, NULL) == ERR_OK)
        {
            event_pending = 0;
            published();
        }
    }

    /* One-snapshot flags of an abandoned snapshot that were not sent yet:
     * ID 12 (IMU fault) and status subtype 9 (impact) go on in the next. */
    static u8_t carry_imu, carry_impact;
    if (next < TELEMETRY_EDR_RECORDS && !s1_telemetry_current(snapshot))
    {
        /* Abandoned part-way: a dropped snapshot, counted. Continue at the
         * unsent field in the fresh snapshot. Width/status fields must not
         * starve behind encoder/IR records on slow links. */
        s1_telemetry_dropped();
        if (values[TEL_IMU_FAULT] &&
            (MQTT_UTK_ID12_INDEX + TELEMETRY_EDR_RECORDS - resume) %
                    TELEMETRY_EDR_RECORDS >=
                next)
        {
            carry_imu = 1;
        }
        if ((values[TEL_QUALITY] & 2) &&
            (MQTT_UTK_QUALITY_INDEX + TELEMETRY_EDR_RECORDS - resume) %
                    TELEMETRY_EDR_RECORDS >=
                next)
        {
            carry_impact = 1;
        }
        resume = (resume + next) % TELEMETRY_EDR_RECORDS;
        next   = TELEMETRY_EDR_RECORDS;
    }
    if (next == TELEMETRY_EDR_RECORDS)
    {
        if (s1_telemetry_take(snapshot) != TELEMETRY_BYTES ||
            !telemetry_decode(snapshot, values))
        {
            return;
        }
        if (carry_imu)
        {
            values[TEL_IMU_FAULT] = 1;
        }
        if (carry_impact)
        {
            values[TEL_QUALITY] |= 2;
        }
        carry_imu = carry_impact = 0;
        next                     = 0;
    }
    /* Four two-byte messages per poll; retain the exact record on backpressure.
     * Finish a snapshot before taking another, so later fields cannot starve.
     */
    for (unsigned budget = 0; budget < 4 && next < TELEMETRY_EDR_RECORDS;)
    {
        uint8_t record[EDR_RECORD_SIZE];
        if (!telemetry_edr_record(
                values, (resume + next) % TELEMETRY_EDR_RECORDS, record))
        {
            ++next;
            continue;
        }
        if (mqtt_publish(client, CSC02_TELEMETRY_TOPIC, record, EDR_RECORD_SIZE,
                         0, 0, publish_done_cb, NULL) != ERR_OK)
        {
            return;
        }
        published();
        ++next;
        ++budget;
    }
}

static void
retry_timeout (void * arg)
{
    (void) arg;
    connecting    = 0;
    retry_pending = 0;
    mqtt_utk_start();
}

static void
connection_cb (mqtt_client_t * c, void * arg, mqtt_connection_status_t status)
{
    (void) c;
    (void) arg;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        command_enabled = 0;
        err_t err;

        /* No longer *connecting* -- clear the guard so that if this
         * connection later drops, the poll loop's mqtt_utk_start() call
         * is allowed to reconnect. Leaving it set would wedge the client
         * permanently after the first disconnect. */
        connecting    = 0;
        retry_pending = 0;
        sys_untimeout(retry_timeout, NULL);

        tm_printf((UB *) "mqtt: connected to " MQTT_BROKER_IP "\n");
        /* Event log: back after an outage, or the first connection. */
        if (link_lost)
        {
            link_event_set(
                ev_link(EV_LINK_RECONNECTED,
                        (unsigned) ((sys_now() - link_lost_ms) / 1000U)));
            link_lost = 0;
        }
        else if (!ever_connected)
        {
            link_event_set(
                ev_link(EV_LINK_BOOT, (unsigned) (sys_now() / 1000U)));
        }
        ever_connected = 1;
        unacked        = 0;

        /* Commands use a separate inbound topic; observations use EDR16 only.
         */
        mqtt_set_inpub_callback(client, incoming_publish_cb, incoming_data_cb,
                                NULL);

        err = mqtt_subscribe(client, CSC02_COMMAND_TOPIC, 0 /* qos */,
                             subscribe_done_cb, NULL);
        if (err != ERR_OK)
        {
            tm_printf((UB *) "mqtt: mqtt_subscribe() failed, err=%d\n",
                      (int) err);
        }

        /* Drop any chain left over from a previous connection before
         * arming a new one. Each reconnect would otherwise add another
         * self-perpetuating heartbeat, and every one of them holds a
         * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
        /* The application snapshot is the heartbeat; no extra text publication.
         */
    }
    else
    {
        command_enabled = 0;
        /* Lost since the first publish the broker never acknowledged. */
        if (ever_connected && !link_lost)
        {
            link_lost    = 1;
            link_lost_ms = unacked ? unacked_since_ms : sys_now();
        }
        unacked = 0; /* lwIP has dropped those publishes with the connection */
        /* Timed ARM expires locally. Autonomous START continues onboard. */
        /* mqtt_client_connect() reconnects on its own for a dropped
         * keepalive; a rejected/never-established connection does not
         * retry itself, so do that here -- on a delay, so a broker
         * that keeps rejecting (e.g. bad client ID) can't turn this
         * into a tight reconnect loop. */
        tm_printf((UB *) "mqtt: connection status %d, retrying in %ds\n",
                  (int) status, MQTT_UTK_RETRY_MS / 1000);
        connecting    = 0;
        retry_pending = 1;
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

/* cyw43_utk.c had no Wi-Fi link for 10 s and is re-joining. lwIP would keep
 * this session until its keep-alive watchdog (~90 s), and a session that
 * outlives the outage never reports it. Close it now: the 3 s retry then
 * reconnects once the link is back and posts EV_LINK with the outage. */
void
mqtt_utk_link_lost (void)
{
    if (client == NULL)
    {
        return;
    }
    mqtt_disconnect(client); /* a local close calls no connection_cb */
    connection_cb(client, NULL, MQTT_CONNECT_DISCONNECTED);
}

void
mqtt_utk_start (void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             err;

    if (retry_pending || connecting ||
        (client != NULL && mqtt_client_is_connected(client)))
    {
        return;
    }

    if (client == NULL)
    {
        client = mqtt_client_new();
        if (client == NULL)
        {
            tm_printf((UB *) "mqtt: mqtt_client_new() failed\n");
            return;
        }
    }

    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr)))
    {
        tm_printf((UB *) "mqtt: bad MQTT_BROKER_IP\n");
        return;
    }
    IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

    client_info.client_id   = MQTT_CLIENT_ID;
    client_info.keep_alive  = MQTT_UTK_KEEPALIVE_S;
    client_info.client_user = MQTT_USERNAME;
    client_info.client_pass = MQTT_PASSWORD;

    connecting = 1;
    err        = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
                                     connection_cb, NULL, &client_info);
    if (err != ERR_OK)
    {
        tm_printf((UB *) "mqtt: mqtt_client_connect() failed, err=%d\n",
                  (int) err);
        connecting    = 0;
        retry_pending = 1;
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

/*** end of file ***/
