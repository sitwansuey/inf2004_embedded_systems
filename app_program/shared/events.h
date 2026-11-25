/* Robot events for the dashboard's event log. Each event is ONE EDR16-format
 * record (two bytes, little-endian: low nibble = event ID, high 12 bits =
 * value) published on EVENT_MQTT_TOPIC, separate from the telemetry stream.
 * Events are queued by subsystem 1 and survive a short link drop: the MQTT
 * port stops handing them to lwIP once the broker has acknowledged nothing for
 * 3 s, and sends them once the robot is back on the broker (only those handed
 * over in the last ~3 s before an outage can be lost). A full queue (16)
 * drops new events. See docs/DATA_REPRESENTATION.md. */
#ifndef CSC02_EVENTS_H
#define CSC02_EVENTS_H
#define EVENT_MQTT_TOPIC "Robopico/events/edr16/v1"
enum { EV_BARCODE = 1, EV_INSTRUCTION, EV_HUMP, EV_OBSTACLE, EV_LINK };

/* EV_BARCODE, one per finished read of a marker: bits 0-1 attempt (1 initial
 * scan, 2 reverse re-read, 3 final slow pass), bits 2-6 bars+spaces read
 * (0..31, a full *X* is 29), bits 7-11 letter (0 = failed, 1..26 = A..Z). */
static inline unsigned ev_barcode(unsigned attempt, unsigned elements, unsigned letter)
{
    return (attempt & 3U) | ((elements > 31U ? 31U : elements) << 2U) |
           ((letter > 26U ? 0U : letter) << 7U);
}
/* EV_INSTRUCTION, a decoded marker accepted as a turn instruction (posted
 * after the EV_BARCODE attempt that decoded it): bits 0-4 letter 1..26,
 * bits 5-7 turn (0 none, 1 left, 2 right, 3 straight, 4 U-turn). */
static inline unsigned ev_instruction(unsigned letter, unsigned turn)
{
    return (letter > 26U ? 0U : letter) | ((turn & 7U) << 5U);
}
/* EV_HUMP: bits 0-2 kind, bits 3-11 value. Heights are this hump's, in 2 mm.
 * The controller's estimate (vehicle_hump.h: wheel distance x tilt from
 * level ground) is sent when it has one (kinds 4 and 5); otherwise
 * subsystem 4's terrain height (kinds 2 and 3), 0 when it lost it. */
enum { EV_HUMP_DETECTED = 1, /* climbing: tilt in 0.1 deg */
       EV_HUMP_CREST,        /* going down (slow): S4's height so far */
       EV_HUMP_LEVEL,        /* level again: S4's completed height */
       EV_HUMP_CREST_EST,    /* going down (slow): the controller's height so far */
       EV_HUMP_LEVEL_EST };  /* level again: the controller's highest point */
static inline unsigned ev_hump(unsigned kind, unsigned value)
{
    return (kind & 7U) | ((value > 511U ? 511U : value) << 3U);
}
/* EV_OBSTACLE: bits 0-1 kind, bits 2-3 side/how, bits 4-11 value. */
enum { EV_OBSTACLE_BACK = 0,  /* avoidance over; how 0 ended by a fault or STOP (disarmed),
                                 1 back on the line on the return, 2 by the spot search,
                                 3 obstacle went away; value = seconds since detection */
       EV_OBSTACLE_DETECTED,  /* value = distance cm */
       EV_OBSTACLE_SCANNED,   /* side 1 left / 2 right; value = width cm. Side 0: no way
                                 round, scanning again; value = the retry (1..3) */
       EV_OBSTACLE_BYPASS };  /* side; value = turn-out heading deg */
static inline unsigned ev_obstacle(unsigned kind, unsigned side, unsigned value)
{
    return (kind & 3U) | ((side & 3U) << 2U) | ((value > 255U ? 255U : value) << 4U);
}
/* EV_LINK: bits 0-1 kind, bits 2-11 seconds (saturating at 1023). */
enum { EV_LINK_RECONNECTED = 1, /* back on the MQTT broker after N s without it (from the
                                   first publish the broker never acknowledged) */
       EV_LINK_BOOT };          /* first broker connection, N s after boot */
static inline unsigned ev_link(unsigned kind, unsigned seconds)
{
    return (kind & 3U) | ((seconds > 1023U ? 1023U : seconds) << 2U);
}
/* Subsystem 1 (subsystem1/telemetry.c): nonblocking, callable from any task;
 * an event is dropped (counted) when the small queue is full. */
void s1_event_post(unsigned id, unsigned value);
#endif
