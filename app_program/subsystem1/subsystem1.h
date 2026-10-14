/*
 *	subsystem1.h
 *	Buddy 1 -- MQTT telemetry and retained LED example. Student ID: 2500238.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telemetry_topics.h"

/* Task context only. Nonblocking copied messages; E_TMOUT means queue full.
 * payload is an MQTT scalar (or event JSON for S1_VEHICLE_CONTROLLER_EVENTS).
 * Caller owns retry/drop policy. No MQTT/lwIP calls from application tasks.
 */
IMPORT ER s1_telemetry_send(S1_READING_ID reading, const char *payload);
IMPORT ER s1_telemetry_integer(S1_READING_ID reading, INT value);
IMPORT ER s1_telemetry_boolean(S1_READING_ID reading, BOOL value);
IMPORT ER s1_telemetry_text(S1_READING_ID reading, const char *value);
IMPORT ER s1_telemetry_init(void);

#define S1_PRI_LED	12		/* priority band 12-13 (TEAM.md) */
#define S1_LED_PIN	19		/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250		/* LED toggles every tick */

/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID	tid_led;

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
