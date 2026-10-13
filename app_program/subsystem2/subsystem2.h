/*
 *	subsystem2.h
 *	Subsystem 2 -- Motor command consumer.  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MOTOR 14 /* priority band 14-15 (TEAM.md) */
#define S2_COMMAND_COUNT 8

typedef enum
{
	S2_MOVE_STOP,
	S2_MOVE_FORWARD,
	S2_MOVE_REVERSE,
	S2_MOVE_LEFT,
	S2_MOVE_RIGHT,
	S2_MOVE_PIVOT_RIGHT, /* left wheel forward, right wheel stopped */
	S2_MOVE_PIVOT_LEFT   /* left wheel stopped, right wheel forward */
} s2_move_command_t;

typedef struct
{
	T_MSG hdr;				   /* used by the kernel, do not touch */
	s2_move_command_t command; /*movement command*/
	UINT power_percent;		   /* 0–100 */
} s2_movement_msg_t;

IMPORT void s2_controller_bind(void);
/* Other subsystems submit expiring requests to the controller. */
IMPORT ER app_vehicle_request(UINT subsystem, s2_move_command_t command,
                              UINT power, UW lease_ms);

IMPORT ER s2_movement_send(
	s2_move_command_t command,
	UINT power_percent);
typedef struct {
    UW positive_pulses, negative_pulses;
    UW a_edges, b_edges;
    UINT a_level, b_level;
} s2_encoder_status_t;
/* Counts one pulse per rising edge of channel A; sign depends on wiring. */
IMPORT ER s2_encoder_get_status(s2_encoder_status_t *status);
#endif /* SUBSYSTEM2_H */
