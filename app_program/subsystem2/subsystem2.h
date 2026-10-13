/*
 * subsystem2.h
 * Buddy 2 -- Open-loop motor control. Student ID: 2500613.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MOTOR 14
#define S2_COMMAND_COUNT 8

/* Refresh moving commands before this lease expires (milliseconds). */
#ifndef S2_COMMAND_TIMEOUT_MS
#define S2_COMMAND_TIMEOUT_MS 1000U
#endif

#if (S2_COMMAND_TIMEOUT_MS == 0) || (S2_COMMAND_TIMEOUT_MS > 2147483647U)
#error "S2_COMMAND_TIMEOUT_MS must fit a positive TMO value"
#endif

/* Leave disabled until the encoder is mounted and wired. */
#ifndef S2_ENCODER_ENABLED
#define S2_ENCODER_ENABLED 0
#endif

#if (S2_ENCODER_ENABLED != 0) && (S2_ENCODER_ENABLED != 1)
#error "S2_ENCODER_ENABLED must be 0 or 1"
#endif

typedef enum
{
    S2_MOVE_STOP,
    S2_MOVE_FORWARD,
    S2_MOVE_REVERSE,
    S2_MOVE_LEFT,
    S2_MOVE_RIGHT,
    S2_MOVE_PIVOT_RIGHT, /* Left wheel forward, right wheel stopped. */
    S2_MOVE_PIVOT_LEFT   /* Left wheel stopped, right wheel forward. */
} s2_move_command_t;

typedef struct
{
    T_MSG hdr; /* Kernel mailbox header must be first. */
    s2_move_command_t command;
    UINT power_percent; /* 0-100. */
    UW issued_at_ms;
} s2_movement_msg_t;

typedef enum
{
    S2_MOTION_STOPPED,
    S2_MOTION_MOVING,
    S2_MOTION_FAULT
} s2_motion_state_t;

typedef struct
{
    s2_motion_state_t state;
    s2_move_command_t command;
    UINT power_percent;
    ER last_error; /* E_TMOUT means the movement command expired. */
    UW last_command_ms;
} s2_motion_status_t;

typedef struct
{
    UW positive_pulses;
    UW negative_pulses;
    UW a_edges;
    UW b_edges;
    UINT a_level;
    UINT b_level;
} s2_encoder_status_t;

IMPORT ER subsystem2_init(void);
IMPORT ER subsystem2_start(void);
IMPORT void s2_controller_bind(void);

/* Other subsystems submit expiring requests to the vehicle controller. */
IMPORT ER app_vehicle_request(UINT subsystem, s2_move_command_t command,
                              UINT power, UW lease_ms);

/* Task context only; moving commands need periodic renewal. */
IMPORT ER s2_movement_send(s2_move_command_t command, UINT power_percent);
/* Reports commanded output, not measured wheel motion. Task context only. */
IMPORT ER s2_motion_get_status(s2_motion_status_t *p_status);
/* E_NOSPT and zeroed output when encoder diagnostics are disabled.
 * Diagnostics require APP_S2_ONLY=1 and S2_ENCODER_ENABLED=1.
 * Counts one pulse per rising edge of A; sign depends on wiring.
 */
IMPORT ER s2_encoder_get_status(s2_encoder_status_t *p_status);

#if APP_TEST
IMPORT void subsystem2_test(void);
#endif

#endif /* SUBSYSTEM2_H */
