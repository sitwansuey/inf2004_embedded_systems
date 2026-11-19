/* Optional per-checkout overrides. Copy to robot_config.local.h (git-ignored):
 * robot_config.h includes it first, so every value here wins. The defaults in
 * robot_config.h are already this robot's measured mission profile; define
 * only what this checkout should change. Host tests ignore the local file
 * (they define ROBOT_CONFIG_NO_LOCAL). */
#ifndef ROBOT_CONFIG_LOCAL_H
#define ROBOT_CONFIG_LOCAL_H
#define BOARD_MOTORS_DISABLED 1 /* USB/sensor bench: drive GPIOs low, ARM/START rejected */
/* Ramp bench test: stop the run (VC_OK) once level again after a ramp. */
/* #define BOARD_RAMP_STOP_AFTER 1 */
/* Edge tracing instead of the mask-pivot follower: */
/* #define BOARD_LINE_MASK_PIVOT 0 */
/* #define BOARD_LINE_EDGE_TRACE 1 */
/* Line lost while tracking: search after this much travel with both IRs white (0: off). */
/* #define BOARD_LINE_LOST_MM 600U */
#endif
