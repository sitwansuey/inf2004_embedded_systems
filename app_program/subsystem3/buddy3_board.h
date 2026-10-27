/* Buddy 3-only board defaults. No actuator output or peripheral takeover.
 * A coordinator's existing robot_config.h, when present, remains authoritative. */
#ifndef BUDDY3_BOARD_H
#define BUDDY3_BOARD_H
#if defined(__has_include)
#if __has_include("../robot_config.h")
#include "../robot_config.h"
#endif
#endif
#ifndef BOARD_MOTORS_DISABLED
#define BOARD_MOTORS_DISABLED 0 /* No shared owner has declared a drive lock. */
#endif
#ifndef BOARD_IR_MODE
#define BOARD_IR_MODE 0
#endif
#ifndef BOARD_IR_RAW_BENCH
#define BOARD_IR_RAW_BENCH 0
#endif
#ifndef BOARD_IR_CALIBRATED_BENCH
#define BOARD_IR_CALIBRATED_BENCH 0
#endif
#ifndef BOARD_IR_BARCODE_CAPTURE
#define BOARD_IR_BARCODE_CAPTURE 0
#endif
#ifndef BOARD_IR_MOTOR_SCAN_BENCH
#define BOARD_IR_MOTOR_SCAN_BENCH 0
#endif
#if (BOARD_IR_CALIBRATED_BENCH || BOARD_IR_MOTOR_SCAN_BENCH) && !defined(BOARD_IR_ADC_CALIBRATED)
#include "buddy3_calibration.h"
#endif
#ifndef BOARD_IR_LINE_MASK
#define BOARD_IR_LINE_MASK 3
#endif
#ifndef BOARD_BARCODE_SENSOR
#define BOARD_BARCODE_SENSOR 2
#endif
#ifndef BOARD_IR_POSITIONS
#define BOARD_IR_POSITIONS {1000,-1000,0}
#endif
#ifndef BOARD_MAP_A
#define BOARD_MAP_A 0
#endif
#ifndef BOARD_MAP_Z
#define BOARD_MAP_Z 0
#endif
#endif
