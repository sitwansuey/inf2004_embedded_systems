/* Copy to robot_config.local.h only after inspecting the robot. Values below
 * intentionally leave motion disabled. Never treat them as calibration. */
#ifndef CSC02_LOCAL_CONFIG_H
#define CSC02_LOCAL_CONFIG_H
#define BOARD_MOTORS_DISABLED 1 /* keep 1 for USB/sensor work, including S2Bench */
#define BOARD_IR_MODE 0 /* 1=verified DO, 2=verified AO on GP26/27/28 */
#define BOARD_IR_BLACK_LEVEL 1 /* digital only: verify actual polarity */
#define BOARD_IR_LINE_MASK 3 /* GP26 right + GP27 left, confirmed physical roles */
#define BOARD_BARCODE_SENSOR 2 /* GP28 is dedicated barcode; S3 array index 2 */
#define BOARD_IR_POSITIONS {1000,-1000,0} /* logical units, not measured spacing */
#define BOARD_MAP_A 0
#define BOARD_MAP_Z 0 /* set only from the actual command codebook */
#define BOARD_IR_ADC_CALIBRATED 0
#define BOARD_IR_ADC_WHITE {0,0,0}
#define BOARD_IR_ADC_BLACK {0,0,0} /* replace with measured endpoints */
#define BOARD_TICKS_PER_REV 0
#define BOARD_WHEEL_CIRCUMFERENCE_MM 0
#define BOARD_TRACK_WIDTH_MM 0
#define BOARD_TERRAIN_CALIBRATED 0 /* must establish X forward, Y left, Z up */
#define BOARD_PITCH_ZERO_CDEG 0
#define BOARD_ROLL_ZERO_CDEG 0
#define BOARD_BYPASS_VALIDATED 0
#endif
