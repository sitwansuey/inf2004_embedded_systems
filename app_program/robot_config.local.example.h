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
#define BOARD_S4_MOUNTING 0 /* 1 for this car's forward=-Y,left=X,up=Z mounting */
#define BOARD_TERRAIN_CALIBRATED 0 /* verify selected mounting and flat zero first */
#define BOARD_PITCH_ZERO_CDEG 0
#define BOARD_ROLL_ZERO_CDEG 0
#define BOARD_ACCEL_CALIBRATED 0 /* multi-orientation bias/gain measurements pending */
#define BOARD_ACCEL_BIAS_X_MG 0
#define BOARD_ACCEL_BIAS_Y_MG 0
#define BOARD_ACCEL_BIAS_Z_MG 0
#define BOARD_ACCEL_GAIN_X_MILLI 1000
#define BOARD_ACCEL_GAIN_Y_MILLI 1000
#define BOARD_ACCEL_GAIN_Z_MILLI 1000
#define BOARD_TERRAIN_ONE_SIDED 1 /* reports IMU rise while one drive wheel stays on floor */
#define BOARD_IMU_GEOMETRY_CALIBRATED 0
#define BOARD_IMU_FORWARD_MM 0 /* sensor centre relative to drive-axle centre */
#define BOARD_IMU_LEFT_MM 0
#define BOARD_IMU_UP_MM 0 /* above wheel axle, not above floor */
#define BOARD_BYPASS_VALIDATED 0
#endif
