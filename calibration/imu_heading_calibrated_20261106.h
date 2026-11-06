/* Measured on the assembled robot: 3D magnetic sweep and upright quarter turns.
 * 223 samples; max manual heading error 7.21 deg; return-to-zero 3.59 deg.
 * Guarded magnetometer heading assistance only. Terrain/height calibration
 * and accelerometer scale/bias calibration are separate and unchanged.
 */
#ifndef IMU_HEADING_CALIBRATED_20261106_H
#define IMU_HEADING_CALIBRATED_20261106_H
#define BOARD_HEADING_CALIBRATED 1
#define BOARD_MAG_OFFSET_MG {365,-100,-155}
#define BOARD_MAG_SCALE_MILLI {959,953,1102}
#endif
