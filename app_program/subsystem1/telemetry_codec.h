#ifndef CSC02_TELEMETRY_CODEC_H
#define CSC02_TELEMETRY_CODEC_H
#include <stdint.h>
#include <stdbool.h>
#define TELEMETRY_BITS 433U
#define TELEMETRY_BYTES                                                        \
    55U /* 433 data bits; the last 7 bits are zero padding */
enum
{
    TEL_VERSION,
    TEL_SEQUENCE,
    TEL_TIME_MS,
    TEL_PHASE,
    TEL_FAULT,
    TEL_COMMAND,
    TEL_POWER,
    TEL_MOTOR_STATE,
    TEL_AVAILABLE,
    TEL_LEFT_COUNTS,
    TEL_RIGHT_COUNTS,
    TEL_SPEED_MM_S,
    TEL_LINE_STATE,
    TEL_IR0,
    TEL_IR1,
    TEL_IR2,
    TEL_BLACK_MASK,
    TEL_LINE_ERROR,
    TEL_BARCODE,
    TEL_NAVIGATION,
    TEL_AX_MG,
    TEL_AY_MG,
    TEL_AZ_MG,
    TEL_MX_MG,
    TEL_MY_MG,
    TEL_MZ_MG,
    TEL_PITCH_CDEG,
    TEL_ROLL_CDEG,
    TEL_TERRAIN,
    TEL_HEIGHT_MM,
    TEL_HIGHEST_MM,
    TEL_DISTANCE_MM,
    TEL_WIDTH_MM,
    TEL_SERVO_DEG,
    TEL_SCAN_STATE,
    TEL_QUALITY,
    TEL_DROPPED, /* internal drops modulo 65536; status subtype 12 sends it
                    modulo 256 */
    TEL_MOTOR_ERROR,
    TEL_BARCODE_ELEMENTS, /* bars+spaces read in the current/last barcode (a
                             full *X* is 29), saturates at 31 */
    TEL_BARCODE_RESULT,   /* 0 none, 1 reading, 2 decoded, 3 failed */
    TEL_WIDTH_LEFT_MM,  /* centre -> left edge half-width, 0 unless width valid
                         */
    TEL_WIDTH_RIGHT_MM, /* centre -> right edge half-width */
    TEL_BYPASS_SIDE,    /* 0 none, 1 left, 2 right */
    TEL_FAULT_CAUSE, /* VC_SENSOR only: 0 other, 1 sonar, 2 tilt, 3 odometry */
    TEL_IMU_FAULT, /* 1: an S4 result since the last snapshot had EDR_FAULT_IMU
                      (ID 12 bit 0x04) */
    TEL_FIELD_COUNT
};
bool telemetry_encode (uint8_t       out[TELEMETRY_BYTES],
                       const int64_t values[TEL_FIELD_COUNT]);
bool telemetry_decode (const uint8_t in[TELEMETRY_BYTES],
                       int64_t       values[TEL_FIELD_COUNT]);
/* One-snapshot flags: impact (TEL_QUALITY bit 1) and TEL_IMU_FAULT. A snapshot
 * dropped for a newer one ORs them into the newer one, so they are still sent.
 */
void telemetry_keep_flags (uint8_t       newer[TELEMETRY_BYTES],
                           const uint8_t older[TELEMETRY_BYTES]);
#endif

/*** end of file ***/
