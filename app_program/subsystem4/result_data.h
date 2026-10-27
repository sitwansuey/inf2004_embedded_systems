#ifndef S4_RESULT_DATA_H
#define S4_RESULT_DATA_H
#include "../shared/edr.h"
#define S4_RESULT_BITS 93U
#define S4_RESULT_VERSION 1U
#define S4_RESULT_SIZE ((S4_RESULT_BITS+7U)/8U)
#define S4_RESULT_KIND 2U
#define S4_RESULT_TILT 1U
#define S4_RESULT_HEIGHT 2U
#define S4_RESULT_SHOCK 4U
#define S4_RESULT_CALIBRATED 8U
#define S4_RESULT_ODOMETRY 16U
#define S4_RESULT_FLAG_MASK 31U
#define S4_RESULT_FAULT_MASK (EDR_FAULT_ENCODER|EDR_FAULT_IMU)
/* Transient decoded calculation view; persistent/inter-task storage is bytes.
 * All encoder errors leave output untouched, and decode requires exact length. */
typedef struct {
    uint32_t calculated_ms;
    int32_t pitch_cdeg, roll_cdeg;
    uint16_t height_mm, maximum_mm;
    uint8_t terrain, flags, faults; /* faults use shared EDR bit meanings */
} s4_result_t;
bool s4_result_encode(const s4_result_t *value,uint8_t *bytes,size_t capacity);
bool s4_result_decode(const uint8_t *bytes,size_t length,s4_result_t *value);
#endif
