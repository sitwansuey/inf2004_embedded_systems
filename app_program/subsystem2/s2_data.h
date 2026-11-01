#ifndef S2_DATA_H
#define S2_DATA_H
#include "../app.h"
#include "../shared/edr.h"
#include "../shared/encoder_snapshot.h"
/* Keep the private snapshot and direct EDR export on the same error mapping. */
static inline unsigned s2_motor_error_code(ER error) {
    return error == E_TMOUT ? EDR_MOTOR_TIMEOUT : error < E_OK ? EDR_MOTOR_ERROR : EDR_MOTOR_OK;
}
/* Full precision, timestamped handoff for odometry/terrain, never MQTT. */
ER s2_encoder_snapshot(uint8_t *output, size_t capacity);
/* Task context. Write four bytes (two EDR16 records) on E_OK; otherwise leave
 * output untouched. Motion order: IDs 11,12; encoder order: IDs 1,2.
 * Publish each record separately. E_OBJ for uninitialized/invalid encoders.
 * Timestamps/validity/full counts are LOCAL s2_*_get_status() fields only. */
ER s2_motion_encode(uint8_t * const output, const size_t capacity);
ER s2_encoder_encode(uint8_t * const output, const size_t capacity);
#endif
