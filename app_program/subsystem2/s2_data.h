#ifndef S2_DATA_H
#define S2_DATA_H
#include "../app.h"
#include "../shared/edr.h"
/* Task context. Write four bytes (two EDR16 records) on E_OK; otherwise leave
 * output untouched. Motion order: IDs 11,12; encoder order: IDs 1,2.
 * Publish each record separately. E_OBJ for uninitialized/invalid encoders.
 * Timestamps/validity/full counts are LOCAL s2_*_get_status() fields only. */
ER s2_motion_encode(uint8_t * const output, const size_t capacity);
ER s2_encoder_encode(uint8_t * const output, const size_t capacity);
#endif
