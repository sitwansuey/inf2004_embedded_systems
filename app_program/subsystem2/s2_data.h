/*
 * s2_data.h
 * Buddy 2 adapters for the shared external data representation.
 */
#ifndef S2_DATA_H
#define S2_DATA_H

#include "../app.h"
#include "../shared/edr.h"

/* Task context on either core. Write exactly EDR_MOTION_SIZE or
 * EDR_ENCODER_SIZE bytes on E_OK; leave output untouched on any error.
 * Encoders return E_NOSPT when disabled, and an all-zero invalid payload
 * before initialization. The encoder timestamp is the boot millisecond
 * operating clock tk_get_otm() sampled inside the sequence-checked count copy.
 * Encoder counts are signed positions represented modulo 2^32; derive
 * interval deltas with unsigned subtraction. No calibration is assumed.
 */
ER s2_motion_encode(uint8_t * const p_payload, const size_t capacity);
ER s2_encoder_encode(uint8_t * const p_payload, const size_t capacity);

#endif /* S2_DATA_H */

/* End of file. */
