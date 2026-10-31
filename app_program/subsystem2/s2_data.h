#ifndef S2_DATA_H
#define S2_DATA_H
#include "../app.h"
#include "../shared/edr.h"
<<<<<<< HEAD
/* Task context. Write four bytes (two EDR16 records) on E_OK; otherwise leave
 * output untouched. Motion order: IDs 11,12; encoder order: IDs 1,2.
 * Publish each record separately. E_OBJ for uninitialized/invalid encoders.
 * Timestamps/validity/full counts are LOCAL s2_*_get_status() fields only. */
ER s2_motion_encode(uint8_t * const output, const size_t capacity);
ER s2_encoder_encode(uint8_t * const output, const size_t capacity);
#endif
=======

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
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
