#ifndef S2_DATA_H
#define S2_DATA_H
#include "../app.h"
#include "../shared/edr.h"

/* Task context. Write EDR16_MOTION_SIZE or EDR16_ENCODER_SIZE bytes on
 * E_OK; otherwise leave output untouched. Each contains two EDR16 records:
 * motion IDs 11,12; encoder IDs 1,2. Publish each record separately.
 * External encoder counts are modulo 4096. E_OBJ during encoder startup. */
ER s2_motion_encode(uint8_t * const output, const size_t capacity);
ER s2_encoder_encode(uint8_t * const output, const size_t capacity);

/* Internal vehicle/S4 snapshot, never an external EDR16 payload. Task context
 * on either core. Write EDR_ENCODER_SIZE bytes on E_OK, leave output untouched
 * on errors. Preserve full-width signed cumulative forward counts modulo 2^32,
 * GPIO levels and the boot-millisecond timestamp from the coherent getter.
 * Startup returns the canonical all-zero invalid payload; E_NOSPT if disabled.
 * Derive interval deltas with unsigned subtraction. No calibration is assumed. */
ER s2_encoder_snapshot_encode(uint8_t * const output, const size_t capacity);

#endif /* S2_DATA_H */

/* End of file. */
