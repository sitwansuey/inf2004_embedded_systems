#ifndef S5_DATA_H
#define S5_DATA_H
#include "subsystem5.h"
#include <stdbool.h>
#include <stdint.h>
#define S5_DATA_SIZE 84U
#define S5_DATA_SOURCE 5U
#define S5_DATA_KIND 0U
/* Explicit byte representation; no compiler bit fields or struct copies.
 * Separate buffers required. Failure leaves output untouched.
 * Decode requires exact length; encode accepts capacity >= S5_DATA_SIZE. */
bool s5_data_encode(const s5_status_t *value, uint8_t *data, size_t capacity);
bool s5_data_decode(const uint8_t *data, size_t size, s5_status_t *value);
/* Atomic snapshot adapter; returns the original mutex error on failure. */
ER s5_status_encode(uint8_t *data, size_t capacity);
#endif
