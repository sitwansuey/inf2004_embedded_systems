#ifndef IR_LINE_PROGRESS_H
#define IR_LINE_PROGRESS_H
#include <stdbool.h>
#include <stdint.h>
#define IR_LINE_PROGRESS_TIMEOUT_MS 750U
typedef struct {
    uint32_t counts[2],progressed_ms[2];
    uint8_t required_mask:2;
    bool initialized;
} ir_line_progress_t;
/* Encoder-change watchdog, not calibrated speed/distance or stall diagnosis.
 * Required wheels must show positive forward counts within750ms. */
bool ir_line_progress_check(ir_line_progress_t *state,unsigned command,
                            const uint32_t counts[2],uint32_t now_ms,bool valid);
#endif
