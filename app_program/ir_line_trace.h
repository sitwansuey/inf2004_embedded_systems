#ifndef IR_LINE_TRACE_H
#define IR_LINE_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#include "subsystem3/buddy3_line.h"

/* Straddle observer: GP26/27 straddle the line, so both WHITE is centred and
 * a sensor on BLACK is that side touching it. Interprets S3's ADC samples
 * only; no hardware, kernel or barcode authority. */
typedef struct {
    s3_line_t line;
    s3_line_result_t result;
    uint32_t sample_ms;
    bool have_sample,usable;
} ir_line_trace_t;
bool ir_line_observe(ir_line_trace_t *line,const s3_line_config_t *config,
                     const uint16_t raw[3],uint8_t valid_mask,
                     uint32_t sample_ms,uint32_t now_ms,bool source_ok);
#endif
