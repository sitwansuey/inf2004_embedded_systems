#ifndef IR_LINE_TRACE_H
#define IR_LINE_TRACE_H
#include "ir_motor_scan.h"
#include "subsystem3/buddy3_line.h"

/* Confirmed straddling geometry: both GP26/27 see WHITE when centred.
 * The same readings also occur off-track on white: this is a supervised
 * timed bench, not reliable automatic line-presence/loss detection. */
#define IR_LINE_START_HOLD_MS 100U
enum { IR_LINE_STOP=0,IR_LINE_FORWARD=1,IR_LINE_RIGHT=5,IR_LINE_LEFT=6 };
typedef struct {
    s3_line_t line;
    s3_line_result_t result;
    uint32_t sample_ms,white_since;
    bool have_sample,white_start,usable;
} ir_line_trace_t;
bool ir_line_observe(ir_line_trace_t *line,const s3_line_config_t *config,
                     const uint16_t raw[3],uint8_t valid_mask,
                     uint32_t sample_ms,uint32_t now_ms,bool source_ok);
bool ir_line_start_ready(const ir_line_trace_t *line);
unsigned ir_line_direction(const ir_line_trace_t *line);
/* Explicit timed user test: both BLACK keeps forward output; invalid data
 * still requests STOP. No encoder-progress/course-pattern stop here. */
unsigned ir_line_continuous_direction(const ir_line_trace_t *line);
void ir_line_enforce(ir_scan_state_t *scan,const ir_line_trace_t *line);
#endif
