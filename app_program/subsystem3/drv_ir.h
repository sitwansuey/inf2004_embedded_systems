#ifndef DRV_IR_H
#define DRV_IR_H

#include "../app.h"
#include "buddy3_core.h"

typedef enum
{
    S3_IR_DISABLED,
    S3_IR_DIGITAL,
    S3_IR_ADC /* Optional worker; requires verified kernel analogue pad config. */
} s3_ir_mode_t;

typedef struct
{
    s3_ir_mode_t mode;
    uint8_t pin[S3_SENSOR_COUNT];
} s3_ir_config_t;

typedef struct
{
    s3_ir_config_t config;
    bool opened;
    bool adc_pending;
    bool adc_failed;
    uint32_t adc_sequence;
    uint32_t adc_started_ms;
    ER adc_error;
} s3_ir_t;

bool s3_ir_config_valid(const s3_ir_config_t *p_config);
ER s3_ir_open(s3_ir_t *p_ir, const s3_ir_config_t *p_config);
/* Sole client is the sampling task; E_TMOUT while waiting is distinguished
 * from a latched timeout by s3_ir_waiting(). Failed ADC stays held until reboot. */
bool s3_ir_waiting(const s3_ir_t *p_ir);
ER s3_ir_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample);
/* Sampler path: submit a current scan and wait on its copied reply, with a
 * bounded kernel timeout. No device call or caller buffer runs in this task.
 * Unlike the legacy polling pipeline, it does not queue the next scan early. */
ER s3_ir_acquire(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample);

#endif
