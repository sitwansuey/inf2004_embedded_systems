#ifndef DRV_IR_H
#define DRV_IR_H

#include "../app.h"
#include "buddy3_core.h"

typedef enum
{
    S3_IR_DISABLED,
    S3_IR_DIGITAL,
    S3_IR_ADC /* Explicit E_NOSPT: current kernel ADC blocks up to 1000 ms. */
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
} s3_ir_t;

bool s3_ir_config_valid(const s3_ir_config_t *p_config);
ER s3_ir_open(s3_ir_t *p_ir, const s3_ir_config_t *p_config);
/* Sole caller is Buddy 3's sampling task. No dynamic storage or ADC register
 * ownership. E_NOSPT = disabled or ADC deadline contract unavailable. */
ER s3_ir_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample);

#endif
