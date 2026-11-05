/* Board layout confirmed by the user on 22 October 2026. Selects
 * sensor roles and optional measured ADC endpoints; never arms control. */
#ifndef ROBOT_IR_CONFIG_H
#define ROBOT_IR_CONFIG_H

#include "robot_config.h"
#include "subsystem3/buddy3_core.h"

static inline void robot_ir_apply_layout(s3_config_t *p_config)
{
    const int16_t positions[S3_SENSOR_COUNT] = BOARD_IR_POSITIONS;
    uint8_t i;
    p_config->line.line_mask = BOARD_IR_LINE_MASK;
    p_config->barcode_sensor = BOARD_BARCODE_SENSOR;
    p_config->line.center_white = BOARD_IR_CENTER_WHITE;
    p_config->barcode_retry = BOARD_BARCODE_RETRY;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
        p_config->line.position[i] = positions[i];
}

/* Shared by startup and host replay; unmeasured ADC defaults stay disabled. */
static inline bool robot_ir_apply_adc_calibration(s3_config_t *p_config)
{
#if defined(BOARD_IR_ADC_CALIBRATED) && BOARD_IR_ADC_CALIBRATED
    const uint16_t white[S3_SENSOR_COUNT] = BOARD_IR_ADC_WHITE;
    const uint16_t black[S3_SENSOR_COUNT] = BOARD_IR_ADC_BLACK;
    uint8_t i;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i) {
        p_config->line.white[i] = white[i];
        p_config->line.black[i] = black[i];
    }
    /* BarcodeDemo temporarily disables the pulse-duration floor (zero).
     * Other calibrated profiles retain their 20 ms floor. */
    p_config->barcode.min_run_ms = BOARD_BARCODE_MIN_RUN_MS;
    return s3_config_valid(p_config);
#else
    (void)p_config;
    return false;
#endif
}

#endif
