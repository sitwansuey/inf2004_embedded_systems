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
    p_config->map_a = BOARD_MAP_A;
    p_config->map_z = BOARD_MAP_Z;
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
    /* This measured ADC profile samples every 10 ms. Require two complete
     * sample periods; one-sample pulses remain rejected. Generic defaults,
     * hysteresis, sample freshness and marker timeout are unchanged. */
    p_config->barcode.min_run_ms = 20U;
    return s3_config_valid(p_config);
#else
    (void)p_config;
    return false;
#endif
}

#endif
