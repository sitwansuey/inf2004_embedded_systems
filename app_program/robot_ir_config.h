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
    p_config->barcode_retry = BOARD_BARCODE_RETRY;
    /* A marker is complete only once GP28 has not changed for this long. */
    p_config->barcode.end_quiet_ms = BOARD_BARCODE_END_QUIET_MS;
    p_config->map_a = BOARD_MAP_A;
    p_config->map_z = BOARD_MAP_Z;
    p_config->map_b = BOARD_MAP_B;
    p_config->map_c = BOARD_MAP_C;
    p_config->map_d = BOARD_MAP_D;
    p_config->map_f = BOARD_MAP_F;
    p_config->line_straddle = BOARD_LINE_STRADDLE != 0;
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
    p_config->barcode.min_run_ms = BOARD_BARCODE_MIN_RUN_MS;
    /* With encoder positions a fast narrow bar is short in time but not in
     * distance; noise must be short in both (half counts, ~0.17 mm each). */
    p_config->barcode.min_run_position = BOARD_BARCODE_MIN_RUN_POSITION;
    if(BOARD_IR_MODE==3) {
        p_config->line.white[2]=1U-BOARD_BARCODE_DIGITAL_BLACK_LEVEL;
        p_config->line.black[2]=BOARD_BARCODE_DIGITAL_BLACK_LEVEL;
    }
    return s3_config_valid(p_config);
#else
    (void)p_config;
    return false;
#endif
}

#endif
