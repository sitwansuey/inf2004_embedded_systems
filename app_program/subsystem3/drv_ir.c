#include "drv_ir.h"
#include "drv_ir_adc.h"


bool s3_ir_config_valid(const s3_ir_config_t *p_config)
{
    uint8_t i;
    uint8_t j;

    /* Cast before range checking so a negative/invalid enum is rejected
     * even when the target compiler chooses an unsigned enum type. */
    if ((p_config == NULL) ||
        ((unsigned int)p_config->mode > (unsigned int)S3_IR_ADC))
    {
        return false;
    }
    if (p_config->mode == S3_IR_DISABLED)
    {
        return true;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        uint8_t pin = p_config->pin[i];
        if ((pin > 28U) || ((pin >= 23U) && (pin <= 25U)) || (pin == 19U))
        {
            return false; /* Radio pins and existing S1 LED claim. */
        }
#if TM_CONSOLE_UART
        if (pin < 2U)
        {
            return false;
        }
#endif
        if ((p_config->mode == S3_IR_ADC) && (pin < 26U))
        {
            return false;
        }
        for (j = 0U; j < i; ++j)
        {
            if (p_config->pin[j] == pin)
            {
                return false;
            }
        }
    }
    return true;
}

ER s3_ir_open(s3_ir_t *p_ir, const s3_ir_config_t *p_config)
{
    uint8_t i;

    if ((p_ir == NULL) || !s3_ir_config_valid(p_config))
    {
        return E_PAR;
    }
    if (p_ir->opened)
    {
        return E_OBJ;
    }
    *p_ir = (s3_ir_t){0};
    p_ir->config = *p_config;
    if (p_config->mode == S3_IR_DISABLED)
    {
        return E_NOSPT;
    }
    if (p_config->mode == S3_IR_DIGITAL)
    {
        for (i = 0U; i < S3_SENSOR_COUNT; ++i)
        {
            ER er = gpio_set_pin(p_config->pin[i], GPIO_MODE_IN);
            if (er < E_OK)
            {
                return er;
            }
        }
    }
    else
    {
        ER er = s3_adc_start();
        if (er < E_OK)
        {
            return er;
        }
    }
    p_ir->opened = true;
    return E_OK;
}


bool s3_ir_waiting(const s3_ir_t *p_ir)
{
    return (p_ir != NULL) && p_ir->opened &&
           (p_ir->config.mode == S3_IR_ADC) && p_ir->adc_pending &&
           !p_ir->adc_failed;
}

static ER adc_failure(s3_ir_t *p_ir, ER error)
{
    p_ir->adc_failed = true;
    p_ir->adc_pending = false;
    p_ir->adc_error = error;
    return error;
}

static ER adc_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample)
{
    s3_adc_request_t request = {0};
    s3_adc_reply_t reply = {0};
    uint8_t i;
    bool received = false;
    ER er;

    if (p_ir->adc_failed)
    {
        return p_ir->adc_error;
    }
    if (p_ir->adc_pending)
    {
        /* Deadline wins even if an old response is already buffered. Never
         * cancel or reuse memory underneath a still-running device call. */
        if ((now_ms - p_ir->adc_started_ms) >= S3_ADC_DEADLINE_MS)
        {
            return adc_failure(p_ir, E_TMOUT);
        }
        er = s3_adc_receive(&reply);
        if (er == E_TMOUT)
        {
            return E_TMOUT;
        }
        if (er != E_OK)
        {
            return adc_failure(p_ir, er);
        }
        if (reply.sequence != p_ir->adc_sequence)
        {
            return adc_failure(p_ir, E_IO);
        }
        if (reply.error != E_OK)
        {
            return adc_failure(p_ir, (reply.error < E_OK) ? reply.error : E_IO);
        }
        if ((reply.sample.valid_mask != S3_SENSOR_MASK) ||
            ((now_ms - reply.sample.time_ms) >= S3_ADC_DEADLINE_MS) ||
            ((reply.sample.time_ms - p_ir->adc_started_ms) >= S3_ADC_DEADLINE_MS))
        {
            return adc_failure(p_ir, E_IO);
        }
        for (i = 0U; i < S3_SENSOR_COUNT; ++i)
        {
            if (reply.sample.raw[i] > 4095U)
            {
                return adc_failure(p_ir, E_IO);
            }
        }
        received = true;
        p_ir->adc_pending = false;
    }
    ++p_ir->adc_sequence;
    if (p_ir->adc_sequence == 0U)
    {
        ++p_ir->adc_sequence;
    }
    request.sequence = p_ir->adc_sequence;
    request.issued_ms = now_ms;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        request.channel[i] = (uint8_t)(p_ir->config.pin[i] - 26U);
    }
    er = s3_adc_submit(&request);
    if (er != E_OK)
    {
        return adc_failure(p_ir, er);
    }
    p_ir->adc_started_ms = now_ms;
    p_ir->adc_pending = true;
    if (received)
    {
        *p_sample = reply.sample;
        return E_OK;
    }
    return E_TMOUT;
}

ER s3_ir_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample)
{
    uint8_t i;
    if (p_sample == NULL)
    {
        return E_PAR;
    }
    *p_sample = (s3_sample_t){0};
    p_sample->time_ms = now_ms;
    if ((p_ir == NULL) || !p_ir->opened)
    {
        return E_NOSPT;
    }
    if (p_ir->config.mode == S3_IR_ADC)
    {
        return adc_poll(p_ir, now_ms, p_sample);
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        UINT value = gpio_get_val(p_ir->config.pin[i]);
        if (value > 1U)
        {
            return E_IO;
        }
        p_sample->raw[i] = (uint16_t)value;
    }
    p_sample->valid_mask = S3_SENSOR_MASK;
    return E_OK;
}
