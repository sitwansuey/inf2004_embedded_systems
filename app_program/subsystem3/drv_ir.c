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
#if BOARD_IR_RAW_BENCH
    tm_printf((UB *)"[S3ADC] client failed error=%d pending=%u\n",
              error, p_ir->adc_pending ? 1U : 0U);
#endif
    p_ir->adc_failed = true;
    p_ir->adc_pending = false;
    p_ir->adc_error = error;
    return error;
}

static ER submit_scan(s3_ir_t *p_ir, uint32_t now_ms)
{
    s3_adc_request_t request = {0};
    uint8_t i;
    ER er;
    ++p_ir->adc_sequence;
    if (p_ir->adc_sequence == 0U) ++p_ir->adc_sequence;
    request.sequence = p_ir->adc_sequence;
    request.issued_ms = now_ms;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
        request.channel[i] = (uint8_t)(p_ir->config.pin[i] - 26U);
    er = s3_adc_submit(&request);
    if (er != E_OK) return adc_failure(p_ir, er);
    p_ir->adc_started_ms = now_ms;
    p_ir->adc_pending = true;
    return E_OK;
}

static ER adc_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample,
                   bool wait)
{
    s3_adc_reply_t reply = {0};
    uint8_t i;
    ER er;

    if (p_ir->adc_failed)
    {
        return p_ir->adc_error;
    }
    if (!p_ir->adc_pending)
    {
        er = submit_scan(p_ir, now_ms);
        if (er != E_OK) return er;
        if (!wait) return E_TMOUT;
    }
    {
        /* Deadline wins even if an old response is already buffered. Never
         * cancel or reuse memory underneath a still-running device call. */
        if ((now_ms - p_ir->adc_started_ms) >= S3_ADC_REPLY_DEADLINE_MS)
        {
            return adc_failure(p_ir, E_TMOUT);
        }
        er = wait ? s3_adc_receive_wait(&reply) : s3_adc_receive(&reply);
        if (wait)
        {
            SYSTIM completed;
            ER clock_error = tk_get_otm(&completed);
            if (clock_error != E_OK) return adc_failure(p_ir, clock_error);
            now_ms = (uint32_t)completed.lo;
            if ((now_ms - p_ir->adc_started_ms) >= S3_ADC_REPLY_DEADLINE_MS)
                return adc_failure(p_ir, E_TMOUT);
        }
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
            ((now_ms - reply.sample.time_ms) >= S3_ADC_REPLY_DEADLINE_MS) ||
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
        p_ir->adc_pending = false;
    }
    if (!wait)
    {
        er = submit_scan(p_ir, now_ms);
        if (er != E_OK) return er;
    }
    *p_sample = reply.sample;
    return E_OK;
}

static ER read_ir(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample, bool wait)
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
        return adc_poll(p_ir, now_ms, p_sample, wait);
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

ER s3_ir_poll(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample)
{
    return read_ir(p_ir, now_ms, p_sample, false);
}

ER s3_ir_acquire(s3_ir_t *p_ir, uint32_t now_ms, s3_sample_t *p_sample)
{
    return read_ir(p_ir, now_ms, p_sample, true);
}
