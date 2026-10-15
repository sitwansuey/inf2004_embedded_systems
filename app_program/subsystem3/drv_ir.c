#include "drv_ir.h"


bool s3_ir_config_valid(const s3_ir_config_t *p_config)
{
    uint8_t i;
    uint8_t j;

    if ((p_config == NULL) || (p_config->mode < S3_IR_DISABLED) ||
        (p_config->mode > S3_IR_ADC))
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
        /* msdrvif.c calls the ADC read function synchronously and ignores
         * TMO_POL; adc_rp2040.c can wait 1000 ms and also has READY busy waits.
         * ADC stays owned by the kernel. Require its owner to provide bounded
         * sampling before implementing this interface. Never initialize the
         * Pico SDK ADC in parallel with the kernel driver. */
        return E_NOSPT;
    }
    p_ir->opened = true;
    return E_OK;
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
    if ((p_ir == NULL) || !p_ir->opened || (p_ir->config.mode != S3_IR_DIGITAL))
    {
        return E_NOSPT;
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
