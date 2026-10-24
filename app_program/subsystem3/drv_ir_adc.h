/* ADC transport: one client task, one ADC-owning worker, copied messages. */
#ifndef DRV_IR_ADC_H
#define DRV_IR_ADC_H

#include "../app.h"
#include "buddy3_core.h"

#define S3_ADC_DEADLINE_MS 20U
/* tk_dly_tsk(10) resumes at +20 ms with this kernel's 10 ms guard tick.
 * Keep conversion bounded to 20 ms, but allow delivery until (not including)
 * the existing 30 ms status freshness limit. Oldest-channel time is retained. */
#define S3_ADC_REPLY_DEADLINE_MS 30U
#define S3_ADC_REPLY_WAIT_MS 1
#define S3_ADC_WORKER_PRI 17

typedef struct
{
    uint32_t sequence;
    uint32_t issued_ms;
    uint8_t channel[S3_SENSOR_COUNT];
} s3_adc_request_t;

typedef struct
{
    uint32_t sequence;
    s3_sample_t sample;
    ER error;
} s3_adc_reply_t;

/* Start from the sampling task, once. No device calls in the client path.
 * Kernel allocations occur only here; data-plane storage is bounded. */
bool s3_adc_available(void);
ER s3_adc_start(void);
ER s3_adc_submit(const s3_adc_request_t *p_request);
ER s3_adc_receive(s3_adc_reply_t *p_reply);
/* Bounded kernel wait, sampling client only. With a 10 ms tick this timeout
 * can take 20 ms plus dispatch jitter; completion normally wakes it sooner. */
ER s3_adc_receive_wait(s3_adc_reply_t *p_reply);
/* Worker-only entry points. The legacy device read may block in this task. */
ER s3_adc_worker_once(TMO timeout);
void s3_adc_worker(INT stacd, void *exinf);

#endif
