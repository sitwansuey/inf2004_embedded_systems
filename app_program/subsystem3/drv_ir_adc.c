#include "drv_ir_adc.h"
#include "../../device/adc/sysdepend/rp2040/adc_cnf_sysdep.h"
#include "../../config/config_device.h"

static ID h_requests;
static ID h_replies;
static ID h_worker;
static ID h_client;
static ID h_device;
static bool worker_failed;

bool s3_adc_available(void)
{
    /* These flags configure pads at kernel boot. No runtime pin takeover.
     * All three channels are required; GP29/ADC3 remains radio-owned. */
#if DEVCNF_USE_ADC && DEVCONF_ADC_PIN_INIT_0 && DEVCONF_ADC_PIN_INIT_1 && DEVCONF_ADC_PIN_INIT_2
    return true;
#else
    return false;
#endif
}

ER s3_adc_start(void)
{
    T_CMBF buffer = {0};
    T_CTSK task = {0};
    ER er;
    ID client = tk_get_tid();

    if (!s3_adc_available())
    {
        return E_NOSPT;
    }
    if ((h_worker > 0) || (h_requests > 0) || (h_replies > 0))
    {
        return E_OBJ;
    }
    if (client <= 0)
    {
        return E_OACV;
    }
    buffer.mbfatr = TA_TFIFO;
    buffer.maxmsz = (INT)sizeof(s3_adc_request_t);
    buffer.bufsz = (INT)(sizeof(s3_adc_request_t) + sizeof(UW));
    h_requests = tk_cre_mbf(&buffer);
    if (h_requests < E_OK)
    {
        er = h_requests;
        h_requests = 0;
        return er;
    }
    buffer.maxmsz = (INT)sizeof(s3_adc_reply_t);
    buffer.bufsz = (INT)(sizeof(s3_adc_reply_t) + sizeof(UW));
    h_replies = tk_cre_mbf(&buffer);
    if (h_replies < E_OK)
    {
        er = h_replies;
        h_replies = 0;
        goto release_requests;
    }
    task.task = (FP)s3_adc_worker;
    task.itskpri = S3_ADC_WORKER_PRI;
    task.stksz = TASK_STACK_SIZE;
    task.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC1;
#endif
    h_worker = tk_cre_tsk(&task);
    if (h_worker < E_OK)
    {
        er = h_worker;
        h_worker = 0;
        goto release_replies;
    }
    h_client = client;
    er = tk_sta_tsk(h_worker, 0);
    if (er == E_OK)
    {
        return E_OK;
    }
    (void)tk_del_tsk(h_worker);
    h_worker = 0;
    h_client = 0;
release_replies:
    (void)tk_del_mbf(h_replies);
    h_replies = 0;
release_requests:
    (void)tk_del_mbf(h_requests);
    h_requests = 0;
    return er;
}

ER s3_adc_submit(const s3_adc_request_t *p_request)
{
    if (p_request == NULL)
    {
        return E_PAR;
    }
    if ((h_client <= 0) || (tk_get_tid() != h_client))
    {
        return E_OACV;
    }
    return tk_snd_mbf(h_requests, p_request, (INT)sizeof(*p_request), TMO_POL);
}

static ER receive_reply(s3_adc_reply_t *p_reply, TMO timeout)
{
    INT length;
    if (p_reply == NULL)
    {
        return E_PAR;
    }
    *p_reply = (s3_adc_reply_t){0};
    if ((h_client <= 0) || (tk_get_tid() != h_client))
    {
        return E_OACV;
    }
    length = tk_rcv_mbf(h_replies, p_reply, timeout);
    if (length < E_OK)
    {
        return length;
    }
    return (length == (INT)sizeof(*p_reply)) ? E_OK : E_IO;
}

ER s3_adc_receive(s3_adc_reply_t *p_reply)
{
    return receive_reply(p_reply, TMO_POL);
}

ER s3_adc_receive_wait(s3_adc_reply_t *p_reply)
{
    return receive_reply(p_reply, S3_ADC_REPLY_WAIT_MS);
}

static ER check_deadline(uint32_t issued_ms, SYSTIM *p_time)
{
    ER er = tk_get_otm(p_time);
    if (er < E_OK)
    {
        return er;
    }
    return (((uint32_t)p_time->lo - issued_ms) < S3_ADC_DEADLINE_MS) ?
            E_OK : E_TMOUT;
}

static ER acquire(const s3_adc_request_t *p_request, s3_sample_t *p_sample)
{
    uint8_t i;
    uint8_t mask = 0U;
    SYSTIM now;
    ER er;

    if (p_request->sequence == 0U)
    {
        return E_PAR;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        uint8_t ch = p_request->channel[i];
        if ((ch > 2U) || ((mask & (1U << ch)) != 0U))
        {
            return E_PAR;
        }
        mask |= (uint8_t)(1U << ch);
    }
    er = check_deadline(p_request->issued_ms, &now);
    if (er < E_OK)
    {
        return er;
    }
    p_sample->time_ms = (uint32_t)now.lo; /* Oldest channel, not delivery time. */
    if (h_device <= 0)
    {
        h_device = tk_opn_dev((const UB *)"adca", TD_READ | TD_EXCL);
        if (h_device <= 0)
        {
            er = (h_device < 0) ? h_device : E_IO;
            h_device = 0;
            return er;
        }
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        UW value = 0U;
        SZ actual = 0;
        er = check_deadline(p_request->issued_ms, &now);
        if (er < E_OK)
        {
            return er;
        }
        /* Only this worker opens/reads the kernel-owned ADC. Never pass a
         * client stack pointer to the legacy synchronous device driver. */
        er = tk_srea_dev(h_device, (W)p_request->channel[i], &value, 1, &actual);
        if (er < E_OK)
        {
            return er;
        }
        er = check_deadline(p_request->issued_ms, &now);
        if (er < E_OK)
        {
            return er;
        }
        if ((actual != 1) || (value > 4095U))
        {
            return E_IO;
        }
        p_sample->raw[i] = (uint16_t)value;
    }
    p_sample->valid_mask = S3_SENSOR_MASK;
    return E_OK;
}

ER s3_adc_worker_once(TMO timeout)
{
    s3_adc_request_t request = {0};
    s3_adc_reply_t reply = {0};
    INT length;
    ER er;
    if ((h_worker <= 0) || (tk_get_tid() != h_worker))
    {
        return E_OACV;
    }
    if (worker_failed)
    {
        return E_OBJ;
    }
    length = tk_rcv_mbf(h_requests, &request, timeout);
    if (length < E_OK)
    {
        return length;
    }
    reply.sequence = request.sequence;
    reply.error = (length == (INT)sizeof(request)) ?
                   acquire(&request, &reply.sample) : E_IO;
    if (reply.error != E_OK)
    {
        /* A timed-out conversion can leave a stale FIFO/IRQ. Never retry on
         * this boot. The kernel owner must repair/reset its device safely. */
        worker_failed = true;
#if BOARD_IR_RAW_BENCH
        tm_printf((UB *)"[S3ADC] worker failed error=%d request=%u\n",
                  reply.error, request.sequence);
#endif
        reply.sample = (s3_sample_t){0};
        er = tk_snd_mbf(h_replies, &reply, (INT)sizeof(reply), TMO_POL);
        return (er < E_OK) ? er : reply.error;
    }
    er = tk_snd_mbf(h_replies, &reply, (INT)sizeof(reply), TMO_POL);
    if (er != E_OK)
    {
        worker_failed = true;
    }
    return er;
}

void s3_adc_worker(INT stacd, void *exinf)
{
    ER er;
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        er = s3_adc_worker_once(TMO_FEVR);
        if (er != E_OK)
        {
            /* Do not spin after a broken transport/device context. No task
             * forcibly cancels this worker while a legacy read is active. */
            tk_ext_tsk();
            return;
        }
    }
}
