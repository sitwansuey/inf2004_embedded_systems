#include "subsystem3.h"
#include "drv_ir_adc.h"


EXPORT ID mbx_samples;
EXPORT ID mtx_stats;
EXPORT APP_STATS app_stats;

static ID h_events;
static ID h_controller;
static ID h_task;
static bool started;
static bool acquisition_attempted;
static ER acquisition_open_error;
static uint32_t applied_generation;
static s3_adapter_config_t configuration;
static s3_status_t status;
static s3_core_t core;
static s3_ir_t ir;
static struct
{
    s3_mode_t mode;
    bool marker_clear;
    uint32_t generation;
    uint32_t issued_ms;
    uint32_t lease_ms;
} control;

static ER lock(void)
{
    return (mtx_stats > 0) ? tk_loc_mtx(mtx_stats, TMO_POL) : E_OBJ;
}

static ER unlock(void)
{
    return tk_unl_mtx(mtx_stats);
}

static bool lease_valid(uint32_t now_ms)
{
    return (control.mode == S3_MODE_TRACK) && (control.lease_ms > 0U) &&
           ((now_ms - control.issued_ms) < control.lease_ms);
}

EXPORT ER subsystem3_init(void)
{
    T_CMBX mailbox = {0};
    T_CMTX mutex = {0};
    T_CMBF buffer = {0};
    ER er;

    if ((mtx_stats > 0) || (mbx_samples > 0) || (h_events > 0))
    {
        return E_OBJ;
    }
    mailbox.mbxatr = TA_TFIFO | TA_MFIFO;
    mbx_samples = tk_cre_mbx(&mailbox);
    if (mbx_samples < E_OK)
    {
        er = mbx_samples;
        mbx_samples = 0;
        return er;
    }
    mutex.mtxatr = TA_INHERIT;
    mtx_stats = tk_cre_mtx(&mutex);
    if (mtx_stats < E_OK)
    {
        er = mtx_stats;
        mtx_stats = 0;
        (void)tk_del_mbx(mbx_samples);
        mbx_samples = 0;
        return er;
    }
    buffer.mbfatr = TA_TFIFO;
    buffer.maxmsz = (INT)sizeof(s3_event_record_t);
    buffer.bufsz = (INT)(S3_EVENT_CAPACITY *
                        (sizeof(s3_event_record_t) + sizeof(UW)));
    h_events = tk_cre_mbf(&buffer);
    if (h_events < E_OK)
    {
        er = h_events;
        h_events = 0;
        (void)tk_del_mtx(mtx_stats);
        (void)tk_del_mbx(mbx_samples);
        mtx_stats = 0;
        mbx_samples = 0;
        return er;
    }
    configuration = (s3_adapter_config_t){0};
    s3_config_defaults(&configuration.algorithms);
    status = (s3_status_t){0};
    status.output.error = S3_ERROR_CONFIG;
    status.acquisition_error = E_NOSPT;
    control.mode = S3_MODE_PAUSED;
    control.generation = 1U;
    return E_OK;
}

EXPORT ER s3_configure(const s3_adapter_config_t *p_config)
{
    ER er;
    if ((p_config == NULL) || !s3_config_valid(&p_config->algorithms) ||
        !s3_ir_config_valid(&p_config->ir) ||
        ((p_config->ir.mode == S3_IR_DIGITAL) &&
         (p_config->algorithms.line.raw_max != 1U)) ||
        ((p_config->ir.mode == S3_IR_ADC) &&
         (p_config->algorithms.line.raw_max != 4095U)))
    {
        return E_PAR;
    }
    if ((p_config->ir.mode == S3_IR_ADC) && !s3_adc_available())
    {
        return E_NOSPT;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if (started)
    {
        (void)unlock();
        return E_OBJ;
    }
    configuration = *p_config;
    return unlock();
}

EXPORT ER subsystem3_start(void)
{
    T_CTSK task = {0};
    ER er = lock();
    ID task_id;

    if (er < E_OK)
    {
        return er;
    }
    if (started || !s3_core_init(&core, &configuration.algorithms))
    {
        (void)unlock();
        return E_OBJ;
    }
    task.task = (FP)consumer_task;
    task.itskpri = S3_PRI_CONSUMER;
    task.stksz = TASK_STACK_SIZE;
    task.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    task.tskatr |= TA_ASSPRC;
    task.assprc = TP_PRC1;
#endif
    task_id = tk_cre_tsk(&task);
    if (task_id < E_OK)
    {
        (void)unlock();
        return task_id;
    }
    h_task = task_id;
    started = true;
    (void)unlock();
    er = tk_sta_tsk(h_task, 0);
    if (er < E_OK)
    {
        (void)tk_del_tsk(h_task);
        h_task = 0;
        /* The initial task is the only lifecycle caller. Task never ran. */
        started = false;
    }
    return er;
}

EXPORT ER s3_controller_bind(void)
{
    ER er = lock();
    ID caller;
    if (er < E_OK)
    {
        return er;
    }
    caller = tk_get_tid();
    if ((caller <= 0) || ((h_controller != 0) && (h_controller != caller)))
    {
        (void)unlock();
        return E_OACV;
    }
    h_controller = caller;
    return unlock();
}

EXPORT ER s3_controller_control(s3_mode_t mode, bool marker_clear, UW lease_ms)
{
    SYSTIM now;
    ER er;
    if (((unsigned int)mode > (unsigned int)S3_MODE_OVERRIDE) ||
        ((mode == S3_MODE_TRACK) &&
         ((lease_ms == 0U) || (lease_ms > S3_CONTROLLER_MAX_LEASE_MS))) ||
        ((mode != S3_MODE_TRACK) && ((lease_ms != 0U) || marker_clear)))
    {
        return E_PAR;
    }
    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        return er;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if ((h_controller == 0) || (tk_get_tid() != h_controller))
    {
        (void)unlock();
        return E_OACV;
    }
    control.mode = mode;
    control.marker_clear = marker_clear;
    control.issued_ms = (uint32_t)now.lo;
    control.lease_ms = (uint32_t)lease_ms;
    ++control.generation;
    /* Invalidate already-published steering and queued events immediately,
     * even if a sensor read is currently running on the other core. */
    status.output.steering_allowed = false;
    status.generation = control.generation;
    return unlock();
}

EXPORT ER s3_controller_renew(UW lease_ms)
{
    SYSTIM now;
    ER er;
    if ((lease_ms == 0U) || (lease_ms > S3_CONTROLLER_MAX_LEASE_MS))
    {
        return E_PAR;
    }
    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        return er;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if ((h_controller == 0) || (tk_get_tid() != h_controller))
    {
        (void)unlock();
        return E_OACV;
    }
    if (!lease_valid((uint32_t)now.lo))
    {
        (void)unlock();
        return E_OBJ; /* Expiry requires explicit resume, never renewal. */
    }
    control.issued_ms = (uint32_t)now.lo;
    control.lease_ms = (uint32_t)lease_ms;
    return unlock();
}

EXPORT ER s3_status_get(s3_status_t *p_status)
{
    SYSTIM now;
    ER er;
    if (p_status == NULL)
    {
        return E_PAR;
    }
    *p_status = (s3_status_t){0};
    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        return er;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    *p_status = status;
    if (!lease_valid((uint32_t)now.lo) ||
        (((uint32_t)now.lo - status.sample_time_ms) >= S3_STATUS_MAX_AGE_MS))
    {
        p_status->output.steering_allowed = false;
    }
    return unlock();
}

EXPORT ER s3_event_receive(s3_event_record_t *p_record)
{
    SYSTIM now;
    ER er;
    unsigned int i;
    if (p_record == NULL)
    {
        return E_PAR;
    }
    *p_record = (s3_event_record_t){0};
    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        return er;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if ((h_controller == 0) || (tk_get_tid() != h_controller))
    {
        (void)unlock();
        return E_OACV;
    }
    for (i = 0U; i < S3_EVENT_CAPACITY; ++i)
    {
        INT length = tk_rcv_mbf(h_events, p_record, TMO_POL);
        bool safe_event;
        if (length < E_OK)
        {
            *p_record = (s3_event_record_t){0};
            (void)unlock();
            return length;
        }
        if (length != (INT)sizeof(*p_record))
        {
            continue;
        }
        safe_event = (p_record->event.kind == S3_EVENT_STOP_REQUEST) ||
                     (p_record->event.kind == S3_EVENT_FAULT);
        if ((p_record->generation == control.generation) &&
            (((uint32_t)now.lo - p_record->event.time_ms) < S3_EVENT_MAX_AGE_MS) &&
            (safe_event || (lease_valid((uint32_t)now.lo) &&
             (status.output.error == S3_ERROR_NONE) &&
             (status.acquisition_error == E_OK) && status.configured &&
             (((uint32_t)now.lo - status.sample_time_ms) < S3_STATUS_MAX_AGE_MS))))
        {
            return unlock();
        }
    }
    *p_record = (s3_event_record_t){0};
    (void)unlock();
    return E_TMOUT;
}

EXPORT ER s3_service_once(void)
{
    SYSTIM now;
    uint32_t generation;
    s3_mode_t mode;
    bool marker_clear;
    bool active;
    s3_sample_t sample = {0};
    s3_output_t output = {0};
    ER acquisition;
    ER er;

    if ((h_task <= 0) || (tk_get_tid() != h_task))
    {
        return E_OACV;
    }
    er = tk_get_tim(&now);
    if (er < E_OK)
    {
        s3_core_fault(&core, S3_ERROR_STALE);
        if (lock() == E_OK)
        {
            status.output.steering_allowed = false;
            status.output.error = S3_ERROR_STALE;
            (void)unlock();
        }
        return er;
    }
    er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if (!started)
    {
        (void)unlock();
        return E_OBJ;
    }
    generation = control.generation;
    mode = control.mode;
    marker_clear = control.marker_clear;
    active = lease_valid((uint32_t)now.lo);
    (void)unlock();
    if (generation != applied_generation)
    {
        (void)s3_core_control(&core, mode, marker_clear);
        applied_generation = generation;
    }
    if ((mode == S3_MODE_TRACK) && !active)
    {
        (void)s3_core_control(&core, S3_MODE_OVERRIDE, false);
        s3_core_fault(&core, S3_ERROR_STALE);
    }
    if (!acquisition_attempted)
    {
        acquisition_attempted = true;
        acquisition_open_error = s3_ir_open(&ir, &configuration.ir);
    }
    sample.time_ms = (uint32_t)now.lo;
    acquisition = (acquisition_open_error < E_OK) ? acquisition_open_error :
                  s3_ir_poll(&ir, (uint32_t)now.lo, &sample);
    if ((acquisition != E_TMOUT) || !s3_ir_waiting(&ir))
    {
        s3_core_step(&core, &sample, &output);
    }
    else
    {
        if (core.have_sample &&
            (((uint32_t)now.lo - core.last_sample) >=
             configuration.algorithms.max_sample_gap_ms))
        {
            s3_core_fault(&core, S3_ERROR_STALE);
        }
        output.error = core.latched_error;
        output.awaiting_controller = core.awaiting_controller;
    }
    er = lock();
    if (er < E_OK)
    {
        s3_core_fault(&core, S3_ERROR_TRANSPORT);
        return er;
    }
    if (generation != control.generation)
    {
        (void)unlock();
        return E_TMOUT; /* Safety directive won the publication race. */
    }
    if (output.event.kind != S3_EVENT_NONE)
    {
        s3_event_record_t record = {0};
        record.event = output.event;
        record.generation = generation;
        er = tk_snd_mbf(h_events, &record, (INT)sizeof(record), TMO_POL);
        if (er < E_OK)
        {
            s3_core_fault(&core, S3_ERROR_TRANSPORT);
            output.error = S3_ERROR_TRANSPORT;
            output.steering_allowed = false;
            output.awaiting_controller = true;
        }
    }
    status.output = output;
    status.output.event = (s3_event_t){0}; /* Events are consumed only via MBF. */
    status.sample = (acquisition == E_OK) ? sample : (s3_sample_t){0};
    status.sample.time_ms = sample.time_ms;
    status.sample_time_ms = sample.time_ms;
    status.generation = generation;
    status.acquisition_error = acquisition;
    status.configured = configuration.ir.mode != S3_IR_DISABLED;
    if (!status.configured)
    {
        status.output.error = S3_ERROR_CONFIG;
        status.output.steering_allowed = false;
    }
    (void)unlock();
    return (er < E_OK) ? er : acquisition;
}
