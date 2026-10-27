#include "subsystem3.h"
#include "../robot_config.h"
#include "drv_ir_adc.h"
#include "event_codec.h"


EXPORT ID mbx_samples;
EXPORT ID mtx_stats;
EXPORT APP_STATS app_stats;

static ID h_events;
#if BOARD_IR_BARCODE_CAPTURE
static ID h_capture;
static uint32_t capture_dropped;
#endif
static ID h_controller;
static ID h_task;
static ID h_sample_cycle;
static bool started;
static bool raw_only;
static bool acquisition_attempted;
static ER acquisition_open_error;
static uint32_t applied_generation;
static s3_adapter_config_t configuration;
static s3_status_t status;
static s3_core_t core;
static s3_ir_t ir;
#if BOARD_IR_MOTOR_SCAN_BENCH
static s3_barcode_t bench_decoder;
static bool bench_observe,bench_have_sample,bench_black;
static uint32_t bench_requested_ms,bench_last_sample;
#endif
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

/* Only the sampler waits briefly for state readers. Other public snapshot
 * operations retain nonblocking behavior. Re-age data after every wait. */
static ER lock_sample(void)
{
    return (mtx_stats > 0) ? tk_loc_mtx(mtx_stats, S3_STATE_LOCK_WAIT_MS) : E_OBJ;
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
    buffer.maxmsz = S3_EVENT_BYTES;
    buffer.bufsz = (INT)(S3_EVENT_CAPACITY *
                        ((S3_EVENT_BYTES + 3U) / 4U * 4U + sizeof(UW)));
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
#if BOARD_IR_BARCODE_CAPTURE
    buffer.maxmsz = (INT)sizeof(s3_capture_record_t);
    buffer.bufsz = (INT)(S3_CAPTURE_CAPACITY *
                        ((sizeof(s3_capture_record_t) + 3U) / 4U * 4U + sizeof(UW)));
    h_capture = tk_cre_mbf(&buffer);
    if (h_capture < E_OK)
    {
        er = h_capture; h_capture = 0;
        (void)tk_del_mbf(h_events); h_events = 0;
        (void)tk_del_mtx(mtx_stats); mtx_stats = 0;
        (void)tk_del_mbx(mbx_samples); mbx_samples = 0;
        return er;
    }
    capture_dropped = 0U;
#endif
    configuration = (s3_adapter_config_t){0};
    s3_config_defaults(&configuration.algorithms);
    status = (s3_status_t){0};
#if BOARD_IR_MOTOR_SCAN_BENCH
    bench_observe=false;bench_have_sample=false;bench_black=false;
    bench_requested_ms=0U;bench_last_sample=0U;s3_barcode_init(&bench_decoder);
#endif
    raw_only = false;
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
    raw_only = false;
    return unlock();
}

EXPORT ER s3_configure_raw(const s3_ir_config_t *p_config)
{
    ER er;
    if (!BOARD_MOTORS_DISABLED) return E_OACV;
    if (p_config == NULL || !s3_ir_config_valid(p_config) || p_config->mode == S3_IR_DISABLED)
        return E_PAR;
    if (p_config->mode == S3_IR_ADC && !s3_adc_available()) return E_NOSPT;
    er = lock();
    if (er < E_OK) return er;
    if (started) { (void)unlock(); return E_OBJ; }
    /* Default core storage is valid, but its surface endpoints are never
     * interpreted or advertised as calibrated in this diagnostic mode. */
    s3_config_defaults(&configuration.algorithms);
    configuration.ir = *p_config;
    raw_only = true;
    return unlock();
}

static void sample_cyclic(void *exinf)
{
    (void)exinf;
    /* Interrupt context: signal only. Never take a mutex, format text or ADC. */
    if (h_task > 0 && started) (void)tk_wup_tsk(h_task);
}

EXPORT ER s3_sample_wait(void)
{
    ER er;
    if (h_task <= 0 || tk_get_tid() != h_task) return E_OACV;
    if (h_sample_cycle <= 0) return tk_dly_tsk(S3_PERIOD_MS); /* disabled IR */
    /* Drop accumulated releases before waiting for a new release. Work that
     * overran a tick cannot cause a burst of duplicate-time ADC requests. */
    er = tk_can_wup(TSK_SELF);
    return er < E_OK ? er : tk_slp_tsk(TMO_FEVR);
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
    if (configuration.ir.mode != S3_IR_DISABLED)
    {
        T_CCYC cycle = {0};
        cycle.cycatr = TA_HLNG | TA_STA;
        cycle.cychdr = (FP)sample_cyclic;
        cycle.cyctim = S3_PERIOD_MS;
        cycle.cycphs = S3_PERIOD_MS;
#if TK_SUPPORT_SMP
        cycle.cycatr |= TA_ASSPRC;
        cycle.assprc = TP_PRC1;
#endif
        h_sample_cycle = tk_cre_cyc(&cycle);
        if (h_sample_cycle < E_OK)
        {
            er = h_sample_cycle; h_sample_cycle = 0;
            (void)tk_del_tsk(task_id);
            (void)unlock();
            return er;
        }
    }
    h_task = task_id;
    started = true;
    (void)unlock();
    er = tk_sta_tsk(h_task, 0);
    if (er < E_OK)
    {
        (void)tk_del_tsk(h_task);
        h_task = 0;
        if (h_sample_cycle > 0) { (void)tk_del_cyc(h_sample_cycle); h_sample_cycle = 0; }
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

#if BOARD_IR_MOTOR_SCAN_BENCH
EXPORT ER s3_barcode_bench_control(bool observe,uint32_t *p_generation)
{
    SYSTIM now;ER er=tk_get_otm(&now);
    if(er<E_OK) return er;
    er=lock_sample();if(er<E_OK) return er;
    if(h_controller<=0 || tk_get_tid()!=h_controller) {(void)unlock();return E_OACV;}
    if(!started || raw_only || configuration.ir.mode!=S3_IR_ADC ||
       configuration.algorithms.barcode_sensor!=2U) {(void)unlock();return E_OBJ;}
    uint32_t generation=status.bench.generation+1U;
    status.bench=(s3_barcode_bench_status_t){0};status.bench.generation=generation;
    bench_observe=observe;bench_have_sample=false;bench_black=false;
    bench_requested_ms=now.lo;s3_barcode_init(&bench_decoder);
    if(p_generation!=NULL) *p_generation=generation;
    return unlock();
}
/* Called only by the sampler while holding its existing publication lock.
 * Latest status polling is not a sample stream; feed the decoder here so a
 * slower status reader cannot discard short bars or invent acquisition gaps. */
static void barcode_bench_sample(const s3_sample_t *sample,ER acquisition)
{
    uint16_t darkness=0U;
    if(!bench_observe || sample->time_ms-bench_requested_ms>INT32_MAX) return;
    status.bench.time_ms=sample->time_ms;
    if(acquisition<E_OK || (sample->valid_mask&4U)==0U ||
       !s3_normalize(sample->raw[2],configuration.algorithms.line.white[2],
                     configuration.algorithms.line.black[2],4095U,&darkness)) {
        status.bench.error=acquisition<E_OK?acquisition:E_IO;bench_observe=false;return;
    }
    if(bench_have_sample && (sample->time_ms-bench_last_sample==0U ||
       sample->time_ms-bench_last_sample>configuration.algorithms.max_sample_gap_ms)) {
        status.bench.error=E_TMOUT;bench_observe=false;return;
    }
    bench_have_sample=true;bench_last_sample=sample->time_ms;++status.bench.samples;
    if(darkness>=configuration.algorithms.line.threshold_high) bench_black=true;
    else if(darkness<=configuration.algorithms.line.threshold_low) bench_black=false;
    status.bench.result=(uint8_t)s3_barcode_feed(&bench_decoder,
        &configuration.algorithms.barcode,bench_black,sample->time_ms,&status.bench.symbol);
    if(status.bench.result!=S3_BARCODE_NONE) bench_observe=false;
}
#endif

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
    er = tk_get_otm(&now);
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
    if (raw_only && mode == S3_MODE_TRACK)
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
    er = tk_get_otm(&now);
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

#if BOARD_IR_MOTOR_SCAN_BENCH
EXPORT ER s3_barcode_bench_status_get(s3_status_t *p_status)
{
    SYSTIM now;ER er;
    if(p_status==NULL) return E_PAR;
    *p_status=(s3_status_t){0};
    er=lock_sample();if(er<E_OK) return er;
    if(h_controller<=0 || tk_get_tid()!=h_controller) {(void)unlock();return E_OACV;}
    /* The publisher may have run during the bounded lock handoff. Read the
     * clock afterward, then let the controller enforce its 30 ms ADC bound. */
    er=tk_get_otm(&now);
    if(er<E_OK) {(void)unlock();return er;}
    *p_status=status;
    if(!lease_valid(now.lo) || now.lo-status.sample_time_ms>=S3_STATUS_MAX_AGE_MS)
        p_status->output.steering_allowed=false;
    er=unlock();if(er<E_OK) *p_status=(s3_status_t){0};
    return er;
}
#endif

EXPORT ER s3_status_get(s3_status_t *p_status)
{
    SYSTIM now;
    ER er;
    if (p_status == NULL)
    {
        return E_PAR;
    }
    *p_status = (s3_status_t){0};
    er = tk_get_otm(&now);
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
    er = tk_get_otm(&now);
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
        uint8_t wire[S3_EVENT_BYTES];
        INT length = tk_rcv_mbf(h_events, wire, TMO_POL);
        bool safe_event;
        if (length < E_OK)
        {
            *p_record = (s3_event_record_t){0};
            (void)unlock();
            return length;
        }
        if (length != S3_EVENT_BYTES || !s3_event_unpack(wire, p_record))
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

#if BOARD_IR_BARCODE_CAPTURE
EXPORT ER s3_capture_receive(s3_sample_t *p_sample, uint32_t *p_dropped)
{
    s3_capture_record_t record = {0};
    INT length;
    if (p_sample == NULL || p_dropped == NULL) return E_PAR;
    *p_sample = (s3_sample_t){0}; *p_dropped = 0U;
    if (h_capture <= 0) return E_OBJ;
    /* The kernel message buffer is already a synchronized copied transfer.
     * No sampler/status mutex is acquired by this cross-core consumer. */
    length = tk_rcv_mbf(h_capture, &record, TMO_POL);
    if (length != (INT)sizeof(record)) return length < E_OK ? length : E_IO;
    *p_sample = record.sample; *p_dropped = record.dropped;
    return E_OK;
}
#endif

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
    er = tk_get_otm(&now);
    if (er < E_OK)
    {
        s3_core_fault(&core, S3_ERROR_STALE);
        if (lock_sample() == E_OK)
        {
            status.output.steering_allowed = false;
            status.output.error = S3_ERROR_STALE;
            (void)unlock();
        }
        return er;
    }
    er = lock_sample();
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
    (void)unlock();
    if (generation != applied_generation)
    {
        (void)s3_core_control(&core, mode, marker_clear);
        applied_generation = generation;
    }
    if (!acquisition_attempted)
    {
        acquisition_attempted = true;
        acquisition_open_error = s3_ir_open(&ir, &configuration.ir);
    }
    /* Device/task startup may have been preempted. Issue a current request,
     * then wait on copied data, never on the legacy ADC device itself. */
    er = tk_get_otm(&now);
    sample.time_ms = (uint32_t)now.lo;
    acquisition = (er < E_OK) ? er :
                  ((acquisition_open_error < E_OK) ? acquisition_open_error :
                   s3_ir_acquire(&ir, (uint32_t)now.lo, &sample));
    if (er < E_OK) s3_core_fault(&core, S3_ERROR_STALE);
    er = tk_get_otm(&now);
    if (er < E_OK)
    {
        acquisition = er;
        sample = (s3_sample_t){0};
        s3_core_fault(&core, S3_ERROR_STALE);
    }
    /* STOP/generation changes and lease expiry can occur during the bounded
     * reply wait. Recheck them before interpreting or publishing a scan. */
    er = lock_sample();
    if (er < E_OK) return er;
    if (generation != control.generation)
    {
        (void)unlock();
        return E_TMOUT;
    }
    er = tk_get_otm(&now);
    if (er < E_OK)
    {
        s3_core_fault(&core, S3_ERROR_STALE);
        status.output.steering_allowed = false;
        status.output.error = S3_ERROR_STALE;
        status.acquisition_error = er;
        status.sample = (s3_sample_t){0}; status.sample_time_ms = 0U;
        (void)unlock(); return er;
    }
    active = lease_valid((uint32_t)now.lo);
    if (acquisition == E_OK &&
        ((uint32_t)now.lo - sample.time_ms >= S3_STATUS_MAX_AGE_MS))
    {
        acquisition = E_TMOUT; sample.valid_mask = 0U;
        s3_core_fault(&core, S3_ERROR_STALE);
    }
    (void)unlock();
    if ((mode == S3_MODE_TRACK) && !active)
    {
        (void)s3_core_control(&core, S3_MODE_OVERRIDE, false);
        s3_core_fault(&core, S3_ERROR_STALE);
    }
    if (raw_only)
    {
        output.error = S3_ERROR_CONFIG;
        output.awaiting_controller = true;
        /* No line classification, events or barcode interpretation before
         * labelled calibration and sensor placement are established. */
    }
    else if ((acquisition != E_TMOUT) || !s3_ir_waiting(&ir))
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
    er = lock_sample();
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
    er = tk_get_otm(&now);
    if (er < E_OK ||
        (acquisition == E_OK && (uint32_t)now.lo - sample.time_ms >= S3_STATUS_MAX_AGE_MS) ||
        (mode == S3_MODE_TRACK && !lease_valid((uint32_t)now.lo)))
    {
        s3_core_fault(&core, S3_ERROR_STALE);
        output.steering_allowed = false; output.awaiting_controller = true;
        output.error = S3_ERROR_STALE; output.event = (s3_event_t){0};
        if (er < E_OK) { acquisition = er; sample.valid_mask = 0U; }
        else if ((uint32_t)now.lo - sample.time_ms >= S3_STATUS_MAX_AGE_MS)
            { acquisition = E_TMOUT; sample.valid_mask = 0U; }
    }
    if (output.event.kind != S3_EVENT_NONE)
    {
        s3_event_record_t record = {0};
        record.event = output.event;
        record.generation = generation;
        uint8_t wire[S3_EVENT_BYTES];
        er = s3_event_pack(wire, &record) ?
             tk_snd_mbf(h_events, wire, S3_EVENT_BYTES, TMO_POL) : E_PAR;
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
#if BOARD_IR_MOTOR_SCAN_BENCH
    barcode_bench_sample(&status.sample,acquisition);
#endif
#if BOARD_IR_BARCODE_CAPTURE
    /* Copy under the existing status lock; no formatting/device wait here.
     * Invalid acquisitions retain a zero valid mask and cannot be decoded. */
    {
        const s3_capture_record_t record = {status.sample, capture_dropped};
        if (tk_snd_mbf(h_capture, &record, (INT)sizeof(record), TMO_POL) < E_OK &&
            capture_dropped != UINT32_MAX) ++capture_dropped;
    }
#endif
    status.generation = generation;
    status.acquisition_error = acquisition;
    status.configured = !raw_only && configuration.ir.mode != S3_IR_DISABLED;
    if (!status.configured)
    {
        status.output.error = S3_ERROR_CONFIG;
        status.output.steering_allowed = false;
    }
    (void)unlock();
    return (er < E_OK) ? er : acquisition;
}
