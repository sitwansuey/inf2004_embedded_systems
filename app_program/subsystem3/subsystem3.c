#include "subsystem3.h"
#include "../robot_config.h"
#include "drv_ir_adc.h"
#include "event_codec.h"


EXPORT ID mtx_stats;

static ID h_events;
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
static struct
{
    s3_mode_t mode;
    bool marker_clear;
    uint32_t generation;
    uint32_t issued_ms;
    uint32_t lease_ms;
    uint32_t barcode_end; /* request generation, see s3_controller_barcode_end */
    uint32_t map_generation; /* see s3_controller_set_map */
    uint8_t map[26]; /* full override table, S3_MAP_DEFAULT = codebook */
} control;
static uint32_t applied_barcode_end;
static uint32_t applied_map;
#if defined(S2_ENCODER_ENABLED) && S2_ENCODER_ENABLED
#include "../subsystem2/subsystem2.h"
/* Cumulative wheel travel for distance-based barcode widths: half encoder
 * counts of |left+right| per scan, so forward and reverse both advance it
 * and a spin in place does not. */
static uint32_t travel_position;
static uint32_t travel_m1, travel_m2;
static bool travel_seen;
static void attach_position(s3_sample_t *p_sample)
{
    s2_encoder_status_t e;
    p_sample->position_valid = false;
    if ((s2_encoder_get_status(&e) < E_OK) || !e.valid)
    {
        travel_seen = false;
        return;
    }
    uint32_t m1 = (uint32_t)e.motor1_forward_counts, m2 = (uint32_t)e.motor2_forward_counts;
    if (travel_seen)
    {
        int64_t sum = (int64_t)(int32_t)(m1 - travel_m1) + (int64_t)(int32_t)(m2 - travel_m2);
        if (sum < 0) sum = -sum;
        if (sum > 2000) /* >35 cm in one scan: a discontinuity, not travel */
        {
            travel_m1 = m1; travel_m2 = m2;
            return;
        }
        travel_position += (uint32_t)sum;
        p_sample->position_valid = true;
    }
    travel_m1 = m1; travel_m2 = m2; travel_seen = true;
    p_sample->position = travel_position;
}
#endif

static ER lock(void)
{
    return (mtx_stats > 0) ? tk_loc_mtx(mtx_stats, TMO_POL) : E_OBJ;
}

/* The sampler and the controller's mode, lease and map requests wait briefly
 * for state readers (a telemetry copy on the other core, or the sampler
 * preempted while publishing); with priority inheritance the holder finishes
 * at once. Snapshot reads, event drains and barcode-end stay nonblocking.
 * Re-age data after every wait. */
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
    T_CMTX mutex = {0};
    T_CMBF buffer = {0};
    ER er;

    if ((mtx_stats > 0) || (h_events > 0))
    {
        return E_OBJ;
    }
    mutex.mtxatr = TA_INHERIT;
    mtx_stats = tk_cre_mtx(&mutex);
    if (mtx_stats < E_OK)
    {
        er = mtx_stats;
        mtx_stats = 0;
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
        mtx_stats = 0;
        return er;
    }
    configuration = (s3_adapter_config_t){0};
    s3_config_defaults(&configuration.algorithms);
    status = (s3_status_t){0};
    raw_only = false;
    status.output.error = S3_ERROR_CONFIG;
    status.acquisition_error = E_NOSPT;
    control.mode = S3_MODE_PAUSED;
    control.generation = 1U;
    for (unsigned int i = 0U; i < 26U; ++i) control.map[i] = (uint8_t)S3_MAP_DEFAULT;
    return E_OK;
}

EXPORT ER s3_configure(const s3_adapter_config_t *p_config)
{
    ER er;
    if ((p_config == NULL) || !s3_config_valid(&p_config->algorithms) ||
        !s3_ir_config_valid(&p_config->ir) ||
        ((p_config->ir.mode == S3_IR_DIGITAL) &&
         (p_config->algorithms.line.raw_max != 1U)) ||
        ((p_config->ir.mode >= S3_IR_ADC) &&
         (p_config->algorithms.line.raw_max != 4095U)))
    {
        return E_PAR;
    }
    if ((p_config->ir.mode >= S3_IR_ADC) && !s3_adc_available())
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
    if (p_config->mode >= S3_IR_ADC && !s3_adc_available()) return E_NOSPT;
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
    /* Bounded wait: a briefly busy status lock must not read as a transport
     * failure. E_TMOUT means nothing changed; retry next period. */
    er = lock_sample();
    if (er < E_OK)
    {
        return er;
    }
    /* Lease time is taken after the wait, so the lease is never shortened. */
    er = tk_get_otm(&now);
    if (er < E_OK)
    {
        (void)unlock();
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

EXPORT ER s3_controller_barcode_end(void)
{
    ER er = lock();
    if (er < E_OK)
    {
        return er;
    }
    if ((h_controller == 0) || (tk_get_tid() != h_controller))
    {
        (void)unlock();
        return E_OACV;
    }
    ++control.barcode_end;
    return unlock();
}

EXPORT ER s3_controller_set_map(UINT letter, UINT nav)
{
    ER er;
    if ((letter > 26U) || (nav > (UINT)S3_NAV_UTURN) || ((letter == 0U) && (nav != 0U)))
    {
        return E_PAR;
    }
    /* Rare operator command: a bounded wait, so a sampler holding the lock
     * does not turn it into a rejected ACK. */
    er = lock_sample();
    if (er < E_OK)
    {
        return er;
    }
    if ((h_controller == 0) || (tk_get_tid() != h_controller))
    {
        (void)unlock();
        return E_OACV;
    }
    if (letter == 0U)
    {
        for (unsigned int i = 0U; i < 26U; ++i) control.map[i] = (uint8_t)S3_MAP_DEFAULT;
    }
    else control.map[letter - 1U] = (uint8_t)nav;
    ++control.map_generation;
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
    /* Bounded wait, as s3_controller_control: E_TMOUT leaves the lease as it
     * was (retry next period); only E_OBJ means it has expired. */
    er = lock_sample();
    if (er < E_OK)
    {
        return er;
    }
    er = tk_get_otm(&now);
    if (er < E_OK)
    {
        (void)unlock();
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
        bool decoded_turn;
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
        /* A decoded turn was checked when it was made (fresh sample, live
         * lease, this generation). A later sampler latch or a late status
         * must not lose it: S3 then waits for the controller with no event.
         * Deliver it on generation and age alone. */
        decoded_turn = p_record->event.kind == S3_EVENT_NAVIGATION;
        /* Other requests need a healthy, current status. "Current" is the
         * sampler's own gap limit (a gap above max_sample_gap_ms faults), so
         * a status this old is never rejected while S3 itself still runs. */
        if ((p_record->generation == control.generation) &&
            (((uint32_t)now.lo - p_record->event.time_ms) < S3_EVENT_MAX_AGE_MS) &&
            (safe_event || decoded_turn || (lease_valid((uint32_t)now.lo) &&
             (status.output.error == S3_ERROR_NONE) &&
             (status.acquisition_error == E_OK) && status.configured &&
             (((uint32_t)now.lo - status.sample_time_ms) <=
              configuration.algorithms.max_sample_gap_ms))))
        {
            return unlock();
        }
    }
    *p_record = (s3_event_record_t){0};
    (void)unlock();
    return E_TMOUT;
}

#ifndef S3_LOG
#define S3_LOG tm_printf
#endif
static uint8_t logged_partial;
static uint8_t logged_abort;
/* One USB log line per partial capture or fault-cut read, so the read rate
 * can be measured; no MQTT event. S3 task only, outside the lock (the core
 * is private to this task). */
static void log_barcode_counts(void)
{
    if (core.barcode_partial_count != logged_partial)
    {
        logged_partial = core.barcode_partial_count;
        (void)S3_LOG((const UB *)"[S3] partial barcode: %u elements, not a read (partial count %u)\n",
                     (unsigned int)core.partial_elements, (unsigned int)logged_partial);
    }
    if (core.barcode_abort_count != logged_abort)
    {
        logged_abort = core.barcode_abort_count;
        (void)S3_LOG((const UB *)"[S3] barcode read cut short by a sampler fault: %u elements, counted as failed (abort count %u)\n",
                     (unsigned int)core.last_elements, (unsigned int)logged_abort);
    }
}

EXPORT ER s3_service_once(void)
{
    SYSTIM now;
    uint32_t generation;
    uint32_t barcode_end;
    uint32_t map_generation;
    uint8_t map[26];
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
    barcode_end = control.barcode_end;
    map_generation = control.map_generation;
    for (unsigned int i = 0U; i < 26U; ++i) map[i] = control.map[i];
    (void)unlock();
    if (generation != applied_generation)
    {
        (void)s3_core_control(&core, mode, marker_clear);
        applied_generation = generation;
    }
    if (barcode_end != applied_barcode_end)
    {
        s3_core_barcode_end(&core);
        applied_barcode_end = barcode_end;
    }
    if (map_generation != applied_map)
    {
        (void)s3_core_set_map(&core, 0U, 0U);
        for (unsigned int i = 0U; i < 26U; ++i)
        {
            if (map[i] != S3_MAP_DEFAULT) (void)s3_core_set_map(&core, i + 1U, map[i]);
        }
        applied_map = map_generation;
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
#if defined(S2_ENCODER_ENABLED) && S2_ENCODER_ENABLED
    if (acquisition == E_OK) attach_position(&sample);
#endif
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
        output.error = (uint8_t)((unsigned)core.latched_error & 7U);
        output.awaiting_controller = core.awaiting_controller;
    }
    /* Latched diagnostics survive paths that do not step the core. */
    s3_core_barcode_summary(&core, &output);
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
        /* The sample was fresh when it was interpreted: a turn decoded from
         * it survives a late publication (same generation, live lease). The
         * core already holds its marker and waits for the controller. */
        bool keep_turn = (er == E_OK) && (output.event.kind == S3_EVENT_NAVIGATION) &&
                         lease_valid((uint32_t)now.lo);
        s3_core_fault(&core, S3_ERROR_STALE);
        s3_core_barcode_summary(&core, &output);
        output.steering_allowed = false; output.awaiting_controller = true;
        output.error = S3_ERROR_STALE;
        if (!keep_turn) output.event = (s3_event_t){0};
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
    status.generation = generation;
    status.acquisition_error = acquisition;
    status.configured = !raw_only && configuration.ir.mode != S3_IR_DISABLED;
    if (!status.configured)
    {
        status.output.error = S3_ERROR_CONFIG;
        status.output.steering_allowed = false;
    }
    (void)unlock();
    log_barcode_counts();
    return (er < E_OK) ? er : acquisition;
}
