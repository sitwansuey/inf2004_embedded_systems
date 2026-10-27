/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"

EXPORT ID mbf_summary;
EXPORT ID flg_events;
EXPORT ID tid_s4_sensor;
EXPORT ID tid_s4_monitor;

static ID status_lock;
static s4_sensor_status_t published_status;

uint64_t s4_kernel_now_us(void *p_context)
{
    SYSTIM now = { 0 };

    (void)p_context;
    /* Operating time starts at zero. The unset TRON wall clock may be
     * negative: casting it to uint64_t and multiplying would wrap. */
    if (tk_get_otm(&now) < E_OK) return UINT64_MAX;
    return (((uint64_t)(UW)now.hi << 32) | (uint64_t)now.lo) * 1000u;
}

ER s4_status_publish(const s4_sensor_status_t *p_status)
{
    ER result;

    if (p_status == NULL)
    {
        return E_PAR;
    }
    if (status_lock <= E_OK)
    {
        return E_OBJ;
    }
    result = tk_loc_mtx(status_lock, S4_LOCK_WAIT_MS);
    if (result >= E_OK)
    {
        uint32_t lines = published_status.legacy_lines_received;

        published_status = *p_status;
        /* This counter belongs to the monitor, under the same mutex. */
        published_status.legacy_lines_received = lines;
        result = tk_unl_mtx(status_lock);
    }
    return result;
}

ER s4_note_legacy_line(void)
{
    ER result = (status_lock > E_OK) ?
                tk_loc_mtx(status_lock, S4_LOCK_WAIT_MS) : E_OBJ;

    if (result >= E_OK)
    {
        published_status.legacy_lines_received++;
        result = tk_unl_mtx(status_lock);
    }
    return result;
}

ER s4_sensor_get_status(s4_sensor_status_t *p_status)
{
    s4_sensor_status_t copy;
    ER result;

    if (p_status == NULL)
    {
        return E_PAR;
    }
    if (status_lock <= E_OK)
    {
        return E_OBJ;
    }
    result = tk_loc_mtx(status_lock, S4_LOCK_WAIT_MS);
    if (result < E_OK)
    {
        return result;
    }
    copy = published_status;
    result = tk_unl_mtx(status_lock);
    if (result >= E_OK)
    {
        uint64_t now_us = s4_kernel_now_us(NULL);
        if (now_us == UINT64_MAX) return E_SYS;
        s4_data_snapshot(&copy, now_us, p_status);
    }
    return result;
}

ER subsystem4_init(void)
{
    T_CMTX mutex = { 0 };
    T_CMBF buffer = { 0 };
    T_CFLG flags = { 0 };
    ER result;

    if ((status_lock > E_OK) || (mbf_summary > E_OK) || (flg_events > E_OK))
    {
        return E_OBJ; /* Lifecycle is init once, then start once. */
    }
    s4_data_reset(&published_status);
    mutex.mtxatr = TA_INHERIT;
    status_lock = tk_cre_mtx(&mutex);
    if (status_lock <= E_OK)
    {
        return status_lock;
    }
    flags.flgatr = TA_TFIFO | TA_WMUL;
    flg_events = tk_cre_flg(&flags);
    if (flg_events <= E_OK)
    {
        result = flg_events;
        (void)tk_del_mtx(status_lock);
        status_lock = 0;
        return result;
    }
    buffer.mbfatr = TA_TFIFO;
    buffer.bufsz = 4 * S4_SUMMARY_MAX;
    buffer.maxmsz = S4_SUMMARY_MAX;
    mbf_summary = tk_cre_mbf(&buffer);
    if (mbf_summary <= E_OK)
    {
        result = mbf_summary;
        (void)tk_del_flg(flg_events);
        (void)tk_del_mtx(status_lock);
        flg_events = 0;
        status_lock = 0;
        return result;
    }
    return E_OK;
}

ER subsystem4_start(void)
{
    if (status_lock <= E_OK)
    {
        return E_OBJ;
    }
    if ((tid_s4_sensor > E_OK) || (tid_s4_monitor > E_OK))
    {
        return E_OBJ;
    }
    /* app_start_task ignores affinity in SMP=0. In SMP=1 the only I2C1
     * owner is pinned to physical core 1; logging may run on either core. */
    tid_s4_sensor = app_start_task(s4_sensor_task, S4_PRI_SENSOR,
                                    TP_PRC2, "s4_sensor_task");
    if (tid_s4_sensor <= E_OK)
    {
        return tid_s4_sensor;
    }
    tid_s4_monitor = app_start_task(monitor_task, S4_PRI_MONITOR,
                                     0, "s4_monitor_task");
    return (tid_s4_monitor <= E_OK) ? tid_s4_monitor : E_OK;
}
