/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"

EXPORT ID tid_s4_sensor;
EXPORT ID tid_s4_monitor;

static ID                status_lock;
static s4_sensor_cache_t published_status;

uint64_t
s4_kernel_now_us (void * p_context)
{
    SYSTIM now = {0};

    (void) p_context;
    /* Operating time starts at zero. The unset TRON wall clock may be
     * negative: casting it to uint64_t and multiplying would wrap. */
    if (tk_get_otm(&now) < E_OK)
    {
        return UINT64_MAX;
    }
    return s4_operating_ms_to_us(((uint64_t) (UW) now.hi << 32) |
                                 (uint64_t) now.lo);
}

ER
s4_status_publish (const s4_sensor_cache_t * p_status)
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
        published_status = *p_status;
        result           = tk_unl_mtx(status_lock);
    }
    return result;
}

ER
s4_sensor_get_record (s4_sensor_cache_t * out)
{
    if (!out)
    {
        return E_PAR;
    }
    if (status_lock <= E_OK)
    {
        return E_OBJ;
    }
    ER result = tk_loc_mtx(status_lock, S4_LOCK_WAIT_MS);
    if (result < E_OK)
    {
        return result;
    }
    s4_sensor_cache_t copy = published_status;
    result                 = tk_unl_mtx(status_lock);
    if (result >= E_OK)
    {
        *out = copy;
    }
    return result;
}
ER
s4_sensor_get_status (s4_sensor_status_t * out)
{
    if (!out)
    {
        return E_PAR;
    }
    s4_sensor_cache_t copy;
    ER                result = s4_sensor_get_record(&copy);
    if (result < E_OK)
    {
        return result;
    }
    uint64_t now = s4_kernel_now_us(NULL);
    if (now == UINT64_MAX)
    {
        return E_SYS;
    }
    s4_data_snapshot(&copy, now, out);
    return result;
}

ER
subsystem4_init (void)
{
    T_CMTX mutex = {0};

    if (status_lock > E_OK)
    {
        return E_OBJ; /* Lifecycle is init once, then start once. */
    }
    s4_data_reset(&published_status);
    mutex.mtxatr = TA_INHERIT;
    status_lock  = tk_cre_mtx(&mutex);
    if (status_lock <= E_OK)
    {
        return status_lock;
    }
    return E_OK;
}

static void
sensor_cyclic (void * context)
{
    (void) context;
    if (tid_s4_sensor > E_OK)
    {
        (void) tk_wup_tsk(tid_s4_sensor);
    }
}

ER
subsystem4_start (void)
{
    if (status_lock <= E_OK)
    {
        return E_OBJ;
    }
    if ((tid_s4_sensor > E_OK) || (tid_s4_monitor > E_OK))
    {
        return E_OBJ;
    }
    T_CCYC config = {0};
    config.cycatr = TA_HLNG | TA_STA;
    config.cychdr = sensor_cyclic;
    config.cyctim = S4_POLL_MS;
    config.cycphs = S4_POLL_MS;
#if TK_SUPPORT_SMP
    config.cycatr |= TA_ASSPRC;
    config.assprc = TP_PRC2;
#endif
    ID cycle = tk_cre_cyc(&config);
    if (cycle < E_OK)
    {
        return cycle;
    }

    /* app_start_task ignores affinity in SMP=0. In SMP=1 the only I2C1
     * owner is pinned to physical core 1; logging may run on either core. */
    tid_s4_sensor = app_start_task(s4_sensor_task, S4_PRI_SENSOR, TP_PRC2,
                                   "s4_sensor_task");
    if (tid_s4_sensor <= E_OK)
    {
        (void) tk_del_cyc(cycle);
        return tid_s4_sensor;
    }
    tid_s4_monitor =
        app_start_task(monitor_task, S4_PRI_MONITOR, 0, "s4_monitor_task");
    return (tid_s4_monitor <= E_OK) ? tid_s4_monitor : E_OK;
}

/*** end of file ***/
