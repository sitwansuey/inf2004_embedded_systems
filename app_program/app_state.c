#include "app_state.h"

static ID state_mutex;
static app_shared_state_t shared;

EXPORT ER app_state_init(void)
{
    T_CMTX cfg = {0};
    cfg.mtxatr = TA_INHERIT;
    shared = (app_shared_state_t){0};
    state_mutex = tk_cre_mtx(&cfg);
    return state_mutex < E_OK ? state_mutex : E_OK;
}

EXPORT void app_state_collect(app_shared_state_t *next)
{
    /* Preserve the latest reading on a temporarily unavailable S5 mutex;
     * availability prevents the FSM from acting on that retained reading. */
    s5_status_t sensor;
    next->s5_available = s5_get_status(&sensor) >= E_OK;
    if (next->s5_available) next->s5 = sensor;
    SYSTIM now;
    tk_get_tim(&now); /* after the cross-core sensor copy */
    next->time_ms = now.lo;
    ++next->sequence;
}

EXPORT ER app_state_publish(const app_shared_state_t *next)
{
    if (!next) return E_PAR;
    ER er = tk_loc_mtx(state_mutex, TMO_FEVR);
    if (er < E_OK) return er;
    shared = *next;
    return tk_unl_mtx(state_mutex);
}

EXPORT ER app_get_state(app_shared_state_t *copy)
{
    if (!copy) return E_PAR;
    ER er = tk_loc_mtx(state_mutex, TMO_FEVR);
    if (er < E_OK) return er;
    *copy = shared;
    return tk_unl_mtx(state_mutex);
}
