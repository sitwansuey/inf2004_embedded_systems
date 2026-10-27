#include "../robot_config.h"
#if APP_S5_ENABLED
#include "subsystem5.h"
#include "s5_hardware.h"
#include "s5_scan.h"

#if !BOARD_UART0_FREE
#error "S5 uses GP0/GP1: build with CONSOLE=usb_cdc CRASH_REPORT=0"
#endif

static ID status_mutex;
static ID controller;
ER s5_controller_bind(void) {
    if(status_mutex<=0)return E_OBJ;
    ER er=tk_loc_mtx(status_mutex,S5_LOCK_WAIT_MS);if(er<0)return er;
    if(controller && controller!=tk_get_tid())er=E_OACV;else controller=tk_get_tid();
    ER unlocked=tk_unl_mtx(status_mutex);return unlocked<E_OK?unlocked:er;
}
static s5_status_t status = {.state = S5_CENTERING};
static BOOL recenter;
static BOOL turn_requested;
static BOOL navigation_requested;
static s5_state_t navigation_state;
static INT navigation_look;

EXPORT ER s5_navigation_set(UW scan_id, s5_state_t state, INT look_deg)
{
    if(controller<=0 || tk_get_tid()!=controller)return E_OACV;
    if ((state != S5_APPROACH && state != S5_OVERCOME &&
         state != S5_REORG && state != S5_ROTATING && state != S5_MONITOR) ||
        look_deg < -90 || look_deg > 90)
        return E_PAR;
    if (status_mutex <= 0) return E_OBJ;
    ER er = tk_loc_mtx(status_mutex, S5_LOCK_WAIT_MS);
    if (er < E_OK) return er;
    if (status.scan_id != scan_id || (status.state == S5_FAULT && state != S5_MONITOR))
        er = E_OBJ;
    else {
        navigation_state = state;
        navigation_look = look_deg;
        navigation_requested = TRUE;
        /* Stop consumers from reusing a sample from the previous mode. */
        status.state = state;
        status.valid = FALSE;
    }
    ER unlocked = tk_unl_mtx(status_mutex);
    return unlocked < E_OK ? unlocked : er;
}

EXPORT ER s5_get_status(s5_status_t *copy)
{
    if (!copy)
        return E_PAR;
    if (status_mutex <= 0) return E_OBJ;
    ER er = tk_loc_mtx(status_mutex, S5_LOCK_WAIT_MS);
    if (er < E_OK)
        return er;
    *copy = status;
    return tk_unl_mtx(status_mutex);
}

EXPORT ER s5_avoidance_begin(UW scan_id)
{
    if(controller<=0 || tk_get_tid()!=controller)return E_OACV;
    if (status_mutex <= 0) return E_OBJ;
    ER er = tk_loc_mtx(status_mutex, S5_LOCK_WAIT_MS);
    if (er < E_OK)
        return er;
    if (status.state != S5_READY || !status.valid || status.scan_id != scan_id)
        er = E_OBJ;
    else
    {
        turn_requested = TRUE;
        status.state = S5_ROTATING;
        status.valid = FALSE;
    }
    ER unlocked = tk_unl_mtx(status_mutex);
    return unlocked < E_OK ? unlocked : er;
}

EXPORT ER s5_avoidance_complete(void)
{
    if(controller<=0 || tk_get_tid()!=controller)return E_OACV;
    if (status_mutex <= 0) return E_OBJ;
    ER er = tk_loc_mtx(status_mutex, S5_LOCK_WAIT_MS);
    if (er < E_OK)
        return er;
    if (status.state != S5_READY && status.state != S5_ROTATING)
        er = E_OBJ;
    else
        recenter = TRUE;
    ER unlocked = tk_unl_mtx(status_mutex);
    return unlocked < E_OK ? unlocked : er;
}

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    if (s5_hw_timer_init() < E_OK) {
        tk_loc_mtx(status_mutex,TMO_FEVR); status.state=S5_FAULT;
        tk_unl_mtx(status_mutex); tk_ext_tsk(); return;
    }
    s5_hw_io_init();
    tm_printf((UB *)"[S5] status copies: bounded inheritance wait\n");
    tm_printf((UB *)"[S5] wiring: TRIG=GP%u ECHO=GP%u\n",
              BOARD_ULTRASONIC_TRIG_GPIO, BOARD_ULTRASONIC_ECHO_GPIO);
    tm_printf((UB *)"[S5] startup: centring servo at 0 deg; waiting %u ms\n",
              S5_STARTUP_CENTER_MS);
    tk_dly_tsk(S5_STARTUP_CENTER_MS);
    tm_printf((UB *)"[S5] startup: centre settling complete; checking forward\n");
    s5_status_t next = {.state = S5_MONITOR};
    s5_scan_context_t scan = {0};
    for (;;)
    {
        s5_hw_sample(&next);
        UW distance = next.distance_mm;
        BOOL valid = next.valid;
        SYSTIM now;
        tk_get_otm(&now);
        next.valid = valid;
        next.distance_mm = distance;
        next.sample_angle_deg = next.angle_deg;
        next.time_ms = now.lo;
        INT target = s5_scan_step(&next, &scan);
        INT travelled;
        tk_loc_mtx(status_mutex, TMO_FEVR);
        if (turn_requested)
        {
            turn_requested = FALSE;
            next.state = S5_ROTATING;
            next.valid = FALSE;
            target = 0;
        }
        if (recenter)
        {
            recenter = FALSE;
            target = 0;
            next.state = S5_GUIDING;
            next.valid = FALSE;
        }
        if (navigation_requested)
        {
            navigation_requested = FALSE;
            next.state = navigation_state;
            target = navigation_look;
            next.valid = FALSE;
        }
        travelled = target - next.angle_deg;
        if (travelled < 0)
        {
            travelled = -travelled;
        }
        /* Never publish a clear reading while the servo is moving. */
        if (target != next.angle_deg)
        {
            next.valid = FALSE;
            next.angle_deg = target;
        }
        status = next;
        tk_unl_mtx(status_mutex);
        s5_hw_servo(target);
        UW settle_ms = (UW)travelled * S5_SERVO_MS_PER_DEG;
        if (settle_ms < S5_SETTLE_MS)
            settle_ms = S5_SETTLE_MS;
        tk_dly_tsk(next.valid ? S5_SAMPLE_MS : settle_ms);
    }
}
EXPORT ER subsystem5_init(void)
{
    if (status_mutex > 0) return E_OBJ;
    T_CMTX cfg = {0};
    cfg.mtxatr = TA_INHERIT;
    status_mutex = tk_cre_mtx(&cfg);
    return status_mutex < E_OK ? status_mutex : E_OK;
}
EXPORT ER subsystem5_start(void)
{
    if (status_mutex <= 0) return E_OBJ;
    ID tid = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC2, "obstacle scan");
    return tid < E_OK ? tid : E_OK;
}

#endif
