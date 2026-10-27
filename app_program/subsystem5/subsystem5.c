#include "subsystem5.h"
#include "s5_hardware.h"
#include "s5_scan.h"
#include "s5_data.h"

#if !BOARD_UART0_FREE
#error "S5 uses GP0/GP1: build with CONSOLE=usb_cdc CRASH_REPORT=0"
#endif

static ID status_mutex;
static uint8_t status_data[S5_DATA_SIZE] = {
    S5_CENTERING, 0, 90, 90, 90, 90, 90, 90
};

/* Caller holds status_mutex. Invalid internal data fails closed. */
static void status_load(s5_status_t *value)
{
    if (!s5_data_decode(status_data, sizeof(status_data), value))
        *value = (s5_status_t){.state = S5_FAULT};
}
static void status_store(const s5_status_t *value)
{
    if (!s5_data_encode(value, status_data, sizeof(status_data))) {
        s5_status_t fault = {.state = S5_FAULT};
        (void)s5_data_encode(&fault, status_data, sizeof(status_data));
    }
}

EXPORT ER s5_status_encode(uint8_t *data, size_t capacity)
{
    if (!data || capacity < S5_DATA_SIZE) return E_PAR;
    s5_status_t copy;
    ER er = s5_get_status(&copy);
    if (er < E_OK) return er;
    return s5_data_encode(&copy, data, capacity) ? E_OK : E_PAR;
}
static BOOL recenter;
static BOOL turn_requested;
static BOOL navigation_requested;
static s5_state_t navigation_state;
static INT navigation_look;

EXPORT ER s5_navigation_set(UW scan_id, s5_state_t state, INT look_deg)
{
    s5_status_t status;
    if ((state != S5_APPROACH && state != S5_OVERCOME &&
         state != S5_REORG && state != S5_ROTATING && state != S5_MONITOR) ||
        look_deg < -90 || look_deg > 90)
        return E_PAR;
    ER er = tk_loc_mtx(status_mutex, TMO_POL);
    if (er < E_OK) return er;
    status_load(&status);
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
    status_store(&status);
    tk_unl_mtx(status_mutex);
    return er;
}

EXPORT ER s5_get_status(s5_status_t *copy)
{
    s5_status_t status;
    if (!copy)
        return E_PAR;
    ER er = tk_loc_mtx(status_mutex, TMO_POL);
    if (er < E_OK)
        return er;
    status_load(&status);
    *copy = status;
    tk_unl_mtx(status_mutex);
    return E_OK;
}

EXPORT ER s5_avoidance_begin(UW scan_id)
{
    s5_status_t status;
    ER er = tk_loc_mtx(status_mutex, TMO_POL);
    if (er < E_OK)
        return er;
    status_load(&status);
    if (status.state != S5_READY || !status.valid || status.scan_id != scan_id)
        er = E_OBJ;
    else
    {
        turn_requested = TRUE;
        status.state = S5_ROTATING;
        status.valid = FALSE;
    }
    status_store(&status);
    tk_unl_mtx(status_mutex);
    return er;
}

EXPORT ER s5_avoidance_complete(void)
{
    s5_status_t status;
    ER er = tk_loc_mtx(status_mutex, TMO_POL);
    if (er < E_OK)
        return er;
    status_load(&status);
    if (status.state != S5_READY && status.state != S5_ROTATING)
        er = E_OBJ;
    else
        recenter = TRUE;
    status_store(&status);
    tk_unl_mtx(status_mutex);
    return er;
}

#if APP_TEST
/* The S5 test task is the only owner of sensor/servo hardware in test builds. */
EXPORT ER s5_test_position(INT angle, BOOL take_sample, s5_status_t *copy)
{
    if (angle < -90 || angle > 90 || !copy)
        return E_PAR;
    s5_hw_servo(angle);
    tk_dly_tsk(1000);
    *copy = (s5_status_t){.state = S5_CENTERING, .angle_deg = angle};
    if (take_sample)
    {
        s5_hw_sample(copy);
        copy->valid = copy->echo_result == S5_ECHO_OK;
    }
    SYSTIM now;
    tk_get_tim(&now);
    copy->time_ms = now.lo;
    copy->sample_angle_deg = angle;
    ER er = tk_loc_mtx(status_mutex, TMO_FEVR);
    if (er < E_OK)
        return er;
    status_store(copy);
    tk_unl_mtx(status_mutex);
    return E_OK;
}

static void sensor_test_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    s5_hw_io_init();
#if APP_S5_MASTER_TEST
    s5_master_test();
#elif APP_ACQUISITION_TEST
    aqcuisition_test();
#elif APP_ACQUISITION_LEFT_TEST
    acquisition_left_test();
#else
    subsystem5_test();
#endif
    tk_ext_tsk();
}
#endif

static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    s5_hw_io_init();
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
        BOOL valid = next.echo_result == S5_ECHO_OK;
        SYSTIM now;
        tk_get_tim(&now);
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
        status_store(&next);
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
    ER er = s5_hw_timer_init();
    if (er < E_OK)
        return er;
    T_CMTX cfg = {0};
    cfg.mtxatr = TA_INHERIT;
    status_mutex = tk_cre_mtx(&cfg);
    return status_mutex < E_OK ? status_mutex : E_OK;
}
EXPORT ER subsystem5_start(void)
{
#if APP_TEST
    ID tid = app_start_task((FP)sensor_test_task, S5_PRI_MAIN, TP_PRC2, "sensor sweep test");
#else
    ID tid = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC2, "obstacle scan");
#endif
    return tid < E_OK ? tid : E_OK;
}
