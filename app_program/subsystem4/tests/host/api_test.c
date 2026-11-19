/* SPDX-License-Identifier: Apache-2.0 */
#include "shim/subsystem4/subsystem4.h"

#include <stdio.h>
#include <stdlib.h>

#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

static int fail_mutex_create;
static ER lock_result;
static ER unlock_result;
static unsigned int tasks_started;
static uint64_t clock_ms;
static int64_t wall_clock_ms = -INT64_C(473385600000);
static unsigned int wall_clock_reads;
static bool clock_failed;
static bool cycle_create_failed;
static bool sensor_start_failed;
static unsigned int cycles_deleted;
static unsigned int sensor_wakes;
static void (*sensor_release)(void *);

ER tk_get_tim(SYSTIM *p_time)
{
    ++wall_clock_reads;
    p_time->hi = (int32_t)((uint64_t)wall_clock_ms >> 32);
    p_time->lo = (UW)wall_clock_ms;
    return E_OK;
}

ER tk_get_otm(SYSTIM *p_time)
{
    if (clock_failed) return E_SYS;
    p_time->hi = (int32_t)(clock_ms >> 32);
    p_time->lo = (UW)clock_ms;
    return E_OK;
}
ID tk_cre_mtx(const T_CMTX *p_config)
{
    REQUIRE(p_config->mtxatr == TA_INHERIT);
    return fail_mutex_create ? E_NOMEM : 10;
}
ID tk_cre_cyc(const T_CCYC *p_config)
{
    REQUIRE(p_config->cycatr == (TA_HLNG | TA_STA));
    REQUIRE(p_config->cyctim == 10 && p_config->cycphs == 10);
    REQUIRE(p_config->cychdr != NULL);
    if (cycle_create_failed) return E_NOMEM;
    sensor_release = p_config->cychdr;
    return 13;
}
ER tk_del_cyc(ID id)
{
    REQUIRE(id == 13);
    ++cycles_deleted;
    sensor_release = NULL;
    return E_OK;
}
ER tk_wup_tsk(ID id)
{
    REQUIRE(id == tid_s4_sensor && id > 0);
    ++sensor_wakes;
    return E_OK;
}
ER tk_loc_mtx(ID id, INT timeout)
{
    REQUIRE(id == 10 && timeout == S4_LOCK_WAIT_MS);
    return lock_result;
}
ER tk_unl_mtx(ID id)
{
    REQUIRE(id == 10);
    return unlock_result;
}
void s4_sensor_task(INT code, void *p_context)
{
    (void)code;
    (void)p_context;
}
void monitor_task(INT code, void *p_context)
{
    (void)code;
    (void)p_context;
}
ID app_start_task(FP entry, PRI pri, UW affinity, const char *p_name)
{
    (void)p_name;
    if (tasks_started == 0)
    {
        REQUIRE(entry == s4_sensor_task && pri == 18 && affinity == TP_PRC2);
        if (sensor_start_failed) return E_NOMEM;
    }
    else
    {
        REQUIRE(entry == monitor_task && pri == 19 && affinity == 0);
    }
    tasks_started++;
    return (ID)(20u + tasks_started);
}

int main(void)
{
    s4_sensor_cache_t state;
    s4_sensor_status_t output;

    REQUIRE(subsystem4_start() == E_OBJ);
    REQUIRE(s4_sensor_get_status(NULL) == E_PAR);
    REQUIRE(s4_sensor_get_status(&output) == E_OBJ);
    fail_mutex_create = 1; /* the status mutex is S4's only kernel object */
    REQUIRE(subsystem4_init() == E_NOMEM);
    REQUIRE(subsystem4_start() == E_OBJ);
    fail_mutex_create = 0;
    REQUIRE(subsystem4_init() == E_OK);
    REQUIRE(subsystem4_init() == E_OBJ);
    REQUIRE(s4_sensor_get_status(&output) == E_OK && !output.configured);
    puts("[TEST] S4-U19 PASS  two-phase lifecycle, mutex failure retry and argument checks");

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_cache_set_count(&state,S4_COUNT_HEARTBEAT,42);
    lsm303_frame_t frame={0};
    frame.accel.fresh=true;frame.accel.valid=true;frame.accel.read_time_us=20000;
    frame.accel.raw[2]=1000;
    s4_data_update(&state,&frame,LSM303_OK);
    clock_ms = 30;
    REQUIRE(s4_kernel_now_us(NULL) == UINT64_C(30000));
    wall_clock_ms = INT64_C(123456789000);
    REQUIRE(s4_kernel_now_us(NULL) == UINT64_C(30000));
    REQUIRE(wall_clock_reads == 0);
    puts("[TEST] S4-U22 PASS  unset/changed wall clock cannot alter shared operating time");
    REQUIRE(s4_status_publish(&state) == E_OK);
    REQUIRE(s4_status_publish(&state) == E_OK);
    REQUIRE(s4_sensor_get_status(&output) == E_OK);
    REQUIRE(output.task_heartbeat == 42 && output.accel_usable);
    REQUIRE(output.accel_age_ms == 10 && output.legacy_lines_received == 0); /* IF-03 retired */
    s4_sensor_cache_t record;
    REQUIRE(s4_sensor_get_record(&record)==E_OK);
    s4_sensor_status_t decoded;
    s4_data_snapshot(&record,30000,&decoded);
    REQUIRE(decoded.accel_usable && decoded.task_heartbeat==42 && sizeof(record)==71);
    lock_result = E_TMOUT;
    output.task_heartbeat = 99;
    REQUIRE(s4_sensor_get_status(&output) == E_TMOUT && output.task_heartbeat == 99);
    for(unsigned n=0;n<S4_CACHE_BYTES;++n)record.bytes[n]=0xa5;
    REQUIRE(s4_sensor_get_record(&record)==E_TMOUT);
    for(unsigned n=0;n<S4_CACHE_BYTES;++n)REQUIRE(record.bytes[n]==0xa5);
    s4_cache_set_count(&state,S4_COUNT_HEARTBEAT,123);
    REQUIRE(s4_status_publish(&state) == E_TMOUT);
    lock_result = E_OK;
    unlock_result=E_SYS;
    REQUIRE(s4_sensor_get_record(&record)==E_SYS);
    for(unsigned n=0;n<S4_CACHE_BYTES;++n)REQUIRE(record.bytes[n]==0xa5);
    unlock_result=E_OK;
    REQUIRE(s4_sensor_get_status(&output) == E_OK && output.task_heartbeat == 42);
    clock_ms = (UINT64_C(1) << 32) + 123;
    REQUIRE(s4_kernel_now_us(NULL) == clock_ms * 1000u);
    REQUIRE(s4_sensor_get_status(&output) == E_OK && !output.accel_usable);
    puts("[TEST] S4-U20 PASS  protected snapshot, timeout propagation and kernel-clock wrap");
    clock_failed = true;
    output.task_heartbeat = 99;
    REQUIRE(s4_kernel_now_us(NULL) == UINT64_MAX);
    REQUIRE(s4_sensor_get_status(&output) == E_SYS && output.task_heartbeat == 99);
    clock_failed = false;
    puts("[TEST] S4-U27 PASS  clock errors cannot publish a fresh sample or replace caller output");

    cycle_create_failed = true;
    REQUIRE(subsystem4_start() == E_NOMEM && tasks_started == 0);
    cycle_create_failed = false;
    sensor_start_failed = true;
    REQUIRE(subsystem4_start() == E_NOMEM && cycles_deleted == 1);
    sensor_start_failed = false;
    REQUIRE(subsystem4_start() == E_OK && tasks_started == 2);
    sensor_release(NULL);
    REQUIRE(sensor_wakes == 1);
    REQUIRE(subsystem4_start() == E_OBJ);
    puts("[TEST] S4-U21 PASS  task priorities/affinity and duplicate-start rejection");
    return EXIT_SUCCESS;
}
