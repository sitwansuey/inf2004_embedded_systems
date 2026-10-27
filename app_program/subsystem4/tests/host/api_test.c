/* SPDX-License-Identifier: Apache-2.0 */
#include "shim/subsystem4/subsystem4.h"

#include <stdio.h>
#include <stdlib.h>

#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

static int fail_flag_create;
static int fail_buffer_create;
static ER lock_result;
static unsigned int deleted_mutexes;
static unsigned int deleted_flags;
static unsigned int tasks_started;
static uint64_t clock_ms;
static int64_t wall_clock_ms = -INT64_C(473385600000);
static unsigned int wall_clock_reads;
static bool clock_failed;

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
    return 10;
}
ID tk_cre_flg(const T_CFLG *p_config)
{
    REQUIRE(p_config->iflgptn == 0);
    return fail_flag_create ? E_NOMEM : 11;
}
ID tk_cre_mbf(const T_CMBF *p_config)
{
    REQUIRE(p_config->maxmsz == S4_SUMMARY_MAX);
    return fail_buffer_create ? E_NOMEM : 12;
}
ER tk_loc_mtx(ID id, INT timeout)
{
    REQUIRE(id == 10 && timeout == S4_LOCK_WAIT_MS);
    return lock_result;
}
ER tk_unl_mtx(ID id)
{
    REQUIRE(id == 10);
    return E_OK;
}
ER tk_del_mtx(ID id)
{
    REQUIRE(id == 10);
    deleted_mutexes++;
    return E_OK;
}
ER tk_del_flg(ID id)
{
    REQUIRE(id == 11);
    deleted_flags++;
    return E_OK;
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
    s4_sensor_status_t state;
    s4_sensor_status_t output;

    REQUIRE(subsystem4_start() == E_OBJ);
    REQUIRE(s4_sensor_get_status(NULL) == E_PAR);
    REQUIRE(s4_sensor_get_status(&output) == E_OBJ);
    fail_flag_create = 1;
    REQUIRE(subsystem4_init() == E_NOMEM && deleted_mutexes == 1);
    fail_flag_create = 0;
    fail_buffer_create = 1;
    REQUIRE(subsystem4_init() == E_NOMEM && deleted_mutexes == 2 && deleted_flags == 1);
    fail_buffer_create = 0;
    REQUIRE(subsystem4_init() == E_OK);
    REQUIRE(subsystem4_init() == E_OBJ);
    REQUIRE(s4_sensor_get_status(&output) == E_OK && !output.configured);
    puts("[TEST] S4-U19 PASS  two-phase lifecycle, object cleanup and argument checks");

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    state.task_heartbeat = 42;
    state.latest.accel.fresh = true;
    state.latest.accel.valid = true;
    state.latest.accel.read_time_us = 20000;
    clock_ms = 30;
    REQUIRE(s4_kernel_now_us(NULL) == UINT64_C(30000));
    wall_clock_ms = INT64_C(123456789000);
    REQUIRE(s4_kernel_now_us(NULL) == UINT64_C(30000));
    REQUIRE(wall_clock_reads == 0);
    puts("[TEST] S4-U22 PASS  unset/changed wall clock cannot alter shared operating time");
    REQUIRE(s4_status_publish(&state) == E_OK);
    REQUIRE(s4_note_legacy_line() == E_OK);
    REQUIRE(s4_status_publish(&state) == E_OK);
    REQUIRE(s4_sensor_get_status(&output) == E_OK);
    REQUIRE(output.task_heartbeat == 42 && output.accel_usable);
    REQUIRE(output.accel_age_ms == 10 && output.legacy_lines_received == 1);
    lock_result = E_TMOUT;
    output.task_heartbeat = 99;
    REQUIRE(s4_sensor_get_status(&output) == E_TMOUT && output.task_heartbeat == 99);
    state.task_heartbeat = 123;
    REQUIRE(s4_status_publish(&state) == E_TMOUT);
    lock_result = E_OK;
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

    REQUIRE(subsystem4_start() == E_OK && tasks_started == 2);
    REQUIRE(subsystem4_start() == E_OBJ);
    puts("[TEST] S4-U21 PASS  task priorities/affinity and duplicate-start rejection");
    return EXIT_SUCCESS;
}
