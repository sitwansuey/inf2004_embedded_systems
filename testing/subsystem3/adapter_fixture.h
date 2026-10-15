/* Executes actual Buddy 3 adapter code with deterministic RTOS/GPIO fakes.
 * Does not simulate a robot or establish physical hardware correctness. */
#include "host_support.h"
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem3/subsystem3.c"
#include "../../app_program/subsystem3/task_consumer.c"
#include "../../app_program/subsystem3/drv_ir_adc.c"

static unsigned int checks;
static unsigned int failures;
static unsigned int groups;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    ++failures; (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
} } while (0)
#define RUN(id, function) do { unsigned int before = failures; ++groups; \
    function(); (void)printf("%s %s\n", id, failures == before ? "PASS" : "FAIL"); \
} while (0)

static uint32_t fake_now;
static ID caller;
static unsigned int creates;
static unsigned int fail_create;
static unsigned int deletes;
static bool fail_start;
static bool fail_time;
static bool busy_mutex;
static bool locked;
static unsigned int gpio_reads;
static unsigned int gpio_configs;
static UINT gpio[30];
static ER gpio_config_error;
static bool race_override;
static bool auto_renew;
static s3_event_record_t queue[4];
static unsigned int queue_count;
static s3_adc_request_t adc_request;
static s3_adc_reply_t adc_reply;
static bool have_request;
static bool have_reply;
static ER adc_open_result;
static ER adc_read_result;
static UW adc_values[3];
static SZ adc_read_size;
static uint32_t adc_read_ms;
static unsigned int adc_opens;
static unsigned int adc_reads;
static void (*adc_read_hook)(void);
static INT malformed_request;
static INT malformed_reply;

static ID create_object(void)
{
    ++creates;
    return (creates == fail_create) ? E_SYS : (ID)creates;
}
ID tk_cre_mbx(const T_CMBX *p_cfg) { (void)p_cfg; return create_object(); }
ID tk_cre_mtx(const T_CMTX *p_cfg)
{
    CHECK(p_cfg->mtxatr == TA_INHERIT);
    return create_object();
}
ID tk_cre_mbf(const T_CMBF *p_cfg)
{
    if (p_cfg->maxmsz == (INT)sizeof(s3_event_record_t))
    {
        CHECK(p_cfg->bufsz >= (INT)(4U * (sizeof(s3_event_record_t) + sizeof(UW))));
    }
    else
    {
        CHECK(p_cfg->maxmsz == (INT)sizeof(s3_adc_request_t) ||
              p_cfg->maxmsz == (INT)sizeof(s3_adc_reply_t));
        CHECK(p_cfg->bufsz == p_cfg->maxmsz + (INT)sizeof(UW));
    }
    return create_object();
}
ID tk_cre_tsk(const T_CTSK *p_cfg)
{
    CHECK((p_cfg->itskpri == 16 || p_cfg->itskpri == 17) && p_cfg->stksz == 2048U);
#if TK_SUPPORT_SMP
    CHECK(p_cfg->assprc == TP_PRC1);
#endif
    return (fail_create == creates + 1U) ? E_SYS :
           ((p_cfg->itskpri == 17) ? 43 : 42);
}
ER tk_del_mbf(ID object) { CHECK(object > 0); ++deletes; return E_OK; }
ER tk_del_mbx(ID object) { CHECK(object > 0); ++deletes; return E_OK; }
ER tk_del_mtx(ID object) { CHECK(object > 0); ++deletes; return E_OK; }
ER tk_del_tsk(ID object) { CHECK(object == 42 || object == 43); ++deletes; return E_OK; }
ER tk_loc_mtx(ID object, TMO timeout)
{
    CHECK(object > 0 && timeout == TMO_POL);
    if (busy_mutex) { return E_TMOUT; }
    CHECK(!locked); locked = true; return E_OK;
}
ER tk_unl_mtx(ID object)
{
    CHECK(object > 0 && locked); locked = false; return E_OK;
}
ER tk_sta_tsk(ID task, INT start)
{
    CHECK((task == 42 || task == 43) && start == 0);
    return fail_start ? E_SYS : E_OK;
}
ER tk_dly_tsk(UW time_ms) { fake_now += time_ms; return E_OK; }
void tk_ext_tsk(void) { }
ID tk_get_tid(void) { return caller; }
ER tk_get_tim(SYSTIM *p_time)
{
    if (fail_time) { return E_SYS; }
    p_time->hi = 0; p_time->lo = fake_now; return E_OK;
}
ER tk_snd_mbf(ID object, const void *p_data, INT length, TMO timeout)
{
    if (object <= 0) { return E_OBJ; }
    if (object == h_requests && h_requests > 0)
    {
        CHECK(length == (INT)sizeof(adc_request) && timeout == TMO_POL && !locked);
        if (have_request) { return E_TMOUT; }
        (void)memcpy(&adc_request, p_data, sizeof(adc_request));
        have_request = true; return E_OK;
    }
    if (object == h_replies && h_replies > 0)
    {
        CHECK(length == (INT)sizeof(adc_reply) && timeout == TMO_POL && !locked);
        if (have_reply) { return E_TMOUT; }
        (void)memcpy(&adc_reply, p_data, sizeof(adc_reply));
        have_reply = true; return E_OK;
    }
    CHECK(object > 0 && length == (INT)sizeof(s3_event_record_t));
    CHECK(timeout == TMO_POL && locked);
    if (queue_count == 4U) { return E_TMOUT; }
    (void)memcpy(&queue[queue_count++], p_data, (size_t)length);
    return E_OK;
}
INT tk_rcv_mbf(ID object, void *p_data, TMO timeout)
{
    if (object <= 0) { return E_OBJ; }
    if (object == h_requests && h_requests > 0)
    {
        CHECK((timeout == TMO_POL || timeout == TMO_FEVR) && !locked);
        if (!have_request) { return E_TMOUT; }
        (void)memcpy(p_data, &adc_request, sizeof(adc_request));
        have_request = false;
        return malformed_request ? malformed_request : (INT)sizeof(adc_request);
    }
    if (object == h_replies && h_replies > 0)
    {
        CHECK(timeout == TMO_POL && !locked);
        if (!have_reply) { return E_TMOUT; }
        (void)memcpy(p_data, &adc_reply, sizeof(adc_reply));
        have_reply = false;
        return malformed_reply ? malformed_reply : (INT)sizeof(adc_reply);
    }
    CHECK(object > 0 && timeout == TMO_POL && locked);
    if (queue_count == 0U) { return E_TMOUT; }
    (void)memcpy(p_data, &queue[0], sizeof(queue[0]));
    --queue_count;
    (void)memmove(queue, queue + 1, queue_count * sizeof(queue[0]));
    return (INT)sizeof(s3_event_record_t);
}
ID tk_opn_dev(const UB *p_name, UINT mode)
{
    CHECK(caller == 43 && !locked);
    CHECK(strcmp((const char *)p_name, "adca") == 0 && mode == (TD_READ | TD_EXCL));
    ++adc_opens;
    return adc_open_result;
}
ER tk_srea_dev(ID descriptor, W channel, void *p_buffer, SZ size, SZ *p_actual)
{
    CHECK(caller == 43 && !locked && descriptor == 71);
    CHECK(channel >= 0 && channel <= 2 && size == 1);
    uint32_t read_started = fake_now;
    ++adc_reads;
    if (adc_read_hook != NULL)
    {
        fake_now += S3_ADC_DEADLINE_MS;
        adc_read_hook();
    }
    fake_now = read_started + adc_read_ms;
    *(UW *)p_buffer = adc_values[(unsigned int)channel];
    *p_actual = adc_read_size;
    return adc_read_result;
}
ER gpio_set_pin(UINT pin, UINT mode)
{
    CHECK(pin < 30U && mode == GPIO_MODE_IN);
    ++gpio_configs;
    return gpio_config_error;
}
UINT gpio_get_val(UINT pin)
{
    CHECK(pin < 30U); ++gpio_reads;
    if (race_override)
    {
        race_override = false;
        caller = 7;
        CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
        caller = 42;
    }
    return gpio[pin];
}
static void reset(void)
{
    mbx_samples = 0; mtx_stats = 0; h_events = 0; h_controller = 0; h_task = 0;
    started = false; acquisition_attempted = false; acquisition_open_error = E_OK;
    applied_generation = 0U;
    h_requests = 0; h_replies = 0; h_worker = 0; h_client = 0; h_device = 0;
    worker_failed = false; have_request = false; have_reply = false;
    adc_request = (s3_adc_request_t){0}; adc_reply = (s3_adc_reply_t){0};
    adc_open_result = 71; adc_read_result = E_OK; adc_read_size = 1;
    adc_read_ms = 0U; adc_opens = 0U; adc_reads = 0U; adc_read_hook = NULL;
    malformed_request = 0; malformed_reply = 0;
    adc_values[0] = 0U; adc_values[1] = 4095U; adc_values[2] = 0U;
    (void)memset(&configuration, 0, sizeof(configuration));
    (void)memset(&status, 0, sizeof(status));
    (void)memset(&core, 0, sizeof(core));
    (void)memset(&ir, 0, sizeof(ir));
    (void)memset(&control, 0, sizeof(control));
    (void)memset(gpio, 0, sizeof(gpio));
    creates = 0U; fail_create = 0U; deletes = 0U; fake_now = 0U; caller = 7;
    fail_start = false; fail_time = false; busy_mutex = false; locked = false;
    gpio_reads = 0U; gpio_configs = 0U; gpio_config_error = E_OK;
    race_override = false; queue_count = 0U; auto_renew = true;
}
