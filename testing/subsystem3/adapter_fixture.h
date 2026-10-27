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
static unsigned int sample_lock_calls,delay_lock_call;
static UW state_lock_delay_ms;
static void (*state_lock_hook)(void);
static T_CCYC fake_cycle;
static bool fake_cycle_active,fail_cycle;
static uint32_t cycle_due;
static unsigned int wake_credits,sleeps,stop_after_sleeps,task_exits;
static bool model_delay_guard;
static unsigned int delay_calls,stop_after_delays;
static unsigned int gpio_reads;
static unsigned int gpio_configs;
static UINT gpio[30];
static ER gpio_config_error;
static bool race_override;
static bool auto_renew;
static s3_event_record_t queue[4];
static unsigned int queue_count;
#if BOARD_IR_BARCODE_CAPTURE
static s3_capture_record_t capture_queue[S3_CAPTURE_CAPACITY];
static unsigned int capture_count;
static bool malformed_capture;
#endif
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
static bool adc_auto_worker;
static uint32_t adc_wait_delay_ms;
static bool adc_fail_clock_after_wait;
static unsigned int adc_reply_waits;
static void (*adc_wait_hook)(void);

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
#if BOARD_IR_BARCODE_CAPTURE
    if (p_cfg->maxmsz == (INT)sizeof(s3_capture_record_t) &&
        p_cfg->bufsz == (INT)(S3_CAPTURE_CAPACITY * (sizeof(s3_capture_record_t) + sizeof(UW))))
        return create_object();
#endif
    if (p_cfg->maxmsz == (INT)S3_EVENT_BYTES)
    {
        CHECK(p_cfg->bufsz >= (INT)(4U * (16U + sizeof(UW))));
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
    CHECK(object > 0 && (timeout == TMO_POL || timeout == S3_STATE_LOCK_WAIT_MS));
    if(timeout==S3_STATE_LOCK_WAIT_MS) {
        ++sample_lock_calls;
        if(sample_lock_calls==delay_lock_call) {
            fake_now+=state_lock_delay_ms;
            if(state_lock_hook) state_lock_hook();
        }
    }
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
ER tk_dly_tsk(UW time_ms) {
    ++delay_calls;
    if(stop_after_delays && delay_calls==stop_after_delays) return E_SYS;
    fake_now += model_delay_guard ? ((time_ms+19U)/10U)*10U : time_ms;
    return E_OK;
}
void tk_ext_tsk(void) { ++task_exits; }
ID tk_cre_cyc(const T_CCYC *p_cfg)
{
    CHECK(p_cfg->cyctim==S3_PERIOD_MS && p_cfg->cycphs==S3_PERIOD_MS);
    CHECK((p_cfg->cycatr&TA_STA)!=0U);
#if TK_SUPPORT_SMP
    CHECK(p_cfg->assprc==TP_PRC1);
#endif
    if(fail_cycle) return E_SYS;
    fake_cycle=*p_cfg;fake_cycle_active=true;cycle_due=fake_now+p_cfg->cycphs+10U;
    return create_object();
}
ER tk_del_cyc(ID object) { CHECK(object>0);++deletes;fake_cycle_active=false;return E_OK; }
ER tk_wup_tsk(ID task) { CHECK(task==42);++wake_credits;return E_OK; }
static void dispatch_cycle(void)
{
    while(fake_cycle_active && (int32_t)(fake_now-cycle_due)>=0) {
        cycle_due+=fake_cycle.cyctim;
        ((void (*)(void *))fake_cycle.cychdr)(fake_cycle.exinf);
    }
}
INT tk_can_wup(ID task)
{
    unsigned int credits;
    CHECK(task==TSK_SELF);dispatch_cycle();credits=wake_credits;wake_credits=0U;
    return (INT)credits;
}
ER tk_slp_tsk(TMO timeout)
{
    CHECK(timeout==TMO_FEVR);++sleeps;
    if(stop_after_sleeps && sleeps==stop_after_sleeps) return E_SYS;
    if(!wake_credits) {CHECK(fake_cycle_active);fake_now=cycle_due;dispatch_cycle();}
    CHECK(wake_credits>0U);--wake_credits;return E_OK;
}
ID tk_get_tid(void) { return caller; }
ER tk_get_otm(SYSTIM *p_time)
{
    if (fail_time) { return E_SYS; }
    p_time->hi = 0; p_time->lo = fake_now; return E_OK;
}
ER tk_snd_mbf(ID object, const void *p_data, INT length, TMO timeout)
{
    if (object <= 0) { return E_OBJ; }
#if BOARD_IR_BARCODE_CAPTURE
    if (object == h_capture)
    {
        CHECK(locked && timeout == TMO_POL && length == (INT)sizeof(s3_capture_record_t));
        if (capture_count == S3_CAPTURE_CAPACITY) return E_TMOUT;
        (void)memcpy(&capture_queue[capture_count++], p_data, sizeof(s3_capture_record_t));
        return E_OK;
    }
#endif
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
    CHECK(object > 0 && length == (INT)S3_EVENT_BYTES);
    CHECK(timeout == TMO_POL && locked);
    if (queue_count == 4U) { return E_TMOUT; }
    CHECK(s3_event_unpack(p_data, &queue[queue_count++]));
    return E_OK;
}
INT tk_rcv_mbf(ID object, void *p_data, TMO timeout)
{
    if (object <= 0) { return E_OBJ; }
#if BOARD_IR_BARCODE_CAPTURE
    if (object == h_capture)
    {
        CHECK(!locked && timeout == TMO_POL);
        if (capture_count == 0U) return E_TMOUT;
        (void)memcpy(p_data, &capture_queue[0], sizeof(s3_capture_record_t));
        --capture_count;
        (void)memmove(capture_queue, capture_queue + 1, capture_count * sizeof(capture_queue[0]));
        return malformed_capture ? 1 : (INT)sizeof(s3_capture_record_t);
    }
#endif
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
        CHECK((timeout == TMO_POL || timeout == S3_ADC_REPLY_WAIT_MS) && !locked);
        if (timeout != TMO_POL && !have_reply)
        {
            ++adc_reply_waits;
            fake_now += adc_wait_delay_ms;
            if (adc_wait_hook != NULL) adc_wait_hook();
            if (adc_auto_worker)
            {
                ID previous = caller;
                caller = 43;
                (void)s3_adc_worker_once(TMO_POL);
                caller = previous;
            }
            if (adc_fail_clock_after_wait) fail_time = true;
        }
        if (!have_reply) { return E_TMOUT; }
        (void)memcpy(p_data, &adc_reply, sizeof(adc_reply));
        have_reply = false;
        return malformed_reply ? malformed_reply : (INT)sizeof(adc_reply);
    }
    CHECK(object > 0 && timeout == TMO_POL && locked);
    if (queue_count == 0U) { return E_TMOUT; }
    CHECK(s3_event_pack(p_data, &queue[0]));
    --queue_count;
    (void)memmove(queue, queue + 1, queue_count * sizeof(queue[0]));
    return (INT)S3_EVENT_BYTES;
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
        fake_now += S3_ADC_REPLY_DEADLINE_MS;
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
    started = false; raw_only = false; acquisition_attempted = false; acquisition_open_error = E_OK;
    applied_generation = 0U;
    h_requests = 0; h_replies = 0; h_worker = 0; h_client = 0; h_device = 0;
    worker_failed = false; have_request = false; have_reply = false;
    adc_request = (s3_adc_request_t){0}; adc_reply = (s3_adc_reply_t){0};
    adc_open_result = 71; adc_read_result = E_OK; adc_read_size = 1;
    adc_read_ms = 0U; adc_opens = 0U; adc_reads = 0U; adc_read_hook = NULL;
    malformed_request = 0; malformed_reply = 0;
    adc_auto_worker = false; adc_wait_delay_ms = 0U;
    adc_fail_clock_after_wait = false; adc_reply_waits = 0U; adc_wait_hook = NULL;
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
    h_sample_cycle=0;fake_cycle=(T_CCYC){0};fake_cycle_active=false;fail_cycle=false;
    cycle_due=0U;wake_credits=0U;sleeps=0U;stop_after_sleeps=0U;task_exits=0U;
    sample_lock_calls=0U;delay_lock_call=0U;state_lock_delay_ms=0U;state_lock_hook=NULL;
    model_delay_guard=false;delay_calls=0U;stop_after_delays=0U;
#if BOARD_IR_BARCODE_CAPTURE
    h_capture = 0; capture_dropped = 0U; capture_count = 0U; malformed_capture = false;
#endif
}
