/* Executes actual Buddy 3 adapter code with deterministic RTOS/GPIO fakes.
 * Does not simulate a robot or establish physical hardware correctness. */
#include "host_support.h"
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem3/subsystem3.c"
#include "../../app_program/subsystem3/task_consumer.c"

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
    CHECK(p_cfg->maxmsz == (INT)sizeof(s3_event_record_t));
    CHECK(p_cfg->bufsz >= (INT)(4U * (sizeof(s3_event_record_t) + sizeof(UW))));
    return create_object();
}
ID tk_cre_tsk(const T_CTSK *p_cfg)
{
    CHECK(p_cfg->itskpri == 16 && p_cfg->stksz == 2048U);
#if TK_SUPPORT_SMP
    CHECK(p_cfg->assprc == TP_PRC1);
#endif
    return (fail_create == 4U) ? E_SYS : 42;
}
ER tk_del_mbx(ID object) { CHECK(object > 0); ++deletes; return E_OK; }
ER tk_del_mtx(ID object) { CHECK(object > 0); ++deletes; return E_OK; }
ER tk_del_tsk(ID object) { CHECK(object == 42); ++deletes; return E_OK; }
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
    CHECK(task == 42 && start == 0);
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
    CHECK(object > 0 && length == (INT)sizeof(s3_event_record_t));
    CHECK(timeout == TMO_POL && locked);
    if (queue_count == 4U) { return E_TMOUT; }
    (void)memcpy(&queue[queue_count++], p_data, (size_t)length);
    return E_OK;
}
INT tk_rcv_mbf(ID object, void *p_data, TMO timeout)
{
    CHECK(object > 0 && timeout == TMO_POL && locked);
    if (queue_count == 0U) { return E_TMOUT; }
    (void)memcpy(p_data, &queue[0], sizeof(queue[0]));
    --queue_count;
    (void)memmove(queue, queue + 1, queue_count * sizeof(queue[0]));
    return (INT)sizeof(s3_event_record_t);
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
    started = false; acquisition_attempted = false; applied_generation = 0U;
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

static void setup(void)
{
    s3_adapter_config_t cfg = {0};
    unsigned int i;
    reset();
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.algorithms.line.raw_max = 1U;
    cfg.algorithms.line.line_mask = 3U;
    cfg.algorithms.barcode_sensor = 2U;
    cfg.algorithms.map_a = S3_NAV_LEFT; /* Explicitly a test mapping only. */
    cfg.ir.mode = S3_IR_DIGITAL;
    for (i = 0U; i < 3U; ++i)
    {
        cfg.algorithms.line.black[i] = 1U;
        cfg.ir.pin[i] = (uint8_t)(i + 2U); /* Synthetic host pins. */
    }
    CHECK(s3_configure(&cfg) == E_OK);
    CHECK(subsystem3_start() == E_OK);
    CHECK(s3_controller_bind() == E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    gpio[2] = 1U;
}

static ER tick(bool black)
{
    ER result;
    fake_now += 10U;
    caller = 7;
    if (auto_renew && control.mode == S3_MODE_TRACK &&
        ((fake_now - control.issued_ms) >= 100U))
    {
        CHECK(s3_controller_renew(1000U) == E_OK);
    }
    caller = 42; gpio[4] = black ? 1U : 0U;
    result = s3_service_once();
    caller = 7;
    return result;
}

static void marker(void)
{
    const unsigned int widths[] = {
        1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1
    };
    unsigned int i;
    unsigned int t;
    for (i = 0U; i < 22U; ++i) { (void)tick(false); }
    (void)tick(true);
    for (i = 0U; i < 29U; ++i)
    {
        for (t = 10U; t <= widths[i] * 40U; t += 10U)
        {
            bool black = (i % 2U) == 0U;
            (void)tick(t == widths[i] * 40U ? !black : black);
        }
    }
    for (t = 0U; t < 500U; t += 10U) { (void)tick(false); }
}

static void test_disabled_and_lifecycle(void)
{
    s3_status_t snapshot;
    unsigned int failure;
    reset();
    CHECK(subsystem3_start() == E_OBJ);
    for (failure = 1U; failure <= 3U; ++failure)
    {
        reset(); fail_create = failure;
        CHECK(subsystem3_init() == E_SYS);
        CHECK(deletes == failure - 1U);
        fail_create = 0U;
        CHECK(subsystem3_init() == E_OK);
    }
    reset(); CHECK(subsystem3_init() == E_OK);
    CHECK(subsystem3_init() == E_OBJ);
    fail_start = true; CHECK(subsystem3_start() == E_SYS);
    CHECK(!started && deletes == 1U);
    fail_start = false; CHECK(subsystem3_start() == E_OK);
    CHECK(subsystem3_start() == E_OBJ);
    caller = 42;
    CHECK(s3_service_once() == E_NOSPT);
    CHECK(gpio_reads == 0U && gpio_configs == 0U);
    CHECK(s3_status_get(&snapshot) == E_OK);
    CHECK(!snapshot.configured && !snapshot.output.steering_allowed);
    CHECK(snapshot.output.error == S3_ERROR_CONFIG);
}

static void test_controller_leases_and_safety(void)
{
    s3_status_t snapshot;
    unsigned int i;
    setup();
    CHECK(tick(false) == E_OK);
    CHECK(s3_status_get(&snapshot) == E_OK && snapshot.output.steering_allowed);
    caller = 8;
    CHECK(s3_controller_bind() == E_OACV);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OACV);
    CHECK(s3_event_receive(&(s3_event_record_t){0}) == E_OACV);
    caller = 7;
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 0U) == E_PAR);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1001U) == E_PAR);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    (void)tick(false);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 100U) == E_OK);
    auto_renew = false;
    for (i = 0U; i < 10U; ++i) { (void)tick(false); }
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    CHECK(s3_controller_renew(1000U) == E_OBJ);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    (void)tick(false);
    CHECK(s3_status_get(&snapshot) == E_OK && snapshot.output.steering_allowed);
    fake_now += 30U;
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    fail_time = true;
    CHECK(s3_status_get(&snapshot) == E_SYS && !snapshot.output.steering_allowed);
    caller = 42;
    CHECK(s3_service_once() == E_SYS);
}

static void test_override_publication_race(void)
{
    s3_status_t snapshot;
    setup(); (void)tick(false);
    race_override = true;
    CHECK(tick(false) == E_TMOUT);
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    CHECK(queue_count == 0U);
    (void)tick(false);
    CHECK(!status.output.steering_allowed);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    (void)tick(false);
    CHECK(status.output.steering_allowed);
    busy_mutex = true;
    CHECK(s3_status_get(&snapshot) == E_TMOUT && !snapshot.output.steering_allowed);
    CHECK(tick(false) == E_TMOUT);
}

static void test_event_handoff_and_staleness(void)
{
    s3_event_record_t record;
    setup(); marker();
    CHECK(queue_count == 1U && !status.output.steering_allowed);
    CHECK(s3_event_receive(&record) == E_OK);
    CHECK(record.event.kind == S3_EVENT_NAVIGATION && record.event.symbol == 'A');
    CHECK(record.event.command == S3_NAV_LEFT);
    CHECK(s3_event_receive(&record) == E_TMOUT && record.event.kind == S3_EVENT_NONE);
    CHECK(s3_controller_renew(1000U) == E_OK);
    CHECK((core.mode == S3_MODE_TRACK) && core.awaiting_controller); /* Renewal cannot resume. */
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    marker(); CHECK(queue_count == 0U); /* Same physical marker, no rearm. */
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 1000U) == E_OK);
    marker(); CHECK(queue_count == 1U);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_event_receive(&record) == E_TMOUT); /* Generation invalidation. */
    setup(); marker(); fake_now += 100U;
    CHECK(s3_event_receive(&record) == E_TMOUT); /* Event too old. */
    setup(); marker();
    gpio[2] = 2U; (void)tick(false);
    CHECK(s3_event_receive(&record) == E_OK); /* Fault may be received. */
    CHECK(record.event.kind == S3_EVENT_FAULT); /* Old turn was discarded. */
}

static void test_transport_overflow(void)
{
    s3_event_record_t record;
    setup(); queue_count = 4U;
    (void)memset(queue, 0, sizeof(queue));
    marker();
    CHECK(status.output.error == S3_ERROR_TRANSPORT);
    CHECK(!status.output.steering_allowed && core.awaiting_controller);
    CHECK(s3_event_receive(&record) == E_TMOUT);
}

static void test_sensor_configuration(void)
{
    s3_ir_config_t cfg = {S3_IR_DIGITAL, {26U, 27U, 28U}};
    s3_ir_t sensor = {0};
    s3_sample_t sample;
    reset();
    CHECK(s3_ir_config_valid(&cfg));
    cfg.pin[0] = 29U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 23U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 19U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 27U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 26U;
    gpio_config_error = E_IO;
    CHECK(s3_ir_open(&sensor, &cfg) == E_IO);
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_NOSPT);
    gpio_config_error = E_OK;
    CHECK(s3_ir_open(&sensor, &cfg) == E_OK);
    CHECK(s3_ir_open(&sensor, &cfg) == E_OBJ);
    gpio[26] = 2U;
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_IO && sample.valid_mask == 0U);
}

static void test_adc_boundary(void)
{
    s3_ir_config_t cfg = {S3_IR_ADC, {26U, 27U, 28U}};
    s3_ir_t sensor = {0};
    s3_sample_t sample;
    s3_adapter_config_t adapter = {0};
    reset();
    CHECK(s3_ir_open(&sensor, &cfg) == E_NOSPT);
    CHECK(gpio_configs == 0U);
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_NOSPT);
    CHECK(sample.valid_mask == 0U);
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&adapter.algorithms);
    adapter.ir = cfg;
    CHECK(s3_configure(&adapter) == E_NOSPT);
    CHECK(configuration.ir.mode == S3_IR_DISABLED);
}

int main(void)
{
    RUN("S3-U15 lifecycle/disabled", test_disabled_and_lifecycle);
    RUN("S3-U16 controller leases", test_controller_leases_and_safety);
    RUN("S3-U17 safety publication race", test_override_publication_race);
    RUN("S3-U18 event handoff/staleness", test_event_handoff_and_staleness);
    RUN("S3-U19 event queue full", test_transport_overflow);
    RUN("S3-U20 sensor configuration", test_sensor_configuration);
    RUN("S3-U21 ADC boundary", test_adc_boundary);
    (void)printf("Buddy 3 adapter: %u groups, %u checks, %u failures\n",
                 groups, checks, failures);
    return (failures == 0U) ? 0 : 1;
}
