/* Regression for the robot's first raw capture: all scans timed out.
 * kernel/tkernel/timer.c adds TIMER_PERIOD to a positive relative wait.
 * At a 10 ms tick, consumer_task's tk_dly_tsk(10) next runs at +20 ms.
 * Execute the production ADC pipeline with that cadence, including rollover.
 * This models scheduling time; it does not emulate ADC hardware/interrupts. */
#include "adapter_fixture.h"

static s3_ir_t sensor;
static s3_sample_t sample;

static void setup(uint32_t start)
{
    const s3_ir_config_t cfg = {S3_IR_ADC, {26U, 27U, 28U}};
    reset();
    sensor = (s3_ir_t){0};
    caller = 42;
    fake_now = start;
    CHECK(s3_ir_open(&sensor, &cfg) == E_OK);
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(s3_ir_waiting(&sensor));
}

static ER run_worker(void)
{
    ER er;
    caller = 43;
    er = s3_adc_worker_once(TMO_POL);
    caller = 42;
    return er;
}

static void test_kernel_cadence(void)
{
    const uint32_t starts[] = {0U, UINT32_MAX - 9U};
    unsigned int trial;
    unsigned int scan;
    for (trial = 0U; trial < 2U; ++trial)
    {
        setup(starts[trial]);
        for (scan = 0U; scan < 8U; ++scan)
        {
            uint32_t acquired = fake_now;
            CHECK(run_worker() == E_OK);
            fake_now += 20U; /* Actual +10 request/+10 kernel guard tick. */
            CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_OK);
            CHECK(!sensor.adc_failed && s3_ir_waiting(&sensor));
            CHECK(sample.valid_mask == S3_SENSOR_MASK);
            CHECK(sample.time_ms == acquired && fake_now - sample.time_ms == 20U);
            CHECK(sample.raw[0] == 0U && sample.raw[1] == 4095U && sample.raw[2] == 0U);
        }
        CHECK(adc_reads == 24U && adc_opens == 1U);
    }
}

static void test_late_delivery(void)
{
    setup(100U);
    CHECK(run_worker() == E_OK);
    fake_now = 130U; /* At the existing 30 ms status freshness limit. */
    /* L3: the stale sample is never used, but the converter is fine: the
     * scan is dropped and a fresh one requested, nothing latches. */
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(!sensor.adc_failed && s3_ir_waiting(&sensor) && sample.valid_mask == 0U);
    CHECK(have_request && adc_request.issued_ms == 130U && adc_request.sequence == 2U);
    CHECK(run_worker() == E_OK);
    CHECK(s3_ir_poll(&sensor, 150U, &sample) == E_OK && sample.time_ms == 130U);
    CHECK(adc_reads == 6U && adc_opens == 1U);
}

static void test_conversion_deadline(void)
{
    /* The worker itself is still held to 20 ms from the request. Held off
     * that long before converting (core 0 busy), it drops the scan and keeps
     * serving; the client resubmits. The IR is not latched off (L3). */
    setup(100U);
    fake_now = 120U;
    CHECK(run_worker() == E_TMOUT);
    CHECK(!worker_failed && adc_reads == 0U && adc_opens == 0U);
    CHECK(adc_reply.error == E_TMOUT && adc_reply.retry);
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(!sensor.adc_failed && s3_ir_waiting(&sensor) && sample.valid_mask == 0U);
    CHECK(have_request && adc_request.issued_ms == 120U && adc_request.sequence == 2U);
    CHECK(run_worker() == E_OK && adc_reads == 3U);
    fake_now = 140U;
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_OK && sample.time_ms == 120U);
    CHECK(sample.valid_mask == S3_SENSOR_MASK);
    /* Repeated hold-offs never latch either. */
    for (unsigned int k = 0U; k < 5U; ++k)
    {
        fake_now += 25U;
        CHECK(run_worker() == E_TMOUT && !worker_failed);
        CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT && !sensor.adc_failed);
    }
    CHECK(run_worker() == E_OK);
    CHECK(s3_ir_poll(&sensor, fake_now + 10U, &sample) == E_OK && sample.time_ms == fake_now);
}

/* Only a driver read that itself overran the deadline latches for the boot
 * (it can leave a stale FIFO/IRQ). */
static void test_read_overrun_latched(void)
{
    setup(100U);
    adc_read_ms = 25U;
    CHECK(run_worker() == E_TMOUT && worker_failed && !adc_reply.retry);
    CHECK(adc_reads == 1U && fake_now == 125U);
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(sensor.adc_failed && !s3_ir_waiting(&sensor) && sample.valid_mask == 0U);
    CHECK(run_worker() == E_OBJ && adc_reads == 1U);
    CHECK(s3_ir_poll(&sensor, 300U, &sample) == E_TMOUT && !have_request && adc_reads == 1U);
}

/* The worker task keeps serving after a dropped scan and stops for good
 * after a failed driver read. */
static unsigned int worker_rounds;
static void client_collects(void)
{
    ID previous = caller;
    caller = 42;
    if (worker_rounds++ == 0U)
    {
        /* The client collects the dropped scan and resubmits at once. */
        CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT && !sensor.adc_failed);
        CHECK(have_request && adc_request.issued_ms == fake_now);
    }
    caller = previous;
}
static void test_worker_loop(void)
{
    setup(100U);
    fake_now = 125U; /* the queued scan is already too late to convert */
    adc_request_wait_hook = client_collects;
    worker_rounds = 0U;
    caller = 43;
    s3_adc_worker(0, NULL); /* drop, serve the resubmitted scan, then idle */
    caller = 42;
    CHECK(worker_rounds == 2U && task_exits == 1U && !worker_failed && adc_reads == 3U);
    CHECK(s3_ir_poll(&sensor, 135U, &sample) == E_OK && sample.time_ms == 125U);
    /* The next scan's driver read fails: the worker stops for this boot. */
    adc_request_wait_hook = NULL;
    adc_read_result = E_IO; fake_now = 135U;
    caller = 43;
    s3_adc_worker(0, NULL);
    caller = 42;
    CHECK(task_exits == 2U && worker_failed && adc_reads == 4U);
    CHECK(s3_ir_poll(&sensor, 140U, &sample) == E_IO && sensor.adc_failed);
}

int main(void)
{
    RUN("S3-U34 kernel tick cadence/rollover", test_kernel_cadence);
    RUN("S3-U35 late reply dropped, resubmitted", test_late_delivery);
    RUN("S3-U36 conversion deadline drops the scan only", test_conversion_deadline);
    RUN("S3-U56 read overrun still latched", test_read_overrun_latched);
    RUN("S3-U57 worker loop survives a dropped scan", test_worker_loop);
    (void)printf("Buddy 3 ADC tick: %u groups, %u checks, %u failures\n", groups, checks, failures);
    return failures == 0U ? 0 : 1;
}
