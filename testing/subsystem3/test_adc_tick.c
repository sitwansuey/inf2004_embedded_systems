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
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(sensor.adc_failed && sample.valid_mask == 0U);
    CHECK(s3_ir_poll(&sensor, 140U, &sample) == E_TMOUT);
    CHECK(adc_reads == 3U); /* No retries after the latched fault. */
}

static void test_conversion_deadline(void)
{
    setup(100U);
    fake_now = 120U; /* Worker itself is still constrained to 20 ms. */
    CHECK(run_worker() == E_TMOUT);
    CHECK(worker_failed && adc_reads == 0U && adc_opens == 0U);
    CHECK(s3_ir_poll(&sensor, fake_now, &sample) == E_TMOUT);
    CHECK(sensor.adc_failed && sample.valid_mask == 0U);
}

int main(void)
{
    RUN("S3-U34 kernel tick cadence/rollover", test_kernel_cadence);
    RUN("S3-U35 stale reply rejected", test_late_delivery);
    RUN("S3-U36 conversion deadline retained", test_conversion_deadline);
    (void)printf("Buddy 3 ADC tick: %u groups, %u checks, %u failures\n", groups, checks, failures);
    return failures == 0U ? 0 : 1;
}
