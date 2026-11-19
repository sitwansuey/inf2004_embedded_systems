/* L2: a GP28 marker cut short by a sampler hiccup is a failed read (so the
 * vehicle re-reads it), and a capture that ends by itself with 3-8 elements
 * is counted as partial (read-rate measurement), never as a failed read.
 * Robot-like core profile: re-arm on failure, end-of-marker quiet time. */
#include <stdio.h>
#include <string.h>
#include "buddy3_core.h"

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static s3_core_t core;
static s3_output_t out;
static uint32_t now = 1000U;

static void sample_at(int black, uint32_t time_ms)
{
    s3_sample_t s;
    memset(&s, 0, sizeof(s));
    s.valid_mask = 7U; s.time_ms = time_ms;
    s.raw[2] = black ? 1U : 0U;
    s3_core_step(&core, &s, &out);
}

static void step(int black, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 10U) { sample_at(black, now); now += 10U; }
}

/* n elements of 40 ms, black first: n edges once the next colour starts. */
static void elements(unsigned n)
{
    for (unsigned i = 0U; i < n; ++i) step(!(i & 1U), 40U);
}

static void setup(bool fault_fails)
{
    s3_config_t c;
    s3_config_defaults(&c);
    CHECK(!c.barcode_fault_fails); /* off unless the board turns it on */
    c.line.line_mask = 3U; c.barcode_sensor = 2U; c.barcode_retry = true; c.line_straddle = true;
    for (unsigned i = 0U; i < 3U; ++i) { c.line.white[i] = 0U; c.line.black[i] = 1U; }
    c.barcode.min_run_ms = 20U; c.barcode.end_quiet_ms = 1000U;
    c.barcode_fault_fails = fault_fails;
    CHECK(s3_core_init(&core, &c));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    step(0, 400U);
}

int main(void)
{
    /* 1. Natural end with 5 elements: partial, not a failed read. */
    setup(true);
    elements(5); step(0, 1100U);
    CHECK(out.barcode_partial_count == 1U && core.partial_elements == 5U);
    CHECK(out.barcode_fail_count == 0U && out.barcode_abort_count == 0U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_NONE && out.error == S3_ERROR_NONE);
    /* One black-white pair (a crossing line) is not counted at all. */
    step(0, 300U);
    elements(2); step(0, 1100U);
    CHECK(out.barcode_partial_count == 1U && out.barcode_fail_count == 0U);
    /* Nine or more elements stay a failed read, not partial. */
    step(0, 300U);
    elements(9); step(0, 1100U);
    CHECK(out.barcode_fail_count == 1U && out.barcode_partial_count == 1U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_FAILED && out.barcode_elements == 9U);

    /* 2. A sampler hiccup 6 elements into a marker: failed read. */
    setup(true);
    elements(6); step(1, 20U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_READING && out.barcode_elements == 6U);
    s3_core_fault(&core, S3_ERROR_STALE);
    s3_core_barcode_summary(&core, &out);
    CHECK(out.barcode_fail_count == 1U && out.barcode_abort_count == 1U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_FAILED);
    CHECK(out.barcode_elements == 6U && out.barcode_bars == 4U);
    CHECK(out.barcode_partial_count == 0U);
    CHECK(core.latched_error == S3_ERROR_STALE && core.awaiting_controller);
    /* The latch then aborts nothing more: no double count. */
    s3_core_fault(&core, S3_ERROR_SENSOR);
    step(0, 50U);
    CHECK(out.barcode_fail_count == 1U && out.barcode_abort_count == 1U);
    CHECK(out.event.kind == S3_EVENT_NONE); /* silent: the latch came first */
    /* The controller re-arms (TRACK, marker clear): a full read decodes. */
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    step(0, 300U);
    {
        static const uint16_t seq[3] = {0x094U, 0x109U, 0x094U}; /* *A* */
        for (unsigned ch = 0U, k = 0U; ch < 3U; ++ch) {
            for (unsigned i = 0U; i < 9U; ++i, ++k)
                step(!(k & 1U), ((seq[ch] >> (8U - i)) & 1U) ? 120U : 40U);
            if (ch < 2U) { step(0, 40U); ++k; }
        }
    }
    step(0, 10U);
    s3_core_barcode_end(&core);
    step(0, 10U);
    CHECK(out.barcode_ok_count == 1U && out.last_symbol == 'A');
    CHECK(out.barcode_fail_count == 1U && out.barcode_abort_count == 1U);

    /* 3. Only 2 elements in: a crossing line, not a read. */
    setup(true);
    elements(2); step(1, 20U);
    s3_core_fault(&core, S3_ERROR_STALE);
    s3_core_barcode_summary(&core, &out);
    CHECK(out.barcode_fail_count == 0U && out.barcode_abort_count == 0U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_NONE);

    /* 4. A late sample seen by the core itself (gap over max_sample_gap_ms)
     * cuts the read short the same way. */
    setup(true);
    elements(4); step(1, 20U);
    sample_at(1, now + 40U); now += 50U;
    CHECK(out.error == S3_ERROR_STALE && core.latched_error == S3_ERROR_STALE);
    CHECK(out.barcode_fail_count == 1U && out.barcode_abort_count == 1U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_FAILED && out.barcode_elements == 4U);

    /* 5. Off (default): the same hiccup is not counted, as before. */
    setup(false);
    elements(6); step(1, 20U);
    s3_core_fault(&core, S3_ERROR_STALE);
    s3_core_barcode_summary(&core, &out);
    CHECK(out.barcode_fail_count == 0U && out.barcode_abort_count == 0U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_NONE);

    /* 6. Not tracking: a controller mode change is not a failed read. */
    setup(true);
    elements(6); step(1, 20U);
    CHECK(s3_core_control(&core, S3_MODE_OVERRIDE, false));
    s3_core_fault(&core, S3_ERROR_STALE);
    s3_core_barcode_summary(&core, &out);
    CHECK(out.barcode_fail_count == 0U && out.barcode_abort_count == 0U);

    printf("S3 L2 fault-cut and partial barcode reads: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
