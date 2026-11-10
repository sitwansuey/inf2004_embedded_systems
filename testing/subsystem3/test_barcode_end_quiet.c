/* GP28 digital barcode: a marker ends only after 1 s without an edge.
 * Checks bars/result reporting, both directions, and that noise, a trailing
 * mark or overflow never end the capture early. */
#include <stdio.h>
#include <string.h>
#include "buddy3_core.h"

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static const uint16_t code39[26] = {
    0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
    0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
    0x181,0x0C1,0x1C0,0x091,0x190,0x0D0
};
static s3_core_t core;
static s3_output_t out;
static uint32_t now = 1000U;
static unsigned results; /* non-NONE barcode statuses seen */
static uint8_t last_status;

static void step(int black, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 10U) {
        s3_sample_t s;
        memset(&s, 0, sizeof(s));
        s.valid_mask = 7U; s.time_ms = now;
        s.raw[0] = 0U; s.raw[1] = 0U; s.raw[2] = black ? 1U : 0U;
        s3_core_step(&core, &s, &out);
        if (out.barcode_status != S3_BARCODE_NONE) { ++results; last_status = out.barcode_status; }
        now += 10U;
    }
}

/* Drive across a marker whose elements are given in position units, with a
 * speed (position units per 10 ms sample) that changes along the way. The
 * colour of each sample is whatever is under the sensor at that position. */
static uint32_t position = 100000U;
static void drive(const uint32_t *w, unsigned n, const uint32_t *speeds, unsigned nspeeds,
                  bool with_position)
{
    uint32_t start = position, total = 0U;
    for (unsigned i = 0U; i < n; ++i) total += w[i];
    for (unsigned k = 0U; position - start < total + 200U; ++k) {
        uint32_t travelled = position - start, edge = 0U;
        int black = 0;
        for (unsigned i = 0U; i < n; ++i) {
            if (travelled < edge + w[i]) { black = !(i & 1U); break; }
            edge += w[i];
        }
        s3_sample_t s;
        memset(&s, 0, sizeof(s));
        s.valid_mask = 7U; s.time_ms = now;
        s.raw[2] = black ? 1U : 0U;
        s.position = position; s.position_valid = with_position;
        s3_core_step(&core, &s, &out);
        if (out.barcode_status != S3_BARCODE_NONE) { ++results; last_status = out.barcode_status; }
        now += 10U;
        position += speeds[(k / 7U) % nspeeds];
    }
}

/* Element widths of a framed *X*, black first. */
static unsigned frame(char letter, uint32_t narrow, uint32_t wide, uint32_t widths[29])
{
    const uint16_t seq[3] = {0x094U, code39[letter - 'A'], 0x094U};
    unsigned k = 0U;
    for (unsigned c = 0U; c < 3U; ++c) {
        for (unsigned i = 0U; i < 9U; ++i) widths[k++] = ((seq[c] >> (8U - i)) & 1U) ? wide : narrow;
        if (c < 2U) widths[k++] = narrow;
    }
    return k;
}

static void cross(const uint32_t *w, unsigned n, int reverse)
{
    for (unsigned i = 0U; i < n; ++i) step(!(i & 1U), w[reverse ? n - 1U - i : i]);
}

/* Read one marker at normal speed; out holds the decoding sample. */
static void read_marker(char letter)
{
    uint32_t w[29];
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    step(0, 300U);
    cross(w, frame(letter, 40U, 120U, w), 0);
    step(0, 10U);
    s3_core_barcode_end(&core);
    s3_sample_t s; memset(&s, 0, sizeof(s));
    s.valid_mask = 7U; s.time_ms = now; now += 10U;
    s3_core_step(&core, &s, &out);
    CHECK(out.barcode_status == S3_BARCODE_OK && out.decoded.symbol == letter);
}

/* The decoding sample turned (NAVIGATION) or was read with no turn (UNMAPPED). */
static int turned(char letter, s3_navigation_t turn)
{
    if (turn == S3_NAV_NONE)
        return out.event.kind != S3_EVENT_NAVIGATION && out.error == S3_ERROR_UNMAPPED;
    return out.event.kind == S3_EVENT_NAVIGATION && out.event.command == turn &&
           out.event.symbol == letter;
}

static void setup(void)
{
    s3_config_t c;
    s3_config_defaults(&c);
    c.line.line_mask = 3U; c.barcode_sensor = 2U; c.barcode_retry = true; c.line_straddle = true;
    c.line.raw_max = 4095U;
    for (unsigned i = 0U; i < 3U; ++i) { c.line.white[i] = 0U; c.line.black[i] = 1U; }
    c.barcode.min_run_ms = 20U; c.barcode.end_quiet_ms = 1000U;
    c.map_a = S3_NAV_LEFT;
    CHECK(s3_core_init(&core, &c));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    step(0, 400U);
}

int main(void)
{
    uint32_t w[29];
    setup();
    CHECK(out.barcode_result == S3_BARCODE_RESULT_NONE && out.barcode_bars == 0U);

    /* Forward C: nothing reported until 1 s of stillness, then success. */
    unsigned n = frame('C', 40U, 120U, w);
    cross(w, n, 0);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_READING && out.barcode_bars == 15U);
    CHECK(out.barcode_elements == 28U); /* last bar still under the sensor */
    step(0, 10U);
    /* Leaving the last bar completes element 29: the vehicle stops reversing
     * here, about 1 s before the reader reports. */
    CHECK(out.barcode_elements == 29U && out.barcode_result == S3_BARCODE_RESULT_READING);
    step(0, 890U);
    CHECK(results == 0U && out.barcode_result == S3_BARCODE_RESULT_READING);
    step(0, 200U);
    CHECK(results == 1U && last_status == S3_BARCODE_OK);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_DECODED && out.barcode_bars == 15U);
    CHECK(out.last_symbol == 'C' && out.barcode_ok_count == 1U && out.barcode_fail_count == 0U);

    /* Slower and backwards (30 % speed reverse pass) still decodes. The
     * reader re-arms after 200 ms of white (the robot's brake pause). */
    step(0, 300U);
    n = frame('C', 130U, 390U, w);
    cross(w, n, 1);
    step(0, 1100U);
    CHECK(results == 2U && last_status == S3_BARCODE_OK && out.barcode_ok_count == 2U);
    CHECK(out.barcode_bars == 15U && out.barcode_result == S3_BARCODE_RESULT_DECODED);

    step(0, 300U);
    /* Garbled marker: keeps reading to the end, then fails with its bar count. */
    for (unsigned i = 0U; i < 29U; ++i) step(!(i & 1U), 40U + (i % 3U) * 30U);
    step(0, 1100U);
    CHECK(results == 3U && last_status != S3_BARCODE_OK);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_FAILED && out.barcode_bars == 15U);
    CHECK(out.barcode_elements == 29U);
    CHECK(out.barcode_fail_count == 1U);

    step(0, 300U);
    /* A single line crossing is not a barcode: result/bars unchanged. */
    step(1, 60U);
    CHECK(out.barcode_result == S3_BARCODE_RESULT_READING && out.barcode_bars == 1U && out.barcode_elements < 29U);
    step(0, 1100U);
    CHECK(out.barcode_fail_count == 1U && out.barcode_result == S3_BARCODE_RESULT_FAILED && out.barcode_bars == 15U && out.barcode_elements == 29U);

    step(0, 300U);
    /* A glitch inside a marker does not end it early; failure only after 1 s. */
    unsigned before = results;
    n = frame('E', 40U, 120U, w);
    for (unsigned i = 0U; i < n; ++i) {
        step(!(i & 1U), w[i]);
        if (i == 11U) { step(1, 10U); step(0, 10U); } /* 10 ms black speck in a space */
    }
    step(0, 500U);
    CHECK(results == before && out.barcode_result == S3_BARCODE_RESULT_READING);
    step(0, 600U);
    CHECK(results == before + 1U && out.barcode_result == S3_BARCODE_RESULT_FAILED);
    CHECK(out.barcode_bars == 16U && out.barcode_fail_count == 2U);

    step(0, 300U);
    /* A stray mark within 1 s after the code: the first complete window wins. */
    n = frame('A', 40U, 120U, w);
    cross(w, n, 0);
    step(0, 300U); step(1, 60U); step(0, 100U); step(1, 60U);
    step(0, 1100U);
    CHECK(last_status == S3_BARCODE_OK && out.last_symbol == 'A');
    CHECK(out.barcode_bars == 17U && out.barcode_result == S3_BARCODE_RESULT_DECODED);
    CHECK(out.event.kind == S3_EVENT_NONE || out.event.kind == S3_EVENT_NAVIGATION);

    /* Distance end (vehicle saw 30 mm without an edge): finish at once.
     * 'A' is a mapped turn, so the controller acknowledges it first. */
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    step(0, 300U);
    before = results;
    s3_core_barcode_end(&core); /* nothing running: ignored */
    step(0, 20U);
    CHECK(results == before && out.barcode_result == S3_BARCODE_RESULT_DECODED);
    n = frame('D', 130U, 390U, w); /* slow pass */
    cross(w, n, 0);
    step(0, 10U);
    CHECK(out.barcode_elements == 29U && out.barcode_result == S3_BARCODE_RESULT_READING);
    s3_core_barcode_end(&core);
    step(0, 10U);
    CHECK(results == before + 1U && last_status == S3_BARCODE_OK && out.last_symbol == 'D');
    CHECK(out.barcode_result == S3_BARCODE_RESULT_DECODED && out.barcode_elements == 29U);

    /* An edge after the request means the marker is not over: the request is
     * cancelled and the capture still reads the whole code. */
    step(0, 300U);
    before = results;
    n = frame('B', 40U, 120U, w);
    for (unsigned i = 0U; i < n; ++i) {
        step(!(i & 1U), w[i]);
        if (i == 13U) s3_core_barcode_end(&core); /* mid-marker, inside a space */
    }
    CHECK(results == before && out.barcode_result == S3_BARCODE_RESULT_READING);
    step(0, 10U);
    s3_core_barcode_end(&core);
    step(0, 10U);
    CHECK(results == before + 1U && last_status == S3_BARCODE_OK && out.last_symbol == 'B');

    /* Distance-measured bars: narrow 23 / wide 69 position units (~4/12 mm).
     * The speed changes up to 8x (~20..140 mm/s) and stops completely while
     * crossing, which wrecks millisecond widths but not travelled distance. */
    {
        static const uint32_t speeds[] = {1U, 0U, 8U, 3U, 0U, 6U};
        core.config.barcode.min_run_position = 4U;
        CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
        step(0, 300U);
        before = results;
        n = frame('D', 23U, 69U, w);
        drive(w, n, speeds, 6U, true);
        s3_core_barcode_end(&core);
        step(0, 10U);
        CHECK(results == before + 1U && last_status == S3_BARCODE_OK && out.last_symbol == 'D');
        CHECK(out.barcode_elements == 29U);

        /* The same uneven crossing measured in milliseconds fails. */
        step(0, 300U);
        before = results;
        drive(w, n, speeds, 6U, false);
        s3_core_barcode_end(&core);
        step(0, 10U);
        CHECK(results == before + 1U && last_status != S3_BARCODE_OK);

        /* Backwards at a crawl with bursts: still decodes ('C' read in
         * reverse), and the slow 1-unit steps are not mistaken for noise. */
        static const uint32_t reverse_speeds[] = {2U, 1U, 7U, 2U};
        step(0, 300U);
        before = results;
        n = frame('C', 23U, 69U, w);
        uint32_t backwards[29];
        for (unsigned i = 0U; i < n; ++i) backwards[i] = w[n - 1U - i];
        drive(backwards, n, reverse_speeds, 4U, true);
        s3_core_barcode_end(&core);
        step(0, 10U);
        CHECK(results == before + 1U && last_status == S3_BARCODE_OK && out.last_symbol == 'C');
    }

    /* Codebook: A = U-turn, Z = right, F = left; B is read but unmapped. */
    {
        static const struct { char letter; s3_navigation_t turn; } book[] = {
            {'F', S3_NAV_LEFT}, {'Z', S3_NAV_RIGHT}, {'A', S3_NAV_UTURN}, {'B', S3_NAV_NONE}};
        core.config.map_a = S3_NAV_UTURN; core.config.map_z = S3_NAV_RIGHT;
        core.config.map_f = S3_NAV_LEFT; core.config.map_b = S3_NAV_NONE;
        for (unsigned k = 0U; k < 4U; ++k) {
            CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
            step(0, 300U);
            n = frame(book[k].letter, 40U, 120U, w);
            cross(w, n, 0);
            step(0, 10U);
            s3_core_barcode_end(&core);
            /* Capture the decoding sample itself to see its event. */
            s3_sample_t s; memset(&s, 0, sizeof(s));
            s.valid_mask = 7U; s.time_ms = now; now += 10U;
            s3_core_step(&core, &s, &out);
            CHECK(out.barcode_status == S3_BARCODE_OK && out.decoded.symbol == book[k].letter);
            if (book[k].turn != S3_NAV_NONE)
                CHECK(out.event.kind == S3_EVENT_NAVIGATION && out.event.command == book[k].turn &&
                      out.event.symbol == book[k].letter);
            else
                CHECK(out.event.kind != S3_EVENT_NAVIGATION && out.error == S3_ERROR_UNMAPPED);
        }
    }

    /* Operator overrides (MQTT MAP) win over the codebook; 0 = read, no turn;
     * letter 0 restores the codebook. Invalid input changes nothing. */
    {
        s3_core_t fresh;
        s3_config_t c;
        s3_config_defaults(&c);
        CHECK(s3_core_init(&fresh, &c));
        for (unsigned i = 0U; i < 26U; ++i) CHECK(fresh.map_override[i] == S3_MAP_DEFAULT);
        CHECK(!s3_core_set_map(NULL, 1U, 1U));
        CHECK(!s3_core_set_map(&core, 27U, S3_NAV_LEFT) && !s3_core_set_map(&core, 1U, 5U));
        CHECK(!s3_core_set_map(&core, 0U, S3_NAV_LEFT));
        for (unsigned i = 0U; i < 26U; ++i) CHECK(core.map_override[i] == S3_MAP_DEFAULT);
        CHECK(s3_core_set_map(&core, 6U, S3_NAV_RIGHT));    /* F: left -> right */
        CHECK(s3_core_set_map(&core, 2U, S3_NAV_STRAIGHT)); /* B: unmapped -> straight */
        CHECK(s3_core_set_map(&core, 1U, S3_NAV_NONE));     /* A: U-turn -> ignore */
        CHECK(s3_core_set_map(&core, 17U, S3_NAV_UTURN));   /* Q: unmapped -> U-turn */
        read_marker('F'); CHECK(turned('F', S3_NAV_RIGHT));
        read_marker('B'); CHECK(turned('B', S3_NAV_STRAIGHT));
        unsigned ok = out.barcode_ok_count;
        read_marker('A'); CHECK(turned('A', S3_NAV_NONE) && out.last_symbol == 'A');
        CHECK(out.barcode_ok_count == (uint8_t)(ok + 1U) && !out.awaiting_controller);
        read_marker('Q'); CHECK(turned('Q', S3_NAV_UTURN));
        read_marker('Z'); CHECK(turned('Z', S3_NAV_RIGHT)); /* not overridden */
        CHECK(s3_core_set_map(&core, 0U, 0U));
        for (unsigned i = 0U; i < 26U; ++i) CHECK(core.map_override[i] == S3_MAP_DEFAULT);
        read_marker('F'); CHECK(turned('F', S3_NAV_LEFT));
        read_marker('A'); CHECK(turned('A', S3_NAV_UTURN));
        read_marker('B'); CHECK(turned('B', S3_NAV_NONE));
        read_marker('Q'); CHECK(turned('Q', S3_NAV_NONE));
    }

    /* Live GP28 bit (the vehicle's junction test): this sample's colour in
     * any mode, 0 for an invalid sample. */
    step(0, 10U);
    CHECK(!out.barcode_black);
    step(1, 10U);
    CHECK(out.barcode_black);
    CHECK(s3_core_control(&core, S3_MODE_OVERRIDE, false));
    step(1, 10U);
    CHECK(out.barcode_black);
    step(0, 10U);
    CHECK(!out.barcode_black);
    step(1, 10U);
    {
        s3_sample_t s; memset(&s, 0, sizeof(s));
        s.valid_mask = 3U; s.time_ms = now; s.raw[2] = 1U; now += 10U;
        s3_core_step(&core, &s, &out);
        CHECK(!out.barcode_black && out.error == S3_ERROR_SENSOR);
    }

    printf("S3 GP28 end-quiet barcode: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
