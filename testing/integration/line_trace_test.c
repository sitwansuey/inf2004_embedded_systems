/* Straddle line observer (ir_line_trace.c) with the robot's layout and
 * measured endpoints, configured as vehicle.c does: GP26 right / GP27 left
 * straddle the line, both WHITE is centred, a sensor on BLACK is that side
 * touching it, with the BOARD_LINE_BLACK/WHITE_PERMILLE hysteresis. */
#include <stdio.h>
#include "../../app_program/ir_line_trace.h"
#include "../../app_program/robot_ir_config.h"
static unsigned checks, failures;
#define CHECK(x)                                                               \
    do                                                                         \
    {                                                                          \
        ++checks;                                                              \
        if (!(x))                                                              \
        {                                                                      \
            ++failures;                                                        \
            printf("FAIL line %u: %s\n", (unsigned) __LINE__, #x);             \
        }                                                                      \
    } while (0)
static s3_config_t cfg;
static bool
sample (ir_line_trace_t * s, unsigned mask, uint32_t t)
{
    uint16_t raw[3] = {mask & 1U ? 3214U : 187U, mask & 2U ? 3177U : 189U,
                       4095U};
    return ir_line_observe(s, &cfg.line, raw, 7U, t, t, true);
}
int
main (void)
{
    s3_config_defaults(&cfg);
    robot_ir_apply_layout(&cfg);
    cfg.line.threshold_high = BOARD_LINE_BLACK_PERMILLE;
    cfg.line.threshold_low  = BOARD_LINE_WHITE_PERMILLE;
    CHECK(robot_ir_apply_adc_calibration(&cfg));
    /* Layout and measured endpoints (robot_ir_calibration.h). */
    CHECK(cfg.line_straddle && cfg.line.line_mask == 3U &&
          cfg.barcode_sensor == 2U);
    CHECK(cfg.line.position[0] == 1000 && cfg.line.position[1] == -1000);
    CHECK(cfg.line.white[0] == 187U && cfg.line.black[0] == 3214U &&
          cfg.line.white[1] == 189U && cfg.line.black[1] == 3177U);
#if BOARD_IR_MODE == 3
    /* Mode 3 (the default): GP28 is the digital barcode input, HIGH = black. */
    CHECK(cfg.line.white[2] == 1U - BOARD_BARCODE_DIGITAL_BLACK_LEVEL &&
          cfg.line.black[2] == BOARD_BARCODE_DIGITAL_BLACK_LEVEL);
#else
    /* Mode 2: GP28 on the ADC with its measured endpoints. */
    CHECK(cfg.line.white[2] == 197U && cfg.line.black[2] == 2472U);
#endif
    /* Each mask is reported as is (bit 0 right, bit 1 left) and held. */
    for (unsigned mask = 0U; mask < 4U; ++mask)
    {
        ir_line_trace_t s = {0};
        CHECK(sample(&s, mask, 1000U));
        CHECK(s.usable && s.have_sample && s.result.black_mask == mask);
        CHECK(s.result.darkness[0] == ((mask & 1U) ? 1000U : 0U) &&
              s.result.darkness[1] == ((mask & 2U) ? 1000U : 0U));
        for (uint32_t t = 1010U; t <= 1100U; t += 10U)
        {
            CHECK(sample(&s, mask, t));
            CHECK(s.result.black_mask == mask);
        }
    }
    /* A repeated sample is reused until it is 30 ms old, then the observer
     * resets. */
    {
        ir_line_trace_t s      = {0};
        uint16_t        raw[3] = {187U, 189U, 197U};
        CHECK(ir_line_observe(&s, &cfg.line, raw, 3U, 1000U, 1000U, true));
        for (uint32_t t = 1001U; t < 1030U; ++t)
        {
            CHECK(ir_line_observe(&s, &cfg.line, raw, 3U, 1000U, t, true));
        }
        CHECK(!ir_line_observe(&s, &cfg.line, raw, 3U, 1000U, 1030U, true));
        CHECK(!s.usable && !s.have_sample && s.result.black_mask == 0U);
    }
    /* Both line sensors must be valid; GP28 does not matter. */
    for (unsigned valid = 0U; valid < 8U; ++valid)
    {
        ir_line_trace_t s      = {0};
        uint16_t        raw[3] = {3214U, 3177U, 0U};
        CHECK(ir_line_observe(&s, &cfg.line, raw, (uint8_t) valid, 1000U, 1000U,
                              true) == ((valid & 3U) == 3U));
    }
    /* Timer wrap. */
    {
        ir_line_trace_t s = {0};
        CHECK(sample(&s, 0U, 0xfffffff0U));
        for (unsigned n = 1U; n <= 10U; ++n)
        {
            CHECK(sample(&s, 0U, 0xfffffff0U + n * 10U));
        }
        CHECK(sample(&s, 3U, 100U));
        CHECK(s.usable && s.result.black_mask == 3U);
    }
    /* A gap over 30 ms between samples resets; a sample older than the last
     * too. */
    {
        ir_line_trace_t s = {0};
        CHECK(sample(&s, 1U, 1000U));
        CHECK(!sample(&s, 1U, 1040U));
        CHECK(!s.usable && s.result.black_mask == 0U);
        CHECK(sample(&s, 0U, 1050U));
        CHECK(!sample(&s, 0U, 1040U));
    }
    /* Invalid inputs: future sample, failed source, out-of-range raw, wrong
     * layout. */
    {
        ir_line_trace_t s      = {0};
        uint16_t        raw[3] = {3214U, 3177U, 0U};
        CHECK(!ir_line_observe(&s, &cfg.line, raw, 7U, 1000U, 999U, true));
        CHECK(!ir_line_observe(&s, &cfg.line, raw, 7U, 1000U, 1000U, false));
        raw[0] = 4096U;
        CHECK(!ir_line_observe(&s, &cfg.line, raw, 7U, 1000U, 1000U, true));
        raw[0]               = 3214U;
        s3_line_config_t bad = cfg.line;
        bad.position[0]      = -1000;
        CHECK(!ir_line_observe(&s, &bad, raw, 7U, 1000U, 1000U, true));
        bad           = cfg.line;
        bad.line_mask = 7U;
        CHECK(!ir_line_observe(&s, &bad, raw, 7U, 1000U, 1000U, true));
        CHECK(!ir_line_observe(&s, NULL, raw, 7U, 1000U, 1000U, true) &&
              !ir_line_observe(&s, &cfg.line, NULL, 7U, 1000U, 1000U, true));
        CHECK(!ir_line_observe(NULL, &cfg.line, raw, 7U, 1000U, 1000U, true));
    }
    /* Hysteresis: a reading between WHITE (550) and BLACK (750) per mille keeps
     * the last decision, white or black. */
    {
        ir_line_trace_t s      = {0};
        uint16_t        mid[3] = {2155U, 2131U, 0U};
        for (uint32_t t = 1000U; t <= 1100U; t += 10U)
        {
            CHECK(sample(&s, 0U, t));
        }
        CHECK(ir_line_observe(&s, &cfg.line, mid, 7U, 1110U, 1110U, true));
        CHECK(s.usable && s.result.black_mask == 0U &&
              s.result.darkness[0] > 550U && s.result.darkness[0] < 750U);
        CHECK(sample(&s, 3U, 1120U) && s.result.black_mask == 3U);
        CHECK(ir_line_observe(&s, &cfg.line, mid, 7U, 1130U, 1130U, true) &&
              s.result.black_mask == 3U);
        CHECK(sample(&s, 0U, 1140U) && s.result.black_mask == 0U);
    }
    printf("Straddle line observer (IR mode %d): %u checks, %u failures\n",
           BOARD_IR_MODE, checks, failures);
    return failures ? 1 : 0;
}

/*** end of file ***/
