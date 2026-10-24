#include "buddy3_barcode.h"

#include <string.h>

/* From the vector bars in supplied Barcode Sample (1).pdf, forms 7 and 15.
 * Nine elements per character, 3 wide elements, narrow:wide = 1:3. */
#define S3_FRAME_PATTERN 0x094U
#define S3_A_PATTERN 0x109U
#define S3_Z_PATTERN 0x0D0U

/* A reflective threshold can shorten black pulses and lengthen white gaps.
 * First require three clearly separated wide elements, then intersect the
 * original 1:3 +/-25% bands with one bounded black/white edge offset. No
 * symbol is used to choose widths. An offset above half a narrow unit fails.
 * Percent-scaled signed 64-bit arithmetic preserves fractional bounds and
 * remains bounded for every uint32_t input. No allocation or persistent state. */
static bool character(const uint32_t *p_runs, uint16_t *p_pattern,
                       uint32_t *p_unit, int64_t *p_bias100)
{
    uint32_t sorted[9];
    uint16_t pattern = 0U;
    size_t i;
    (void)memcpy(sorted, p_runs, sizeof(sorted));
    for (i = 1U; i < 9U; ++i)
    {
        size_t j = i;
        uint32_t value = sorted[i];
        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    *p_unit = (uint32_t)(((uint64_t)sorted[2] + sorted[3]) / 2U);
    if ((sorted[0] == 0U) || (*p_unit == 0U) ||
        ((uint64_t)sorted[6] * 2U < (uint64_t)sorted[5] * 3U))
    {
        return false;
    }
    int64_t lower = -(int64_t)*p_unit * 50;
    int64_t upper = (int64_t)*p_unit * 50;
    for (i = 0U; i < 9U; ++i)
    {
        bool wide = p_runs[i] >= sorted[6];
        int64_t low = (int64_t)*p_unit * (wide ? 225 : 75);
        int64_t high = (int64_t)*p_unit * (wide ? 375 : 125);
        int64_t value = (int64_t)p_runs[i] * 100;
        int64_t a = ((i & 1U) == 0U) ? low - value : value - high;
        int64_t b = ((i & 1U) == 0U) ? high - value : value - low;
        if (a > lower) lower = a;
        if (b < upper) upper = b;
        if (lower > upper) return false;
        pattern = (uint16_t)((unsigned int)pattern << 1U);
        if (wide) pattern |= 1U;
    }
    *p_pattern = pattern;
    *p_bias100 = (lower <= 0 && upper >= 0) ? 0 : (lower + upper) / 2;
    return true;
}

static s3_barcode_status_t decode_forward(const uint32_t *p_runs,
                                           s3_symbol_t *p_symbol)
{
    uint16_t pattern[3] = {0};
    uint32_t unit[3] = {0};
    int64_t bias100[3] = {0};
    size_t i;

    for (i = 0U; i < 3U; ++i)
    {
        if (!character(&p_runs[i * 10U], &pattern[i], &unit[i], &bias100[i]))
        {
            return S3_BARCODE_BAD_PATTERN;
        }
    }
    if ((pattern[0] != S3_FRAME_PATTERN) || (pattern[2] != S3_FRAME_PATTERN))
    {
        return S3_BARCODE_BAD_PATTERN;
    }
    for (i = 0U; i < 2U; ++i)
    {
        uint32_t adjacent = (uint32_t)(((uint64_t)unit[i] + unit[i + 1U]) / 2U);
        int64_t separator100 = (int64_t)p_runs[9U + (i * 10U)] * 100 -
                               (bias100[i] + bias100[i + 1U]) / 2;
        /* Separators are white and use the adjacent fitted edge offset.
         * Local character widths permit gradual speed
         * changes; reject more than 2:1 change between adjacent characters. */
        if (((uint64_t)unit[i] > ((uint64_t)unit[i + 1U] * 2U)) ||
            ((uint64_t)unit[i + 1U] > ((uint64_t)unit[i] * 2U)) ||
            (separator100 < (int64_t)adjacent * 75) ||
            (separator100 > (int64_t)adjacent * 125))
        {
            return S3_BARCODE_BAD_PATTERN;
        }
    }
    if (pattern[1] == S3_A_PATTERN)
    {
        p_symbol->symbol = 'A';
    }
    else if (pattern[1] == S3_Z_PATTERN)
    {
        p_symbol->symbol = 'Z';
    }
    else
    {
        return S3_BARCODE_UNKNOWN;
    }
    return S3_BARCODE_OK;
}

s3_barcode_status_t s3_barcode_decode(const uint32_t *p_runs, uint32_t count,
                                     s3_symbol_t *p_symbol)
{
    uint32_t reversed[S3_BARCODE_RUN_COUNT];
    s3_barcode_status_t forward;
    s3_barcode_status_t backward;
    size_t i;

    if (p_symbol == NULL)
    {
        return S3_BARCODE_BAD_PATTERN;
    }
    (void)memset(p_symbol, 0, sizeof(*p_symbol));
    if ((p_runs == NULL) || (count != S3_BARCODE_RUN_COUNT))
    {
        return S3_BARCODE_INCOMPLETE;
    }
    forward = decode_forward(p_runs, p_symbol);
    if (forward == S3_BARCODE_OK)
    {
        return forward;
    }
    for (i = 0U; i < S3_BARCODE_RUN_COUNT; ++i)
    {
        reversed[i] = p_runs[S3_BARCODE_RUN_COUNT - 1U - i];
    }
    backward = decode_forward(reversed, p_symbol);
    if (backward == S3_BARCODE_OK)
    {
        p_symbol->reverse = true;
        return backward;
    }
    return ((forward == S3_BARCODE_UNKNOWN) || (backward == S3_BARCODE_UNKNOWN)) ?
           S3_BARCODE_UNKNOWN : S3_BARCODE_BAD_PATTERN;
}

bool s3_barcode_config_valid(const s3_barcode_config_t *p_config)
{
    return (p_config != NULL) && (p_config->quiet_ms > 0U) &&
           (p_config->min_run_ms > 0U) &&
           (p_config->timeout_ms > p_config->quiet_ms) &&
           (p_config->timeout_ms <= INT32_MAX) &&
           ((uint64_t)p_config->min_run_ms * S3_BARCODE_RUN_COUNT <
            p_config->timeout_ms);
}

void s3_barcode_init(s3_barcode_t *p_barcode)
{
    if (p_barcode != NULL)
    {
        (void)memset(p_barcode, 0, sizeof(*p_barcode));
        p_barcode->state = S3_CAPTURE_WAIT_QUIET;
    }
}

void s3_barcode_abort(s3_barcode_t *p_barcode)
{
    if (p_barcode != NULL)
    {
        bool hold = (p_barcode->state == S3_CAPTURE_RUNNING) ||
                    (p_barcode->state == S3_CAPTURE_HOLD);
        s3_barcode_init(p_barcode);
        if (hold)
        {
            p_barcode->state = S3_CAPTURE_HOLD;
        }
    }
}

void s3_barcode_rearm(s3_barcode_t *p_barcode)
{
    if ((p_barcode != NULL) && (p_barcode->state == S3_CAPTURE_HOLD))
    {
        p_barcode->rearm_requested = true;
    }
}

static s3_barcode_status_t reject(s3_barcode_t *p_barcode,
                                   s3_barcode_status_t status)
{
    p_barcode->state = S3_CAPTURE_HOLD;
    p_barcode->rearm_requested = false;
    return status;
}

s3_barcode_status_t s3_barcode_feed(s3_barcode_t *p_barcode,
                                   const s3_barcode_config_t *p_config,
                                   bool black, uint32_t now_ms,
                                   s3_symbol_t *p_symbol)
{
    bool edge;
    uint32_t elapsed;
    uint64_t trailing;

    if (p_symbol == NULL)
    {
        return S3_BARCODE_BAD_PATTERN;
    }
    (void)memset(p_symbol, 0, sizeof(*p_symbol));
    if ((p_barcode == NULL) || !s3_barcode_config_valid(p_config))
    {
        return S3_BARCODE_BAD_PATTERN;
    }
    if (!p_barcode->have_sample)
    {
        p_barcode->have_sample = true;
        p_barcode->last_sample = now_ms;
        p_barcode->last_edge = now_ms;
        p_barcode->black = black;
        return S3_BARCODE_NONE;
    }
    elapsed = now_ms - p_barcode->last_sample;
    p_barcode->last_sample = now_ms;
    if ((elapsed == 0U) || (elapsed > INT32_MAX))
    {
        return reject(p_barcode, S3_BARCODE_NOISE);
    }
    edge = black != p_barcode->black;
    p_barcode->black = black;
    if (p_barcode->state != S3_CAPTURE_RUNNING)
    {
        if (edge)
        {
            p_barcode->last_edge = now_ms;
        }
        if (p_barcode->state == S3_CAPTURE_HOLD)
        {
            if (p_barcode->rearm_requested && !black &&
                ((now_ms - p_barcode->last_edge) >= p_config->quiet_ms))
            {
                p_barcode->state = S3_CAPTURE_READY;
                p_barcode->rearm_requested = false;
            }
        }
        else if ((p_barcode->state == S3_CAPTURE_WAIT_QUIET) && !black &&
                 ((now_ms - p_barcode->last_edge) >= p_config->quiet_ms))
        {
            p_barcode->state = S3_CAPTURE_READY;
        }
        else if ((p_barcode->state == S3_CAPTURE_READY) && edge && black)
        {
            p_barcode->state = S3_CAPTURE_RUNNING;
            p_barcode->started = now_ms;
            p_barcode->last_edge = now_ms;
            p_barcode->count = 0U;
            p_barcode->max_run = 0U;
        }
        return S3_BARCODE_NONE;
    }
    if ((now_ms - p_barcode->started) >= p_config->timeout_ms)
    {
        return reject(p_barcode, S3_BARCODE_TIMEOUT);
    }
    elapsed = now_ms - p_barcode->last_edge;
    if (edge)
    {
        p_barcode->last_edge = now_ms;
        if (elapsed < p_config->min_run_ms)
        {
            return reject(p_barcode, S3_BARCODE_NOISE);
        }
        if (p_barcode->count >= S3_BARCODE_RUN_COUNT)
        {
            return reject(p_barcode, S3_BARCODE_OVERFLOW);
        }
        p_barcode->runs[p_barcode->count++] = elapsed;
        if (elapsed > p_barcode->max_run)
        {
            p_barcode->max_run = elapsed;
        }
        return S3_BARCODE_NONE;
    }
    trailing = (uint64_t)p_barcode->max_run * 4U;
    if (trailing < p_config->quiet_ms)
    {
        trailing = p_config->quiet_ms;
    }
    if (!black && ((uint64_t)elapsed >= trailing))
    {
        s3_barcode_status_t status = s3_barcode_decode(p_barcode->runs,
                                                      p_barcode->count,
                                                      p_symbol);
        /* Every finished physical marker, including invalid ones, latches. */
        return reject(p_barcode, status);
    }
    return S3_BARCODE_NONE;
}
