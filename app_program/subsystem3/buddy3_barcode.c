#include "buddy3_barcode.h"

#include <string.h>

/* Framed single-character Code 39: *X*, X in A..Z.
 * Nine elements per character, 3 wide elements, narrow:wide = 1:3. */
#define S3_FRAME_PATTERN 0x094U
/* Code 39 A..Z, nine elements MSB first (1=wide). Verified against ZXing
 * Code39Reader CHARACTER_ENCODINGS; these are symbol encodings, not routes.
 * https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java */
static const uint16_t alphabet[26] = {
    0x109U,0x049U,0x148U,0x019U,0x118U,0x058U,0x00DU,0x10CU,0x04CU,0x01CU,
    0x103U,0x043U,0x142U,0x013U,0x112U,0x052U,0x007U,0x106U,0x046U,0x016U,
    0x181U,0x0C1U,0x1C0U,0x091U,0x190U,0x0D0U
};

/* A reflective threshold can shorten black pulses and lengthen white gaps.
 * Select three wide elements and intersect the original 1:3 +/-25% bands
 * with one bounded black/white edge offset before checking separation. No
 * symbol is used to choose widths. An offset above half a narrow unit fails.
 * Percent-scaled signed 64-bit arithmetic preserves fractional bounds and
 * remains bounded for every uint32_t input. No allocation or persistent state. */
static bool character_bounds(const uint32_t *p_runs, uint16_t *p_pattern,
                       uint32_t *p_unit, int64_t *p_lower100, int64_t *p_upper100)
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
    /* A-Z and the framing star have three narrow black and three narrow
     * white elements. Their mean cancels a constant edge displacement. */
    uint64_t narrow_sum = 0U;
    for (i = 0U; i < 6U; ++i) narrow_sum += sorted[i];
    *p_unit = (uint32_t)((narrow_sum + 3U) / 6U);
    if ((sorted[0] == 0U) || (*p_unit == 0U) || sorted[5] >= sorted[6])
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
    *p_lower100 = lower;
    *p_upper100 = upper;
    return true;
}

static int64_t nearest_zero(int64_t lower, int64_t upper)
{
    return lower > 0 ? lower : upper < 0 ? upper : 0;
}

static bool separated(const uint32_t *runs, uint16_t pattern, int64_t bias100)
{
    int64_t smallest_wide = INT64_MAX, largest_narrow = 0;
    for (unsigned i = 0U; i < 9U; ++i) {
        int64_t value = (int64_t)runs[i] * 100 + ((i & 1U) ? -bias100 : bias100);
        if (pattern & (1U << (8U-i))) {
            if (value < smallest_wide) smallest_wide = value;
        } else if (value > largest_narrow) largest_narrow = value;
    }
    return smallest_wide * 2 >= largest_narrow * 3;
}

/* Offline inspectors use one representative fit; decoding below also fits
 * the two separators before selecting the three compensation values. */
static inline bool character(const uint32_t *runs, uint16_t *pattern,
                              uint32_t *unit, int64_t *bias100)
{
    int64_t lower,upper;
    if (!character_bounds(runs,pattern,unit,&lower,&upper)) return false;
    *bias100 = nearest_zero(lower,upper);
    return separated(runs,*pattern,*bias100);
}

static s3_barcode_status_t frame_fit(const uint32_t *p_runs, uint16_t pattern[3],
                                    uint32_t unit[3], int64_t bias100[3])
{
    int64_t lower100[3] = {0}, upper100[3] = {0};
    int64_t sum_lower[2], sum_upper[2];
    size_t i;

    for (i = 0U; i < 3U; ++i)
    {
        if (!character_bounds(&p_runs[i * 10U], &pattern[i], &unit[i],
                              &lower100[i], &upper100[i]))
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
        int64_t raw100 = (int64_t)p_runs[9U + (i * 10U)] * 100;
        /* Separators are white and use the adjacent fitted edge offset.
         * Local character widths permit gradual speed
         * changes; reject more than 2:1 change between adjacent characters. */
        if (((uint64_t)unit[i] > ((uint64_t)unit[i + 1U] * 2U)) ||
            ((uint64_t)unit[i + 1U] > ((uint64_t)unit[i] * 2U)))
        {
            return S3_BARCODE_BAD_PATTERN;
        }
        sum_lower[i] = 2 * (raw100 - (int64_t)adjacent * 125);
        sum_upper[i] = 2 * (raw100 - (int64_t)adjacent * 75);
    }
    /* Solve the two adjacent-sum constraints on this three-character chain.
     * No search over symbols, relaxed tolerances or independent gap offsets. */
    int64_t middle_lower = lower100[1], middle_upper = upper100[1];
    for (i = 0U; i < 2U; ++i) {
        size_t outer = i == 0U ? 0U : 2U;
        int64_t lo = sum_lower[i] - upper100[outer];
        int64_t hi = sum_upper[i] - lower100[outer];
        if (lo > middle_lower) middle_lower = lo;
        if (hi < middle_upper) middle_upper = hi;
    }
    if (middle_lower > middle_upper) return S3_BARCODE_BAD_PATTERN;
    bias100[1] = nearest_zero(middle_lower,middle_upper);
    for (i = 0U; i < 2U; ++i) {
        size_t outer = i == 0U ? 0U : 2U;
        int64_t lo = sum_lower[i] - bias100[1], hi = sum_upper[i] - bias100[1];
        if (lo < lower100[outer]) lo = lower100[outer];
        if (hi > upper100[outer]) hi = upper100[outer];
        if (lo > hi) return S3_BARCODE_BAD_PATTERN;
        bias100[outer] = nearest_zero(lo,hi);
    }
    for (i = 0U; i < 3U; ++i)
        if (!separated(p_runs+i*10U,pattern[i],bias100[i]))
            return S3_BARCODE_BAD_PATTERN;
    return S3_BARCODE_OK;
}

static s3_barcode_status_t decode_forward(const uint32_t *p_runs,
                                           s3_symbol_t *p_symbol)
{
    uint16_t pattern[3] = {0};uint32_t unit[3] = {0};int64_t bias100[3] = {0};
    s3_barcode_status_t status = frame_fit(p_runs,pattern,unit,bias100);
    if (status != S3_BARCODE_OK) return status;
    for (size_t i = 0U; i < 26U; ++i) {
        if (pattern[1] == alphabet[i]) {
            p_symbol->symbol = (char)('A' + (int)i);
            return S3_BARCODE_OK;
        }
    }
    return S3_BARCODE_UNKNOWN;
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

static s3_barcode_status_t feed(s3_barcode_t *p_barcode,
                                   const s3_barcode_config_t *p_config,
                                   bool black, uint32_t now_ms,
                                   s3_symbol_t *p_symbol, bool resync)
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
            p_barcode->resync_pairs = 0U;
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
        if ((p_config->min_run_ms != 0U) && (elapsed < p_config->min_run_ms))
        {
            return reject(p_barcode, S3_BARCODE_NOISE);
        }
        if (p_barcode->count >= S3_BARCODE_RUN_COUNT)
        {
            /* A full window ends black. The next completed run is white;
             * drop a black/white pair to keep the next candidate black-first.
             * Never restart the absolute scan deadline or grow the buffer. */
            if (!resync || p_barcode->count != S3_BARCODE_RUN_COUNT || !black ||
                p_barcode->resync_pairs >= S3_BARCODE_MAX_RESYNC_PAIRS)
                return reject(p_barcode, S3_BARCODE_OVERFLOW);
            (void)memmove(p_barcode->runs, p_barcode->runs + 2U,
                          (S3_BARCODE_RUN_COUNT - 2U) * sizeof(p_barcode->runs[0]));
            p_barcode->count -= 2U;
            ++p_barcode->resync_pairs;
            /* Discarded noise must not inflate the required trailing white. */
            p_barcode->max_run = 0U;
            for (uint32_t i = 0U; i < p_barcode->count; ++i)
                if (p_barcode->runs[i] > p_barcode->max_run)
                    p_barcode->max_run = p_barcode->runs[i];
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

s3_barcode_status_t s3_barcode_feed(s3_barcode_t *p_barcode,
                                   const s3_barcode_config_t *p_config,
                                   bool black, uint32_t now_ms,
                                   s3_symbol_t *p_symbol)
{
    return feed(p_barcode,p_config,black,now_ms,p_symbol,false);
}

s3_barcode_status_t s3_barcode_feed_resync(s3_barcode_t *p_barcode,
                                   const s3_barcode_config_t *p_config,
                                   bool black, uint32_t now_ms,
                                   s3_symbol_t *p_symbol)
{
    return feed(p_barcode,p_config,black,now_ms,p_symbol,true);
}
