#ifndef S4_PACKED_BITS_H
#define S4_PACKED_BITS_H
#include <stdint.h>
/* Private bounded-storage helpers; callers supply validated offsets/widths.
 * LSB-first within bytes, independent of C bitfield layout and host endian. */
static inline uint64_t s4_bits_get(const uint8_t *p, unsigned bit, unsigned width)
{
    uint64_t value = 0;
    unsigned shift = 0;
    while (width) {
        unsigned start = bit % 8U, take = 8U - start;
        if (take > width) take = width;
        unsigned mask = (1U << take) - 1U;
        value |= (uint64_t)((p[bit / 8U] >> start) & mask) << shift;
        bit += take; shift += take; width -= take;
    }
    return value;
}
static inline void s4_bits_put(uint8_t *p, unsigned bit, unsigned width, uint64_t value)
{
    while (width) {
        unsigned start = bit % 8U, take = 8U - start;
        if (take > width) take = width;
        unsigned mask = (1U << take) - 1U;
        unsigned old = p[bit / 8U];
        p[bit / 8U] = (uint8_t)((old & ~(mask << start)) |
                                (((unsigned)value & mask) << start));
        value >>= take; bit += take; width -= take;
    }
}
#endif
