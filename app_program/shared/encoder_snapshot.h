#ifndef ENCODER_SNAPSHOT_H
#define ENCODER_SNAPSHOT_H
#include "edr.h"
#define ENCODER_SNAPSHOT_SIZE 13U
#define ENCODER_SNAPSHOT_VALID 1U
typedef struct { uint32_t left_counts, right_counts, sampled_ms; uint8_t flags; } encoder_snapshot_t;
static inline bool encoder_snapshot_encode(const encoder_snapshot_t *s, uint8_t *out, size_t cap)
{
    if (!s || !out || cap < ENCODER_SNAPSHOT_SIZE || (s->flags & ~1U) ||
        (!s->flags && (s->left_counts || s->right_counts || s->sampled_ms))) return false;
    const uint32_t v[3] = {s->left_counts, s->right_counts, s->sampled_ms};
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<4; ++j)
        out[i*4U+j]=(uint8_t)(v[i]>>(j*8U));
    out[12]=s->flags; return true;
}
static inline bool encoder_snapshot_decode(const uint8_t *in, size_t size, encoder_snapshot_t *out)
{
    if (!in || !out || size != ENCODER_SNAPSHOT_SIZE || (in[12] & ~1U)) return false;
    uint32_t v[3]={0};
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<4; ++j)
        v[i]|=(uint32_t)in[i*4U+j]<<(j*8U);
    if (!in[12] && (v[0] || v[1] || v[2])) return false;
    *out=(encoder_snapshot_t){v[0],v[1],v[2],in[12]}; return true;
}
#endif
