#ifndef S3_EVENT_CODEC_H
#define S3_EVENT_CODEC_H
#include "subsystem3.h"
#define S3_EVENT_BYTES 14U
static inline void s3_u32_store(uint8_t *p, uint32_t n) {
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(n>>(8U*i));
}
static inline uint32_t s3_u32_load(const uint8_t *p) {
    uint32_t n=0; for (unsigned i=0; i<4; ++i) n|=(uint32_t)p[i]<<(8U*i); return n;
}
static inline bool s3_event_pack(uint8_t p[S3_EVENT_BYTES], const s3_event_record_t *r) {
    if (!p || !r) return false;
    unsigned k=(unsigned)r->event.kind,c=(unsigned)r->event.command;
    unsigned s=(r->event.symbol>='A' && r->event.symbol<='Z')?
        (unsigned)(r->event.symbol-'A')+1U:0U;
    if (k>5U || c>4U || (r->event.symbol!=0 && s==0U)) return false;
    p[0]=(uint8_t)(k | (c<<3));p[1]=(uint8_t)s;
    s3_u32_store(p+2,r->event.sequence); s3_u32_store(p+6,r->event.time_ms);
    s3_u32_store(p+10,r->generation); return true;
}
static inline bool s3_event_unpack(const uint8_t p[S3_EVENT_BYTES], s3_event_record_t *r) {
    if (!p || !r) return false;
    unsigned k=p[0]&7U,c=(p[0]>>3)&7U,s=p[1];
    if (k>5U || c>4U || s>26U || (p[0]&0xc0U)) return false;
    *r=(s3_event_record_t){0}; r->event.kind=(uint8_t)(k & 7U);
    r->event.command=(uint8_t)(c & 7U); r->event.symbol=s>=1U && s<=26U?(char)('A'+s-1U):0;
    r->event.sequence=s3_u32_load(p+2); r->event.time_ms=s3_u32_load(p+6);
    r->generation=s3_u32_load(p+10); return true;
}
#endif
