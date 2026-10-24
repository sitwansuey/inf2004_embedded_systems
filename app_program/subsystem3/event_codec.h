#ifndef S3_EVENT_CODEC_H
#define S3_EVENT_CODEC_H
#include "subsystem3.h"
#define S3_EVENT_BYTES 13U
static inline void s3_u32_store(uint8_t *p, uint32_t n) {
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(n>>(8U*i));
}
static inline uint32_t s3_u32_load(const uint8_t *p) {
    uint32_t n=0; for (unsigned i=0; i<4; ++i) n|=(uint32_t)p[i]<<(8U*i); return n;
}
static inline bool s3_event_pack(uint8_t p[S3_EVENT_BYTES], const s3_event_record_t *r) {
    unsigned k=(unsigned)r->event.kind,c=(unsigned)r->event.command;
    unsigned s=r->event.symbol=='A'?1U:r->event.symbol=='Z'?2U:0U;
    if (k>4U || c>4U || (r->event.symbol!=0 && s==0U)) return false;
    p[0]=(uint8_t)(k | (c<<3) | (s<<6));
    s3_u32_store(p+1,r->event.sequence); s3_u32_store(p+5,r->event.time_ms);
    s3_u32_store(p+9,r->generation); return true;
}
static inline bool s3_event_unpack(const uint8_t p[S3_EVENT_BYTES], s3_event_record_t *r) {
    unsigned k=p[0]&7U,c=(p[0]>>3)&7U,s=p[0]>>6;
    if (k>4U || c>4U || s>2U) return false;
    *r=(s3_event_record_t){0}; r->event.kind=(s3_event_kind_t)k;
    r->event.command=(s3_navigation_t)c; r->event.symbol=s==1U?'A':s==2U?'Z':0;
    r->event.sequence=s3_u32_load(p+1); r->event.time_ms=s3_u32_load(p+5);
    r->generation=s3_u32_load(p+9); return true;
}
#endif
