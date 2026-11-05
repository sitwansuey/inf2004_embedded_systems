#include "barcode_route.h"
#include <stddef.h>
#include <string.h>

bool barcode_map_valid(const uint8_t letters[BARCODE_MAP_BYTES]) {
    if (!letters) return false;
    for (unsigned n=0; n<BARCODE_MAP_BYTES; ++n) {
        if (letters[n]<'A' || letters[n]>'Z') return false;
        for (unsigned j=0; j<n; ++j) if (letters[n]==letters[j]) return false;
    }
    return true;
}
static void whitespace(const uint8_t *s,unsigned len,unsigned *i) {
    while (*i<len && (s[*i]==' ' || s[*i]=='\t' || s[*i]=='\r' || s[*i]=='\n')) ++*i;
}
static bool token(const uint8_t *s,unsigned len,unsigned *i,uint8_t expected) {
    whitespace(s,len,i);
    if (*i>=len || s[*i]!=expected) return false;
    ++*i;return true;
}
bool barcode_map_parse(const uint8_t *s,unsigned len,uint8_t letters[BARCODE_MAP_BYTES]) {
    const char *keys[4]={"forward","uturn","left","right"};
    uint8_t result[4]={0};unsigned i=0,seen=0;
    if (!s || !letters || !len || len>BARCODE_MAP_JSON_MAX || !token(s,len,&i,'{')) return false;
    for (unsigned n=0;n<4;++n) {
        if (n && !token(s,len,&i,',')) return false;
        if (!token(s,len,&i,'"')) return false;
        unsigned start=i;
        while (i<len && s[i]!='"') ++i;
        if (i==len) return false;
        unsigned key=4;
        for (unsigned k=0;k<4;++k)
            if (i-start==strlen(keys[k]) && !memcmp(s+start,keys[k],i-start)) key=k;
        if (key==4 || (seen&(1U<<key))) return false;
        ++i;seen|=1U<<key;
        if (!token(s,len,&i,':') || !token(s,len,&i,'"') || i>=len) return false;
        result[key]=s[i++];
        /* No whitespace inside a single-character value. */
        if (i>=len || s[i++]!='"') return false;
    }
    if (!token(s,len,&i,'}')) return false;
    whitespace(s,len,&i);
    if (i!=len || seen!=15U || !barcode_map_valid(result)) return false;
    memcpy(letters,result,sizeof(result));return true;
}
unsigned barcode_route_configure(barcode_route_t *r,const uint8_t letters[BARCODE_MAP_BYTES],bool armed) {
    if (!r || !barcode_map_valid(letters)) return BARCODE_MAP_INVALID;
    if (armed) return BARCODE_MAP_BUSY;
    memcpy(r->letters,letters,BARCODE_MAP_BYTES);r->configured=true;
    barcode_route_clear(r);return BARCODE_MAP_OK;
}
void barcode_route_clear(barcode_route_t *r) {
    if (!r) return;
    /* Keep the received mapping across STOP/RESET, clear the pending marker. */
    r->pending=0;r->fault=false;r->timing=false;r->junction_latched=false;
}
bool barcode_route_store(barcode_route_t *r,char symbol) {
    if (!r || symbol<'A' || symbol>'Z' || r->pending) return false;
    r->pending=symbol;return true;
}
void barcode_route_observe_capture(barcode_route_t *r,bool fresh,unsigned mask,
                                  bool scanning,uint32_t result_id,unsigned result_mask) {
    if(!r || !fresh || mask>3U)return;
    if(scanning && mask==3U)r->junction_latched=true;
    if(result_id!=r->capture_result_id) {
        if(result_id && result_mask==3U)r->junction_latched=true;
        r->capture_result_id=result_id;
    }
}
s3_navigation_t barcode_route_step(barcode_route_t *r,uint32_t now,bool fresh,unsigned mask,bool tracking) {
    if (!r) return S3_NAV_NONE;
    if (!fresh || !tracking || mask>3U) { r->timing=false;return S3_NAV_NONE; }
    bool black=mask==3U;
    /* A stale/missing observation cannot help confirm a junction or clearance. */
    if (!r->timing || now-r->last_ms>=30U || black!=r->candidate_black) {
        r->since_ms=now;r->candidate_black=black;r->timing=true;
    }
    r->last_ms=now;
    if (now-r->since_ms<BARCODE_JUNCTION_MS) return S3_NAV_NONE;
    if (!black) {r->junction_latched=false;return S3_NAV_NONE;}
    if (r->junction_latched) return S3_NAV_NONE;
    r->junction_latched=true;
    if (!r->pending) return r->default_forward?S3_NAV_STRAIGHT:S3_NAV_NONE;
    const s3_navigation_t actions[4]={S3_NAV_STRAIGHT,S3_NAV_UTURN,S3_NAV_LEFT,S3_NAV_RIGHT};
    if (r->configured) for (unsigned n=0;n<4;++n) if (r->letters[n]==(uint8_t)r->pending) {
        r->pending=0;return actions[n];
    }
    if (r->default_forward) {r->pending=0;return S3_NAV_STRAIGHT;}
    r->fault=true;return S3_NAV_NONE; /* Strict profiles require a mapped character. */
}
