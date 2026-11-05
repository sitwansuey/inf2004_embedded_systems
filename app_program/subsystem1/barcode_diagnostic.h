/* Optional debug channel. Primary telemetry remains individual EDR16 records.
 * Explicit little-endian framing: no native struct crosses MQTT. */
#ifndef CSC02_BARCODE_DIAGNOSTIC_H
#define CSC02_BARCODE_DIAGNOSTIC_H
#include <stdint.h>
#include <string.h>
#define BARCODE_DIAGNOSTIC_BYTES 192U
#define BARCODE_DIAGNOSTIC_RUNS 29U
#define CSC02_BARCODE_DIAGNOSTIC_TOPIC "Robopico/diagnostics/barcode/v1"
typedef struct {
    uint32_t sequence,time_ms,result_id,result_ms,started_ms,rejected,decoded;
    uint32_t max_gap_ms,run_count,max_run_ms,dropped,sample_ms;
    int32_t acquisition_error,left_counts,right_counts;
    uint8_t kind,status,phase,fault,power,navigation,symbol,flags,capture_state;
    uint8_t capture_runs,line_mask,stored_symbol,valid_mask,algorithm_error;
    uint32_t runs[BARCODE_DIAGNOSTIC_RUNS];
} barcode_diagnostic_t;
static inline void barcode_diag_put32(uint8_t *p,uint32_t v) {
    for(unsigned n=0;n<4;++n)p[n]=(uint8_t)(v>>(8U*n));
}
static inline void barcode_diag_pack(uint8_t out[BARCODE_DIAGNOSTIC_BYTES],const barcode_diagnostic_t *d) {
    memset(out,0,BARCODE_DIAGNOSTIC_BYTES);
    out[0]=0xBC;out[1]=1;out[2]=d->kind;out[3]=d->status;
    const uint32_t fields[]={d->sequence,d->time_ms,d->result_id,d->result_ms,d->started_ms,
        d->rejected,d->decoded,d->max_gap_ms,d->run_count,d->max_run_ms,d->dropped,d->sample_ms,
        (uint32_t)d->acquisition_error,(uint32_t)d->left_counts,(uint32_t)d->right_counts};
    for(unsigned n=0;n<15;++n)barcode_diag_put32(out+4U+n*4U,fields[n]);
    out[64]=d->phase;out[65]=d->fault;out[66]=d->power;out[67]=d->navigation;
    out[68]=d->symbol;out[69]=d->flags;out[70]=d->capture_state;out[71]=d->capture_runs;
    out[72]=d->line_mask;out[73]=d->stored_symbol;out[74]=d->valid_mask;
    out[75]=d->algorithm_error;
    for(unsigned n=0;n<BARCODE_DIAGNOSTIC_RUNS;++n)barcode_diag_put32(out+76U+n*4U,d->runs[n]);
}
#endif
