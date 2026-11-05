#include "../../app_program/subsystem1/barcode_diagnostic.h"
#include <stdio.h>
int main(void) {
    barcode_diagnostic_t d={.kind=2,.status=6,.sequence=UINT32_MAX,.time_ms=5000,
        .result_id=3,.result_ms=4990,.started_ms=1000,.rejected=2,.decoded=1,
        .max_gap_ms=20,.run_count=29,.max_run_ms=120,.sample_ms=4990,
        .acquisition_error=-50,.left_counts=-123,.right_counts=456,
        .phase=1,.power=30,.navigation=2,.symbol='Z',.flags=51,
        .capture_state=3,.capture_runs=29,.line_mask=2,.stored_symbol=26,.valid_mask=7,.algorithm_error=3};
    for(unsigned n=0;n<29;++n)d.runs[n]=n%2?120:40;
    uint8_t wire[BARCODE_DIAGNOSTIC_BYTES];barcode_diag_pack(wire,&d);
    for(unsigned n=0;n<sizeof(wire);++n)printf("%02x",wire[n]);
    puts("");return 0;
}
