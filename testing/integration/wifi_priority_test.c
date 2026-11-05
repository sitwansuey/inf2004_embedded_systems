#include "host.h"
#include "../../app_program/subsystem3/subsystem3.h"
#include "../../app_program/subsystem3/drv_ir_adc.h"
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"
#include <assert.h>
#include <stdio.h>
static unsigned started;
static bool fail_create;
void cyw43_utk_task(INT code,void *context){(void)code;(void)context;}
ID tk_cre_tsk(const T_CTSK *task) {
#if BOARD_BARCODE_DEMO
    assert(task->itskpri==18 && task->itskpri>S3_PRI_CONSUMER && task->itskpri>S3_ADC_WORKER_PRI);
#else
    assert(task->itskpri==4);
#endif
    assert(task->assprc==TP_PRC1 && task->task==(FP)cyw43_utk_task);
    assert(task->stksz==8*1024);
    return fail_create?E_SYS:7;
}
ER tk_sta_tsk(ID id,INT code){assert(id==7 && code==0);++started;return E_OK;}
int main(void) {
    assert(cyw43_utk_start()==E_OK && started==1);
    fail_create=true;assert(cyw43_utk_start()==E_SYS && started==1);
    puts("Radio task ownership/startup and profile-specific ADC-over-WiFi priority PASS");
    return 0;
}
