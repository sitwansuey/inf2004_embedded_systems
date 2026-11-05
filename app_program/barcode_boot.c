#include "barcode_boot.h"
#include <stddef.h>
void barcode_boot_cancel(barcode_boot_t *s) {
    if(s){s->consumed=true;s->waiting=false;}
}
bool barcode_boot_step(barcode_boot_t *s,uint32_t now,bool mapped,bool ready) {
    if(!s || s->consumed)return false;
    if(!mapped || !ready){s->waiting=false;return false;}
    if(!s->waiting || now-s->last_ms>=30U){s->since_ms=now;s->waiting=true;}
    s->last_ms=now;
    if(now-s->since_ms<BARCODE_BOOT_DELAY_MS)return false;
    s->consumed=true;s->waiting=false;return true;
}
