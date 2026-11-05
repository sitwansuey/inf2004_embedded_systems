#ifndef CSC02_BARCODE_BOOT_H
#define CSC02_BARCODE_BOOT_H
#include <stdbool.h>
#include <stdint.h>
#define BARCODE_BOOT_DELAY_MS 3000U
typedef struct { uint32_t since_ms,last_ms;bool waiting,consumed; } barcode_boot_t;
/* Once per boot, after a received map and three seconds of ready observations.
 * STOP/fault cancellation is permanent for this boot, including retained maps. */
bool barcode_boot_step(barcode_boot_t *s,uint32_t now,bool mapped,bool ready);
void barcode_boot_cancel(barcode_boot_t *s);
#endif
