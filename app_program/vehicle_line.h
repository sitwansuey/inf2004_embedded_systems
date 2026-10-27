/* IR line following and barcode reading for the vehicle task. */
#ifndef VEHICLE_LINE_H
#define VEHICLE_LINE_H
#include "app.h"
#include "subsystem2/subsystem2.h"

/* Configure Buddy 3 for the calibrated ADC line and barcode sensors.
 * Call after subsystem3_init() and before subsystem3_start(). */
IMPORT ER vehicle_line_init(void);
/* One vehicle period: newest IR scan -> line-following command; also feeds
 * the barcode decoder. STOP until the start delay passes, without usable IR
 * readings, or after VEHICLE_LINE_MAX_MS when that limit is set. */
IMPORT void vehicle_line_step(s2_move_command_t *command, UINT *power);

#endif
