/* Watchdog supervisor and boot reset report (app_watchdog.c). */
#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H
#include "app.h"

/* First thing at boot: read and clear the crash record, read why the chip
 * reset, print the first "[RESET] ..." line. TRUE: the previous runs ended
 * in HEALTH_HOLD_AFTER early crashes in a row; call app_boot_hold_loop()
 * instead of starting anything (the network task already waits). */
IMPORT BOOL app_reset_capture(void);
/* Never returns: repeats the "[RESET]" line every 2 s with nothing started. */
IMPORT void app_boot_hold_loop(void);
/* After every task has started: create and start the 100 ms supervisor
 * cyclic handler (core 1). E_OK and nothing started if BOARD_WATCHDOG 0. */
IMPORT ER app_watchdog_start(void);
/* usermain's idle loop: repeats the "[RESET]" line every 10 s for 60 s and
 * reports when the watchdog arms (or why not). Never returns. */
IMPORT void app_watchdog_report_loop(void);
#endif
