#ifndef APP_LOGGING_H
#define APP_LOGGING_H
#include "app_state.h"
/* Main calls after publication. One row per new sensor report or state change. */
IMPORT void app_log_state(const app_shared_state_t *state);
#endif
