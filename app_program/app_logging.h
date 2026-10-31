#ifndef APP_LOGGING_H
#define APP_LOGGING_H
#include "app_state.h"
/* Telemetry task only, after a coherent application-state copy. No network calls. */
void app_log_state(const app_state_t *state);
#endif
