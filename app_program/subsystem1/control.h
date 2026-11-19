#ifndef CSC02_CONTROL_H
#define CSC02_CONTROL_H
#include "../app.h"
#include "network_bridge.h"
typedef struct {
    UW time_ms, epoch;
    uint8_t wire[CSC02_COMMAND_BYTES];
} control_message_t;
ER s1_command_take(control_message_t *p);
ER s1_binary_send(const uint8_t data[TELEMETRY_BYTES]);
uint16_t s1_drop_count(void); /* internal drops modulo 65536 (wraps) */
#endif
