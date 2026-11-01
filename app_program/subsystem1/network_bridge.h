/* No kernel/SDK includes: safe for the lwIP owner translation unit. */
#ifndef CSC02_NETWORK_BRIDGE_H
#define CSC02_NETWORK_BRIDGE_H
#include "telemetry_edr.h"
#define CSC02_TELEMETRY_TOPIC EDR_MQTT_TOPIC
#define CSC02_COMMAND_TOPIC "csc02/command"
#define CSC02_COMMAND_BYTES 6U
int s1_telemetry_take(unsigned char *out); /* explicit length, including NUL bytes */
int s1_telemetry_current(const unsigned char *data);
int s1_command_offer(const unsigned char *data,unsigned length);
#endif
