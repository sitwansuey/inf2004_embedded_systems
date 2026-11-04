/* No kernel/SDK includes: safe for the lwIP owner translation unit. */
#ifndef CSC02_NETWORK_BRIDGE_H
#define CSC02_NETWORK_BRIDGE_H
#include "telemetry_edr.h"
#include "../subsystem4/motion.h"
#define CSC02_TELEMETRY_TOPIC EDR_MQTT_TOPIC
#define CSC02_COMMAND_TOPIC "csc02/command"
#define CSC02_COMMAND_BYTES 6U
#define CSC02_ACK_TOPIC "csc02/command/ack"
#define CSC02_ACK_BYTES 6U
int s1_command_ack_take(unsigned char *out);
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault);
int s1_telemetry_take(unsigned char *out); /* explicit length, including NUL bytes */
int s1_telemetry_current(const unsigned char *data);
int s1_command_offer(const unsigned char *data,unsigned length);
int s1_motion_send(const unsigned char *data);
int s1_motion_take(unsigned char *out);
int s1_motion_current(const unsigned char *data);
#endif
