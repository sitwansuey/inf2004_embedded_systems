/* No kernel/SDK includes: safe for the lwIP owner translation unit. */
#ifndef CSC02_NETWORK_BRIDGE_H
#define CSC02_NETWORK_BRIDGE_H
#include "telemetry_edr.h"
#include "../shared/events.h"
#define CSC02_TELEMETRY_TOPIC EDR_MQTT_TOPIC
#define CSC02_COMMAND_TOPIC "csc02/command"
#define CSC02_COMMAND_BYTES 6U
#define CSC02_ACK_TOPIC "csc02/command/ack"
#define CSC02_ACK_BYTES 6U
/* Sequence bit 7 marks a USB command (usb_control.c); the dashboard uses 0..127. */
#define CSC02_USB_SEQUENCE 0x80U
#define CSC02_EVENT_TOPIC EVENT_MQTT_TOPIC
int s1_event_take(unsigned char *out); /* one two-byte event record */
int s1_command_ack_take(unsigned char *out);
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault);
/* An ACK the VC did not make (queue full, RESET needed): phase and fault from
 * the last published VC state. */
void s1_command_reply(unsigned seq,unsigned opcode,unsigned result);
int s1_telemetry_take(unsigned char *out); /* explicit length, including NUL bytes */
void s1_telemetry_dropped(void); /* a snapshot abandoned part-way: count it */
int s1_telemetry_current(const unsigned char *data);
int s1_command_offer(const unsigned char *data,unsigned length);
#endif
