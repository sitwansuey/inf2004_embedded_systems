/* Stateless steering map for the confirmed GP26/27 straddling geometry.
 * No sensor access, task, barcode decoder or movement authorization. */
#ifndef VEHICLE_LINE_H
#define VEHICLE_LINE_H
#include <stdint.h>
/* Mask bit 0 is RIGHT, bit 1 LEFT. Both white/black map forward only for
 * explicitly supervised continuous tests: white cannot prove line presence.
 * Returns S2 command codes (0 STOP, 1 FORWARD, 5 RIGHT, 6 LEFT). */
uint8_t vehicle_line_command(uint8_t black_mask);
#endif
