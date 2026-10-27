# CR1 application data contract

27 October Buddy 4 integration retains this aggregate schema unchanged. The
incoming 13-byte EDR encoder payload (15 bytes with its separate envelope) is
not this packet. See `BUDDY4_IMPLEMENTATION.md` and `BUDDY4_CR1_STORAGE.md`.
Terrain availability now requires controller state at most 60 ms old and a
usable accelerometer snapshot; retained maximum is independent of that flag.

Only Change Requirement 1 is implemented. Requirements 2 and 3 are outside this
change. The schema carries 37 fields in 394 bits, rounded to 50 bytes. Bit
number zero is the least significant bit of byte zero. Every field stores
`value - minimum`; decode by adding the minimum. The version envelope occupies
four bits; encoded zero denotes schema version 1. Other version values and all
six trailing padding bits are rejected. This is an application payload, not a
dump of C structures, and is independent of compiler enum sizes, alignment and
host endianness.

The scalar widths are `ceil(log2(maximum-minimum+1))`, except the four-bit
version envelope. Width's 500 mm range safely covers the implemented bounds
geometry: depth <=200 mm and each edge <50 degrees give a width below 477 mm.
Do not expand a domain or change a scale without a new version and both codecs.

| Field | Bit offset | Bits | Inclusive range |
|---|---:|---:|---|
| version | 0 | 4 | 1..1 |
| sequence | 4 | 16 | 0..65535 |
| time_ms | 20 | 32 | 0..4294967295 |
| phase | 52 | 4 | 0..12 |
| fault | 56 | 3 | 0..7 |
| command | 59 | 3 | 0..6 |
| power | 62 | 7 | 0..100 |
| motor_state | 69 | 2 | 0..2 |
| available | 71 | 8 | 0..255 |
| left_counts | 79 | 32 | -2147483648..2147483647 |
| right_counts | 111 | 32 | -2147483648..2147483647 |
| speed_mm_s | 143 | 12 | -2000..2000 |
| line_state | 155 | 3 | 0..4 |
| ir0 | 158 | 12 | 0..4095 |
| ir1 | 170 | 12 | 0..4095 |
| ir2 | 182 | 12 | 0..4095 |
| black_mask | 194 | 3 | 0..7 |
| line_error | 197 | 11 | -1000..1000 |
| barcode | 208 | 2 | 0..2 |
| navigation | 210 | 3 | 0..4 |
| ax_mg | 213 | 12 | -2048..2047 |
| ay_mg | 225 | 12 | -2048..2047 |
| az_mg | 237 | 12 | -2048..2047 |
| mx_raw | 249 | 12 | -2048..2047 |
| my_raw | 261 | 12 | -2048..2047 |
| mz_raw | 273 | 12 | -2048..2047 |
| pitch_cdeg | 285 | 15 | -9000..9000 |
| roll_cdeg | 300 | 16 | -18000..18000 |
| terrain | 316 | 3 | 0..5 |
| height_mm | 319 | 10 | 0..1000 |
| highest_mm | 329 | 10 | 0..1000 |
| distance_mm | 339 | 13 | 0..5000 |
| width_mm | 352 | 9 | 0..500 |
| servo_deg | 361 | 8 | -90..90 |
| scan_state | 369 | 4 | 0..14 |
| quality | 373 | 5 | 0..31 |
| dropped | 378 | 16 | 0..65535 |

The canonical machine-readable definition is `tools/telemetry_schema.json`.
`telemetry_codec.c` validates all values before changing the caller's output.
The Python dashboard uses an independent integer implementation with the same
schema. `testing/integration/protocol_golden.json` supplies fixed boundary
packets. The sequence is modulo 65536 and is a display identifier, not a safety
clock. Millisecond clocks and encoder counts remain 32 bits, with modular
subtraction; public driver diagnostics and hardware IRQ counters retain their
original ranges. Height integration and geometry use wide intermediates.

`available` bits 0..7 mean motor status, encoder counts, calibrated IR sample,
fresh accelerometer, valid ultrasonic echo, valid tilt, completed hump height,
and fresh magnetometer. Copy success alone is not measurement validity.
`quality` bits 0..4 mean completed scan width, shock, armed, bypass validated,
and calibrated odometry. Width is the last completed scan estimate. A positive
highest height is the retained maximum of completed humps in the current run;
RESET starts a new run. Drop count is a saturating diagnostic, not a transport
delivery guarantee. Concurrent producers can make it a lower bound.

Acceleration is in mg; magnetometer fields are raw signed LSBs. Pitch/roll are
centidegrees, speed mm/s and dimensions mm. Line error is logical sensor-layout
units, not a measured millimetre displacement. Unavailable fields must not be
displayed as valid zero measurements. GY-511 has no gyroscope; absolute heading,
gyroscope rates and claimed measurement accuracy are not provided.

S2 stores command/power/state in 3/7/2-bit fields. The RTOS mailbox header and
STOP epoch remain naturally aligned. S3's discrete event wire format is exactly
13 bytes: byte 0 has kind(3), navigation(3), symbol(2; none/A/Z), followed by
sequence, timestamp and controller generation as three little-endian uint32s.
Consumers continue to use decoded snapshots under the existing locks. S5 groups
seven Boolean flags and narrows angles/ranges; its C layout is never sent as a
wire packet. ARM size probes are recorded separately; native enum sizes differ.

## MQTT and command behavior

Publish binary snapshots to `csc02/telemetry` at 5 Hz, with fault changes offered
on the next telemetry tick. QoS 0 is latest-state reporting, not a lossless event
log. A two-entry copied buffer feeds only the existing network-owning task.
The owner drains a bounded budget, replaces old pending data with the latest
sample, and drops frames older than 500 ms. No strlen or textual conversion is
used for binary payloads. Ordinary application tasks never call MQTT/lwIP.

Commands use `csc02/command`, six bytes: `C1 opcode power lease_lo lease_hi seq`.
Opcode 0 is STOP, 1 is ARM/renew line tracking, and 2 is RESET while stopped.
ARM accepts power 1..50 and a 1..1000 ms lease. STOP bypasses a full queue and
invalidates older queued ARM requests. Queued commands expire after 100 ms.
RESET/ARM after a network reconnect requires a fresh RESET before ARM; link loss
offers STOP. Do not retain commands. The sequence byte is reserved for operator
correlation; it is not an authentication or replay-protection mechanism.

USB supports the same binary frames, or `s` STOP, `r` RESET and repeated `a`
500 ms ARM renewals at 30 percent. Use one command transport per test; a loss of
one transport must not be masked by another sender continuing to renew ARM.
No motion begins merely from boot, MQTT reconnection or telemetry traffic.
