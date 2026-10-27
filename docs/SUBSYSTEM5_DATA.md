# Subsystem 5 bit representation

Subsystem 5 stores its authoritative shared status as 84 explicit bytes in
`status_data`, protected by the existing status mutex. `s5_status_t` remains
a decoded API/workspace view. The measured host ABI size is 136 bytes, so the
shared snapshot saves 52 bytes (38.2%). This is not a claim about total task
RAM: local decoded snapshots and codec scratch buffers still use stack space.

`s5_data_encode` and `s5_data_decode` implement the byte contract without C
bit fields, packed structures, allocation, or native structure serialization.
Encoding checks every bounded field before changing the output. Decoding
checks exact length, reserved bits, enum codes and angles before changing
the output. Input and output buffers must be separate. Invalid data is
rejected, never silently truncated. Internal publication of an invalid
snapshot stores an invalid `S5_FAULT` snapshot instead.

## Source 5, kind 0: obstacle snapshot

The EDR v1 frame is 86 bytes: header `45 54` in hexadecimal followed by the
84-byte payload. Offsets below are payload offsets. All multibyte integers
are little-endian. Bit 0 is the least significant bit.

| Offset | Bits / bytes | Field and range |
|---|---|---|
| 0 | bits 0..3 | State, existing `S5_MONITOR`..`S5_REORG` codes 0..14; 15 rejected |
| 0 | bits 4..6 | Echo result, existing `S5_ECHO_OK`..`S5_ECHO_TOO_SHORT` codes 0..4; 5..7 rejected |
| 0 | bit 7 | Reserved, must be zero |
| 1 | bits 0..6 | `acquisition_clear`, `approach_plan_valid`, `encoder_valid`, `valid`, `left_bound_valid`, `right_bound_valid`, `object_width_valid`, respectively |
| 1 | bit 7 | Reserved, must be zero |
| 2..7 | one byte each | `angle_deg`, `sample_angle_deg`, `avoidance_deg`, `planned_rotate_deg`, `left_bound_deg`, `right_bound_deg`; encode degrees + 90, range 0..180 |
| 8 | 4 bytes | `scan_id` |
| 12 | 4 bytes | `planned_rotate_ms` |
| 16 | 4 bytes | `planned_approach_mm` |
| 20 | 4 bytes | `planned_return_mm` |
| 24 | 4 bytes | `obstacle_distance_mm` |
| 28 | 4 bytes | `distance_mm` |
| 32 | 4 bytes | `echo_us` |
| 36 | 4 bytes | `time_ms` |
| 40 | 4 bytes | `motor1_counts_start` |
| 44 | 4 bytes | `motor2_counts_start` |
| 48 | 4 bytes | `motor1_counts_end` |
| 52 | 4 bytes | `motor2_counts_end` |
| 56 | 4 bytes | `encoder_start_ms` |
| 60 | 4 bytes | `encoder_end_ms` |
| 64 | 4 bytes | `left_bound_distance_mm` |
| 68 | 4 bytes | `right_bound_distance_mm` |
| 72 | 4 bytes | `object_width_mm` |
| 76 | 4 bytes | `object_width_min_mm` |
| 80 | 4 bytes | `object_width_max_mm` |

State needs 4 bits for 15 values, echo result needs 3 bits for five values,
and each Boolean needs one bit. Six signed degree values each need 8 bits
for the accepted -90..90 domain. Scan-only angles may have tighter bounds,
but navigation and the test-position API also support +/-90 degrees.

The nineteen 32-bit fields retain their existing domains. Encoder counts
use modulo-32 two's-complement bits and decode to signed INT32 values; no
count precision is lost. Other words retain the full unsigned 32-bit range.
Kernel timestamps and scan IDs must not be narrowed. Distance and duration
words include diagnostic/invalid readings and calculated geometry; this
change does not invent smaller bounds or saturate them. Width-valid and
range-valid checks remain the responsibility of the existing policy.

All-zero bytes represent angles of -90 degrees. A zero-initialized native
status instead encodes with six `5a` angle bytes. Initial shared state is
`S5_CENTERING` with zero-degree angles and all validity flags clear.
Invalid samples retain their diagnostics, timestamps and prior geometry;
consumers must check the corresponding validity bits, not infer validity
from nonzero values. Codec validity is not movement authorization.

## Use and ownership

Include `s5_data.h` and `../shared/edr.h`. Obtain a coherent snapshot with
`s5_status_encode(payload, sizeof(payload))`, check `E_OK`, then call
`edr_frame_encode(S5_DATA_SOURCE, S5_DATA_KIND, payload, S5_DATA_SIZE,
packet, sizeof(packet), &packet_size)` with an 86-byte packet buffer.
Check its return value and copy exactly `packet_size` bytes to an agreed
transport. Mutex failures leave the output unchanged; never publish that
old buffer as a new sample. The Python receiver `tools/edr_decode.py`
recognizes source 5/kind 0 as an `obstacle` record.

The scan task and navigation APIs retain their existing ownership, locks,
stale-sample checks and invalidation during servo motion. Packing creates
no new motor writer and does not add an MQTT handoff. No RTOS object IDs,
pointers, mutex internals, or scan arithmetic intermediates are serialized.

## Verification

Run `python -B testing/data_representation/run.py --report edr-check-report.json`.
The common gate discovers `test_s5.py`, which builds the real S5 storage API,
codec, scan, width and policy sources against a host-only RTOS/hardware shim
in both normal and `APP_TEST=1` configurations. It checks all control/flag
bytes, every encoded angle byte in each slot, every bit of the nineteen
32-bit fields, signed extremes, exact bytes/endian order, C-to-Python decoding,
invalid input/output preservation, mutex failures, navigation invalidation,
fault fallback, and policy freshness across timestamp rollover.

These host tests do not establish firmware stack high-water marks, lock
timing, sensor accuracy, or physical navigation performance. On-target
validation is still required for those properties.
