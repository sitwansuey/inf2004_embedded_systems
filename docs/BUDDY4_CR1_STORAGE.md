# Buddy 4 CR1 representation and evidence

CR1 applies to stored application fields, cross-task/ISR records, queues and
communication payloads. Each buddy may use its own compact format. A shared
contract and matching decoder are required at each boundary; a universal
same-size format is not required. This table documents implemented Buddy 4
fields. Other owners' application storage is outside this audit.

Checkpoint 373ad6d is published in draft PR #8. The subsequent Buddy 4 changes
and evidence described here are local until explicitly published. The old
checkpoint field review is recoverable from that commit; it is not the current
persistent layout. Main's newer Buddy 5 integration remains unreconciled.

## Persistent sensor cache and cross-task acquisition record

The sensor task owns a 63-byte cache; the protected published copy uses the
same 500-bit representation. s4_sensor_get_record returns a coherent copied
63-byte record. Consumers decode it after reading operating time. Scaled XYZ,
ages and observation time are derived in short-lived calculation/diagnostic
views rather than duplicated in the cache. The compatibility get_status API is
used by S4 diagnostics/tests; the controller and B1 use the compact record API.

Bits are LSB-first within bytes. Unsigned offset encoding stores value minus
minimum. Private cache offsets are fixed by sensor_data.c; this cache is not
silently substituted for either MQTT schema.

| Field | Bit offset | Valid range / resolution | Required and implemented bits |
|---|---:|---|---:|
| init_result | 0 | Six result codes 0..5 | 3 |
| poll_result | 3 | Six result codes 0..5 | 3 |
| configured | 6 | Boolean | 1 |
| accel raw X / Y / Z | 7 / 19 / 31 | -2048..2047, configured 1 mg/LSB | 12 each |
| accel read operating ms | 43 | 0..18446744073709551, 1 ms | 55 |
| accel fresh / valid / overrun | 98 / 99 / 100 | Boolean | 1 each |
| accel status | 101 | Five codes 0..4 | 3 |
| mag raw X / Y / Z | 104 / 120 / 136 | -32768..32767, including invalid diagnostic reads | 16 each |
| mag read operating ms | 152 | 0..18446744073709551, 1 ms | 55 |
| mag fresh / valid | 207 / 208 | Boolean | 1 each |
| mag status | 209 | Five codes 0..4 | 3 |
| init_attempts | 212 | Modulo 2^32 diagnostic count | 32 |
| task_heartbeat | 244 | Modulo 2^32 diagnostic count | 32 |
| accel_samples | 276 | Modulo 2^32 diagnostic count | 32 |
| mag_samples | 308 | Modulo 2^32 diagnostic count | 32 |
| accel_io_errors | 340 | Modulo 2^32 diagnostic count | 32 |
| mag_io_errors | 372 | Modulo 2^32 diagnostic count | 32 |
| accel_overruns | 404 | Modulo 2^32 diagnostic count | 32 |
| publish_errors | 436 | Modulo 2^32 diagnostic count | 32 |
| legacy_lines_received | 468 | Modulo 2^32 diagnostic count | 32 |

The last four bits are zero padding: ceil(500/8)=63 bytes. Full magnetic raw
precision is retained so clipped/overflow values can be diagnosed. Scaled mg
and mG are reconstructed exactly from raw values using the driver's fixed
configuration (XY gain 1100, Z gain 980 for mG). They are not calibration claims.
Counters retain the existing cumulative modulo-32 domain because no shorter
operating lifetime or reset-on-run policy was agreed; reducing them would
change diagnostic behavior. Saturated sample ages and UINT32_MAX no-sample
codes are calculated, not cached.

Public timestamps remain operating milliseconds scaled by 1000 for API
compatibility. Thus the largest representable valid millisecond timestamp is
floor((UINT64_MAX-1)/1000), requiring 55 bits. Cache writes reject submillisecond
or invalid timestamps instead of silently rounding. Clock multiplication is
checked for overflow. Fresh/valid flags distinguish absence and invalid data;
no fake timestamp sentinel consumes an otherwise valid time code.

## Task-local terrain storage

The following fields occupy 165 information bits. GNU packed bitfields occupy
21 bytes on the tested Cortex-M0+ ABI and 26 on native MinGW (compiler layouts
differ). No member address is exposed. GCC handles packed ARM member accesses;
this compiler-specific layout is never sent to another task or dumped onto a
wire. Wide arithmetic intermediates retain necessary signed range/precision.

| Field | Valid range / resolution | Required and implemented bits |
|---|---|---:|
| previous_distance_um | Full signed int32 calibrated cumulative position | 32 |
| height_code | -20000..1000000 um, stored height + 20000 | 20 |
| peak_um | 0..1000000 um | 20 |
| sample_ms | Modulo 2^32 operating milliseconds | 32 |
| pitch_code | -9000..9000 centidegrees, offset by 9000 | 15 |
| roll_code | -18000..18000 centidegrees, offset by 18000 | 16 |
| height_mm | 0..1000 mm | 10 |
| maximum_mm | 0..1000 mm | 10 |
| terrain state | Six states | 3 |
| tilt_valid / height_valid / shock / initialized | Boolean | 1 each |
| flat_samples | 0..configured threshold (default 5) | 3 at default; compile-time width follows configured 1..255 threshold |
| encoder adapter reason | Six codes | 3 |
| acquisition initialized | Boolean | 1 |
| retry operating ms | Existing supported clock domain plus retry interval | 55 |
| consecutive failed polls | 0..10, then reinitialize | 4 |

Micrometre and centidegree resolution describe numerical representation, not
established physical accuracy. The encoder adapter retains B2-owned full-range
counts/timestamps and calibrated odometry history. Its synchronization/atomic
metadata and RTOS object IDs remain correctly aligned and are not narrowed as
application flags. Raw driver frames and decoded snapshot/result views are
bounded-lifetime processing views; authoritative stored/shared data uses the
compact forms above. int32/int64 intermediates are used for signed differences,
raw-times-gain arithmetic, norm sums and geometry rather than narrowed before
checking.

## Buddy 4 result contract for the controller and Buddy 1

result_data.c/.h define a separate 93-bit result in exactly 12 bytes. Both
receivers use the same checked decoder. The controller keeps its terrain
history private and publishes only these result bytes in app_state. B1 decodes
them into the existing aggregate telemetry schema. The controller also decodes
this checked record before using terrain validity/state for decisions.

| Field | Bit offset | Valid range / meaning | Bits |
|---|---:|---|---:|
| calculated_ms | 0 | Modulo 2^32 operating ms when result was calculated | 32 |
| pitch_cdeg | 32 | -9000..9000, offset encoding | 15 |
| roll_cdeg | 47 | -18000..18000, offset encoding | 16 |
| height_mm | 63 | 0..1000; validity flag required | 10 |
| maximum_mm | 73 | 0..1000; positive value is completed-hump history | 10 |
| terrain | 83 | Flat/ascent/crest/descent/shock/invalid, codes 0..5 | 3 |
| tilt_valid | 86 | Boolean | 1 |
| height_valid | 87 | Boolean | 1 |
| shock | 88 | Boolean | 1 |
| mounting_calibrated | 89 | Boolean | 1 |
| odometry_valid | 90 | Boolean | 1 |
| encoder fault | 91 | Decodes to shared EDR_FAULT_ENCODER bit 0x02 | 1 |
| IMU fault | 92 | Decodes to shared EDR_FAULT_IMU bit 0x04 | 1 |

Last three bits are reserved zero padding. Decode rejects wrong length,
padding, unused numeric codes and contradictory validity/fault combinations.
Encode validates everything before touching output. Decoders leave their
caller output unchanged on error. Source/type framing, when needed externally,
uses existing EDR v1 source 4 / kind 2: header bytes 54 0c (hex), then the
12-byte payload, total 14 bytes. This optional framing is not added to the
fixed internal record or automatically adopted as the MQTT dashboard format.
The existing 50-byte aggregate schema and 13-byte B2 encoder payload remain
separate. tools/buddy4_result.py supplies an independent receiver decoder.

These are individual bits, not integer fault variables. They map explicitly
to common EDR meanings. The aggregate controller reason remains a diagnostic
reason code and must not be cast into a fault mask. A generic VC_SENSOR reason
cannot identify IMU versus encoder versus sonar without source information.
Other buddies and the full team fault-aggregation policy still need owner review.

## Synchronization and measurements

Sensor working data and terrain history have one task owner each. All shared
cache copies and legacy-counter read-modify-write operations are inside S4's
existing bounded priority-inheritance mutex. Packed output bytes are copied
under app_state's existing mutex. Network operations stay in the network owner.
No volatile/local-IRQ workaround or packed atomic word was introduced.

Compared with checkpoint 373ad6d using the same compiler/SDK/profile: persistent
sensor record **120 -> 63 bytes**; shared terrain member **28 -> 12 bytes**;
ARM task-local terrain **28 -> 21 bytes**. Native task-local terrain is
**28 -> 26 bytes**. Derived calculation snapshots remain 120 native/ARM bytes,
but are no longer the persistent cache or the cross-task acquisition record.
The whole 50-byte MQTT aggregate remains 50 bytes. A previously nonexistent
standalone B4 result packet is 12 bytes, or 14 with EDR framing; no fictional
before size is claimed for that new packet.

Exact full-image data/BSS/flash measurements and hashes are recorded in
../testing/subsystem4/cr1-verification.json. These are static RAM measurements,
not runtime heap/stack high-water evidence. Native benchmark results in
cr1-benchmark.json compare the actual checkpoint data code with the compact
implementation: approximately 20.5?22.5 ns versus 610.5?716 ns per update+snapshot on
this host, with identical checksums (2,000,000 iterations per run, three runs).
The host timer is coarse and these results cannot establish RP2040 timing.
Packing saves storage while adding work; hardware timing remains required.

Correctness evidence includes acquisition/quality regressions, packed raw/time
boundaries, counter isolation/wrap, mutex/error behavior, terrain transitions,
maximum retention/reset, actual vehicle and telemetry consumers, 259 independent
C/Python result vectors (57 accepted/202 rejected), and the unchanged aggregate
protocol tests. Full integrated and S4Bench builds are checked separately.
ASan/UBSan remain unavailable in this native toolchain. Physical calibration,
performance under concurrent load, fault-to-PWM timing and runtime memory
high-water measurements remain Week 13 evidence still to collect. This is a
Buddy 4 software field-width review, not a claim of whole-team CR1 completion.

The compiled controller function has a 688-byte own frame within its configured
2048-byte task stack. This excludes nested calls/interrupt effects and is not
a runtime high-water or complete stack-safety measurement. See cr1-static-stack.log.
