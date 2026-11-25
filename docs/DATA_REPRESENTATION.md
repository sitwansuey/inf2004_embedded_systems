# External data representation: EDR16 v1

Each telemetry record is one 16-bit word: bits 0–3 identify the reading;
bits 4–15 contain unsigned data. Send exactly two bytes, low byte first,
on `Robopico/sensors/edr16/v1`. Use explicit byte lengths, not `strlen()`
or native structure copies.

```c
word = (data << 4) | id;
id = word & 0x000f;
data = word >> 4;
```

The C codec is `app_program/shared/edr.h` and `edr.c`. The Python codec is
`tools/edr_decode.py`, imported by the dashboard adapter. `edr_encode()`
checks capacity and values; `edr_decode()` requires exactly two bytes.
Errors leave C output buffers unchanged. No envelope, timestamp or checksum
is added. Robot events use the same shape on a separate topic and must use
the event decoder below. Both codecs reject reserved ID 0 values, unused
flag bits and out-of-range values. The topic suffix `/v1` is the only
version marker, so update the firmware and dashboard/Python together.

## Sensor records

| ID | Reading | Meaning of the 12-bit data |
|---|---|---|
| 1 | S2 left encoder / motor 1 | Cumulative forward count modulo 4096 |
| 2 | S2 right encoder / motor 2 | Cumulative forward count modulo 4096 |
| 3 | S5 servo | Commanded angle 0–180°; S5's signed angle plus 90 |
| 4 | S5 ultrasonic | Whole centimetres, 0–4095 in the codec |
| 5 | S3 left IR | Raw ADC 0–4095; 0/1 with `BOARD_IR_MODE` 1 (digital) |
| 6 | S3 right IR | Raw ADC 0–4095; 0/1 with `BOARD_IR_MODE` 1 (digital) |
| 7 | S3 barcode IR | GP28 digital 0/1, 1 = black, with `BOARD_IR_MODE` 3 (the mission setting); raw ADC with mode 2 |
| 8 | S4 magnetic X | Milligauss + 2048 |
| 9 | S4 magnetic Y | Milligauss + 2048 |
| 10 | S4 magnetic Z | Milligauss + 2048 |
| 11 | S2 motor status | Command in bits 0–2, power in bits 3–9, state in bits 10–11 |
| 12 | Motor and IMU fault flags | Bits 0–6 below; bits 7–11 zero |
| 13 | S4 acceleration X | Milligravity (mg) + 2048 |
| 14 | S4 acceleration Y | Milligravity (mg) + 2048 |
| 15 | S4 acceleration Z | Milligravity (mg) + 2048 |

Magnetic and acceleration values use signed integers −2048 to 2047 before
the offset. The dashboard converts magnetic data with `(data - 2048) / 10`
µT and acceleration with `(data - 2048) * 9.80665 / 1000` m/s². S1 bounds these
inputs before packing. Its sonar source requires a fresh, valid distance
above zero and at most 5000 mm; it rounds `(distance_mm + 5) / 10` to cm.
The servo value is a command, not measured position feedback.

Motor commands 0–6 are STOP, FORWARD, REVERSE, LEFT, RIGHT, PIVOT_RIGHT and
PIVOT_LEFT. Power is 0–100%; states 0–2 are STOPPED, MOVING and FAULT.
Other command, power or state values are rejected. The motor word is
software-packed command state, not a hardware encoder packet.

| ID 12 bit | Definition |
|---|---|
| `0x01` | Stall |
| `0x02` | Encoder |
| `0x04` | IMU |
| `0x08` | No echo |
| `0x10` | Wi-Fi |
| `0x20` | Motor timeout |
| `0x40` | Other motor error |

The current firmware sets only three of these bits. Motor timeout or other
motor error comes from S2's last error (`s2_motor_error_code()`). S1 adds IMU
when a controller S4 result, or a change in the VC's IMU-fault count, showed an
IMU fault since the last queued snapshot that carried ID 12. Stall, encoder,
no echo and Wi-Fi are defined but never set. A clipped accelerometer sample
is also an impact; an unclipped impact alone is not an IMU fault. ID 12 is
sent only with motor availability, so the IMU bit stays set until a queued
snapshot has motor availability.

Examples, with bytes in transmission order:

| Reading | Word | Bytes |
|---|---|---|
| Left count 100 | `0x0641` | `41 06` |
| Left count 0 | `0x0001` | `01 00` |
| Forward, 30%, moving | `0x4f1b` | `1b 4f` |
| Timeout flag | `0x020c` | `0c 02` |
| Magnetic X zero | `0x8008` | `08 80` |

## Mission-status ID 0

For ID 0, data bits 11–8 contain a subtype and bits 7–0 contain its value:
`data = (subtype << 8) | value`. Each record is independently decodable.
Quantisation affects telemetry only; control uses full internal precision.

| Subtype | Value layout and units |
|---|---|
| 0 | Low nibble: phase 0–13, with 13 reverse before turn-out. High nibble: controller fault 0–8, with 8 lifted |
| 1 | Bits 0–4: last decoded letter, 0 none or 1–26 A–Z. Bits 5–7: action 0 none, 1 left, 2 right, 3 straight, 4 U-turn |
| 2 | Bits 0–2: line request, 0 straight/not following, 1 steer right, 2 steer left, 3 pivot right, 4 pivot left; the default mask-pivot follower (`BOARD_LINE_MASK_PIVOT` 1) sends 3/4 only while exactly one line sensor is on black, else 0. Bits 3–5: black mask; mission uses 1 right, 2 left, 3 both. Bits 6–7 zero |
| 3 | Pitch, whole degrees + 128; values 38–218 |
| 4 | Roll, two-degree units + 128; values 38–218 |
| 5 | Bits 0–2: terrain 0 flat, 1 ascent, 2 crest, 3 descent, 4 shock, 5 invalid. Bits 3–7: barcode elements read, 0–31; a full `*X*` has 29 |
| 6 | Completed hump height from S4's terrain result (the VC runs S4's terrain code), 5 mm units, 0–200; sent only with height availability |
| 7 | Highest completed hump this run, from the same result, 5 mm units, 0–200 |
| 8 | Width pages: 0–100 total in 5 mm units; 128–191 left half in 8 mm units; 192–255 right half in 8 mm units. For a half, use `value & 63`; 101–127 rejected. All three pages are sent only while quality bit 0 (width valid) is set |
| 9 | Bits 0–5: width valid, impact, armed, bypass validated (build setting), odometry valid, servo status valid. Bits 6–7: fault cause below |
| 10 | Availability bits 0–7: motor, encoder, IR, acceleration, sonar, tilt, height, magnetic field |
| 11 | Observation sequence modulo 256 |
| 12 | Internal drop count modulo 256 |
| 13 | Signed speed in 20 mm/s units + 128; values 28–228 |
| 14 | Signed line error in 10-unit steps + 128; values 28–228 |
| 15 | Bits 0–3: S5 scan state 0–14. Bits 4–5: barcode result 0 none, 1 reading, 2 successful, 3 unsuccessful. Bits 6–7: bypass side 0 none, 1 left, 2 right; 3 rejected |

Phase and fault numbers follow `app_program/vehicle_policy.h`; scan states
follow `app_program/subsystem5/subsystem5.h`. With fault 1 (sensor), cause
0–3 means other, sonar, tilt or odometry. With fault 2 (line), cause 1 is
line data stale (no line data for `VC_LINE_GAP_MS`, 250 ms, or no TRACK
handoff), 2 is an S3 fault event (including an S3 latch still present after
`VC_S3_LATCH_MS`, 1 s) and 3 is line not found (the spot search failed; it
also starts after `BOARD_LINE_LOST_MM` of tracking with neither line sensor
on black); 0 is other. Other faults send cause zero.
Consumers must mask quality with 63 before reading its flags.

Signed angle, speed and line-error conversions truncate toward zero.
Height and total width use `(mm + 2) / 5`. Half-widths use `(mm + 4) / 8`,
saturating at 63 (504 mm). The total width field is bounded at 500 mm.
Independent rounding means the displayed halves need not add to the total.

The reported widths come from `s5_width_reported()` in `app_program/subsystem5/s5_width.c`:
`half = depth * tan(max(0, edge_angle - S5_BEAM_HALF_DEG))`, rounded to mm.
`S5_BEAM_HALF_DEG` defaults to 20°. Planning retains the measured edge bounds
and conservative widths; it does not use the reduced display width. S5
holds a valid width for at least `S5_WIDTH_PUBLISH_MS` (300 ms) before choosing
a side. S1 requests an extra snapshot for new widths or a changed side.

The bypass side in subtype 15 is S5's planned side: the open side with the smaller
measured half-width (equal halves: the side whose nearest side echo is farther, then the smaller heading, then left). It is 0 until S5
plans a side. The side is sent while S5 still probes that side's corridor; if a probe rejects it, S5 plans the other side or reports BLOCKED (side 0). S5 compares its measured halves, not the beam-corrected
ones shown on the dashboard, so the chosen side need not be the side with
the smaller displayed half.

A reading is sent only when its source is usable: IDs 1–2 need encoder
availability, ID 3 quality bit 5 (servo status valid), ID 4 sonar, IDs 5–7
and subtypes 2 and 14 IR, IDs 8–10 magnetic field, IDs 11–12 motor, IDs 13–15
acceleration, subtypes 3–4 tilt, subtype 6 height, subtype 8 quality bit 0
and subtype 13 quality bit 4 (odometry valid). The other subtypes are always
sent. Availability and quality clear unusable live dashboard values. The receiver keeps the last scan's widths
and chosen side across quality-bit clearing, robot silence and broker
reconnects, until new results replace them. Other live observations clear on
broker disconnect/reconnect. Retained records are ignored. Each displayed
reading becomes stale after 5 s without an update. Arrival time is not
source time.

## Queues and local records

S1 normally samples every 200 ms and sends extra snapshots on selected
changes. Impact-triggered extras are limited to one per 200 ms; controller
fault, width and side changes are not rate-limited this way. The two-slot
queue drops older snapshots and selects the newest current one. Drops
increment a shared counter. Discarded snapshots carry impact and IMU-fault
bits into the next snapshot.

The network task (`cyw43_utk_task`) alone calls lwIP/MQTT. Each poll attempts
a pending reply, one event and up to four telemetry records. It retries a telemetry record
when the network cannot accept it. An unfinished snapshot aged 500 ms or
more is abandoned and counted; unsent impact/IMU bits carry forward. The
next snapshot starts at the unsent field so later fields do not starve.
A complete observation can produce up to 33 records: 15 sensor IDs,
16 status subtypes and two extra width pages. QoS 0 does not guarantee that
every record arrives. The sequence is diagnostic, not an atomic frame ID.

| Internal record | Size | Definition / use |
|---|---|---|
| S1 telemetry snapshot | 433 bits in 55 bytes, seven zero padding bits | `app_program/subsystem1/telemetry_codec.h`, `tools/telemetry_schema.json`; private message-buffer payload |
| S2 encoder snapshot | 13 bytes | `app_program/shared/encoder_snapshot.h`; full 32-bit counts, time and flags for odometry |
| S3 navigation event | 14 bytes | `app_program/subsystem3/event_codec.h`; kind/command, letter, sequence, time and generation |
| S4 acquisition cache | 564 bits in 71 bytes | `app_program/subsystem4/sensor_data.h`; copied sensor data and counters |
| S4 terrain result | 93 bits in 12 bytes | `app_program/subsystem4/result_data.h`; checked result used by the controller and telemetry |
| S5 status | 92 bytes | `app_program/subsystem5/s5_data.h`; packed shared status, decoded for callers |

These internal records are not MQTT payloads. `s2_motion_encode()` and
`s2_encoder_encode()` each return four bytes containing two separate EDR16
records (IDs 11/12 and 1/2 respectively). Publish each two-byte slice
separately. Ownership, synchronization and interfaces are in
[TEAM.md](../TEAM.md), including its
[data routes to the dashboard through S1](../TEAM.md#data-to-the-dashboard-through-s1).
The S3 navigation event ends with three little-endian 32-bit values:
sequence, time and generation.

## Robot events

Publish one two-byte record per message on `Robopico/events/edr16/v1`.
The low nibble is an event ID and the upper 12 bits are its value.
`app_program/shared/events.h` packs events;
`decode_event()` in `tools/edr_decode.py` validates and decodes them.
`decode_packet()` is for telemetry and must not decode events.

| ID | Event | Value layout |
|---|---|---|
| 1 | Barcode attempt | Bits 0–1 attempt: 1 initial, 2 reverse, 3 final. Bits 2–6 elements 0–31. Bits 7–11 letter: 0 failed, 1–26 A–Z |
| 2 | Instruction | Bits 0–4 letter 1–26. Bits 5–7 action 0 none/unmapped, 1 left, 2 right, 3 straight, 4 U-turn. Bits 8–11 zero |
| 3 | Hump | Bits 0–2 kind 1–5; bits 3–11 value 0–511, as below |
| 4 | Obstacle | Bits 0–1 kind, bits 2–3 side/completion reason, bits 4–11 value 0–255, as below |
| 5 | Broker link | Bits 0–1 kind: 1 reconnected, 2 first connection. Bits 2–11 seconds, saturated at 1023 |

Hump kinds are 1 detected (tilt in 0.1°), 2 crest (S4 height so far),
3 level (S4 completed height), 4 crest (controller height so far) and
5 level (controller's highest point).
All heights use 2 mm units. Kinds 4/5 use the controller's estimate from
wheel distance and smoothed pitch in `app_program/vehicle_hump.h`, including
its smoothing lag correction. When that estimate is unavailable, the controller sends
kind 2/3 using S4; zero means no valid height. These event estimates do not
replace S4's telemetry height fields. The dashboard labels kinds 4/5 as
estimated from wheel distance and tilt.

Obstacle kinds are 1 detected (range cm), 2 scanned (reported width cm),
3 bypass started (heading degrees), and 0 avoidance ended (elapsed seconds).
For kinds 2/3, side 1 is left and 2 is right. Kind 2 with side 0 instead means
no way round was found and S5 is scanning again; the value is the retry number, 1 up to `AVOID_RESCAN_RETRIES` (2 in
`app_program/shared/avoidance_config.h`; the format allows 1–3). For kind 0,
bits 2–3 mean 0 fault/STOP,
1 line found on return, 2 spot search, 3 obstacle went away.

An instruction follows the barcode attempt that decoded it. Broker reconnect
seconds are counted from the first publish the broker did not acknowledge
(or from the disconnect if none was waiting); first-connection seconds are
uptime. The controller posts robot events through `s1_event_post()`. S1's
`event_buffer` is 160 bytes; each two-byte event takes 8 bytes in the kernel
buffer (a 4-byte length header and the data rounded up to 4), so it holds 20.
When it is full, a new event is dropped and counted in the drop counter
(status subtype 12). Once the broker has acknowledged nothing for 3 s, the
network task stops passing events to lwIP and leaves them in S1's buffer:
lwIP keeps a dead connection until its keep-alive closes it (about 90 s)
and would drop them unsent. After 10 s without a Wi-Fi link (20 s for the
first join after boot) the network task leaves and re-joins the access point
and closes the MQTT session, so the 3 s MQTT retry reconnects when the link
returns. The network task creates link events itself, keeps only the latest
unsent one and publishes it before the queued robot events.

The receiver keeps 200 event rows and adds its own rows: robot offline after
3 s without robot data and back online, its own broker link, each new
controller fault from status subtype 0, and rising impact/IMU-fault flags.
Those flags stay shown for 2 s. Event text is created by the dashboard; it
is not sent by the robot. Demo and imported dashboard data stay separate
from live MQTT data.

## Limits and verification

Encoder telemetry wraps from 4095 to 0; reverse count −1 becomes 4095.
A signed delta is ambiguous beyond 2047 counts between observations, and
reboot cannot be distinguished from wrap. Sequence/drop counters also alias
if more than 255 increments pass between received records. Related records
are not atomic and carry no source timestamps. Keep control decisions onboard.
The only heading in this protocol is the relative bypass heading event;
there is no compass heading field.

Run `python -B testing/data_representation/run.py`, or the root launchers
`.\check_data.cmd --no-pause` (Windows) / `sh check_data.sh`, which also write
`edr-check-report.json` (git-ignored) in the repository root. A native GCC or
Clang is needed. Stages 1–2 cover the Python codec and dashboard tests, the S5
status codec, the C codec and S1's queue test. Stage 3 runs S2's nine host
profiles. `python -B testing/integration/run.py` checks the 55-byte snapshot
codec and a snapshot through the 33 records to the dashboard. Record results
under the test IDs in
[S1](../testing/subsystem1/TEST_LOG.md) (S1-U04, S1-U05), [S2](../testing/subsystem2/TEST_LOG.md) (S2-U01..S2-U09),
[S5](../testing/subsystem5/TEST_LOG.md) (S5-U01) and
[integration](../testing/integration/TEST_LOG.md) (INT-05, INT-06). Results
go in each folder's `TEST_LOG.md`, with full output in its `logs/` folder;
[data representation](../testing/data_representation/TEST_LOG.md) keeps the earlier migration record. See
[the verification guide](../testing/data_representation/README.md) and
[testing/README.md](../testing/README.md). A result applies only to the
commit it records; these host tests do not show hardware performance or a
live broker link. Command frames are documented in
[MQTT_TOPICS.md](../app_program/external_systems/Dashboard/MQTT_TOPICS.md#command-topic).
