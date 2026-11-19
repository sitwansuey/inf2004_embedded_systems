# Repository external data representation: EDR16 v1

EDR16 is the common binary telemetry standard for all buddies. Each record is
one uint16_t: bits 0–3 identify the reading and bits 4–15 hold its unsigned data.
Transmit exactly two bytes, least-significant byte first. No source/length
header is prepended. Use explicit byte lengths, never strlen or native structs.
The MQTT topic `Robopico/sensors/edr16/v1` identifies this protocol/version.

```c
word = (data << 4) | id;
id = word & 0x000f;
data = word >> 4;
```

The canonical C definition is `app_program/shared/edr.h`; the canonical Python
codec is `tools/edr_decode.py`, which the dashboard adapter imports. Do not add
another independent packing scheme in a buddy's folder.

| ID | Binary | Owner / reading | Meaning of the 12-bit data |
|---|---|---|---|
| 1 | 0001 | B2 left encoder / motor 1 | cumulative forward count modulo 4096 |
| 2 | 0010 | B2 right encoder / motor 2 | cumulative forward count modulo 4096 |
| 3 | 0011 | B5 servo | whole degrees, 0–180 |
| 4 | 0100 | B5 ultrasonic | whole centimetres, 0–4095 |
| 5 | 0101 | B3 left IR | raw reading, 0–4095; digital mode 0/1 |
| 6 | 0110 | B3 right IR | raw reading, 0–4095; digital mode 0/1 |
| 7 | 0111 | B3 barcode IR | GP28 digital level 0/1 (1 = black) in IR mode 3; raw ADC reading in mode 2; not decoded text |
| 8 | 1000 | B4 magnetometer X | milligauss + 2048 |
| 9 | 1001 | B4 magnetometer Y | milligauss + 2048 |
| 10 | 1010 | B4 magnetometer Z | milligauss + 2048 |
| 11 | 1011 | B2 motor status | command[2:0], power[9:3], state[11:10] |
| 12 | 1100 | B2 motor fault flags (+ S4 IMU bit) | fault bit mask, 0–127; bits 7–11 zero |
| 13 | 1101 | B4 accelerometer X | milligravity (mg) + 2048 |
| 14 | 1110 | B4 accelerometer Y | milligravity (mg) + 2048 |
| 15 | 1111 | B4 accelerometer Z | milligravity (mg) + 2048 |

ID zero identifies mission-status observations. Its 12-bit data contains a
4-bit subtype in bits 11-8 and an 8-bit value in bits 7-0. This extension uses
no sensor ID twice. Each record remains independently decodable; no selector
state, envelope, multi-record MQTT payload or atomic snapshot is assumed.
Older receivers that reject ID zero must be updated with this repository.
Out-of-range inputs are rejected without changing output buffers. Offset-binary
magnetic/acceleration helpers accept -2048…2047. Magnetic display is (data-2048)/10
uT; acceleration display is (data-2048)*9.80665/1000 m/s². The latter covers the
current +/-2 g acquisition range to the stated integer bounds. The internal
snapshot bounds signed observations to this range before the EDR adapter. No float printf is required on the Pico.

## Mission-status ID 0

`data = (subtype << 8) | value`, then `word = data << 4` (ID zero).
Angles/speed/line error truncate toward zero; height/width round to nearest
5 mm. Quantisation is telemetry-only: control uses full internal precision.

| Subtype | Meaning of value |
|---|---|
| 0 | phase in low nibble (0-13; 13 = reverse before turn-out), controller fault in high nibble (0-8; 8 = lifted) |
| 1 | last decoded barcode in bits 4:0 (0 none, 1-26 = A-Z, mapped to a route or not); navigation in bits 7:5 (0 none, 1 left, 2 right, 3 straight, 4 U-turn) |
| 2 | line state bits 2:0, the line follower's request this tick: 0 straight (or not following), 1 steer right, 2 steer left, 3 pivot right, 4 pivot left (the default mask-pivot follower sends 3/4 only while that IR is on black and 0 when both are white); straddle IR black mask bits 5:3 (1 right, 2 left, 3 both) |
| 3 | pitch in whole degrees + 128 |
| 4 | roll in two-degree units + 128 |
| 5 | bits 2-0 terrain: 0 flat, 1 ascent, 2 crest, 3 descent, 4 shock, 5 invalid; bits 7-3 barcode elements (bars + spaces) read in the current/last barcode, shown as x/29 (0-31, saturating) |
| 6 | current completed hump height, in 5 mm units |
| 7 | highest completed hump this run, in 5 mm units |
| 8 | paged obstacle width: 0-100 total width in 5 mm units; 128-191 centre->left edge half-width, (value&63)*8 mm; 192-255 centre->right edge half-width, same units (saturates at 63 = 504 mm); 101-127 rejected. All three pages are sent only with quality bit 0 (width valid) |
| 9 | bits 5-0 quality: width valid, shock, armed, bypass validated, odometry valid, servo status valid (bit 5); bits 7-6 fault cause, sent with fault 1 (sensor): 0 none/other, 1 sonar (stale/fault/blocked scan, width or plan rejected), 2 tilt (terrain invalid), 3 odometry; and with fault 2 (line): 0 other, 1 line data stale (a line-data gap of 250 ms or more, or no TRACK handoff), 2 S3 fault event (or a silent S3 STALE/SENSOR latch still returning 1 s after it was first seen, although the controller re-sends TRACK every 100 ms), 3 line not found (REACQUIRE spot search, including after `BOARD_LINE_LOST_MM` of tracking with neither straddle IR on black); 0 with any other fault. All 256 values valid; consumers mask quality with 63 |
| 10 | availability bits: motor, encoder, IR, acceleration, sonar, tilt, height, magnetic field |
| 11 | observation sequence modulo 256 |
| 12 | internal drop count modulo 256 (wraps like subtype 11; receivers take differences between records) |
| 13 | signed vehicle speed in 20 mm/s units + 128 |
| 14 | signed line error in 10-unit increments + 128 |
| 15 | bits 3-0 scan state, matching the subsystem5 enum; bits 5-4 barcode decode result: 0 none yet, 1 reading, 2 successful, 3 unsuccessful; bits 7-6 bypass side: 0 none, 1 left, 2 right (3 rejected) |

Half-widths are measured from the sonar's forward axis: with forward range d0
and mean edge slant range dE, half = isqrt(dE²-d0²), falling back to
d0·tan(edge angle) when dE <= d0; the total (page 0-100) is left + right.
Telemetry rounds each half to (mm+4)/8. The bypass side is the planned side
(S5 picks the feasible side with the smaller half-width) and is 0 until a
verified plan exists. After READY, widths and side persist through navigation.
S5 publishes the widths for at least 300 ms before
choosing a side; the telemetry task sends an extra snapshot as soon as a new
valid width or a changed side appears, so the halves reach MQTT first.

Controller phases and faults match `vehicle_policy.h`. Sensor values are sent
only when their source is usable; mission availability/quality records clear
invalid dashboard values. The dashboard keeps each reading's latest value and
marks it stale once it is more than 5 s old, independently of traffic from
other sensors. Arrival time is not source time.

The telemetry task samples at 5 Hz. The network task sends at most four
records per poll, retries the same record on backpressure, and discards an
unfinished observation once its internal timestamp reaches 500 ms, counting
the drop. The two-slot queue keeps the newest observation and counts older
ones it discards. Dropped observations pass impact and IMU-fault bits to
the next one. Impact extras are limited to one per 200 ms; fault, width
and side changes are not limited. A failed queue send retries the change. Up to 33
messages (15 sensor IDs, 16 status subtypes, 2 width pages) describe one observation; MQTT QoS 0 does not guarantee all arrive.
The sequence field is diagnostic, not an atomic-frame assembly protocol.

The 55-byte `telemetry_codec` record is a private RTOS mailbox format; its
schema is in `tools/telemetry_schema.json`. It never crosses MQTT. Similarly,
`encoder_snapshot.h` retains 32-bit counts, monotonic time and validity for
odometry/terrain. These internal codecs prevent loss of control precision;
they are not alternate external telemetry formats.

## Buddy 2 motor and encoder distinction

The physical encoder supplies GPIO transitions. Buddy 2's code counts those
transitions into full-width counters. Its older uint16_t motor variable is
software-packed COMMAND/POWER/STATE, not a hardware encoder packet.

The existing motor variable is now the 12-bit DATA field of ID 11. Command
codes 0–6 mean stop, forward, reverse, left, right, pivot-right, pivot-left.
Power is 0–100; states 0–2 mean stopped, moving, fault. Other combinations are
rejected. In the complete record, command occupies bits 4–6, power 7–13, state
14–15, and the bottom four bits are 1011.

ID 12 retains the shared fault bit assignments: stall 0x01, encoder 0x02,
IMU 0x04, no-echo 0x08, Wi-Fi 0x10, timeout 0x20, motor 0x40. The current B2
adapter populates timeout or motor error only; it does not invent sensor faults.
The telemetry adapter also sets IMU 0x04 when any vehicle-controller S4 result
since the last snapshot had `EDR_FAULT_IMU` (I/O error, accelerometer sample
older than 100 ms, clipped or overrun sample, IMU/encoder skew). A sample
clipped at +/-2 g is also a shock (impact, quality bit 1); an unclipped shock
is not an IMU fault. S4's monotonic sample counters also cover faults and
impacts between controller reads. The VC consumes all counter increments
and counts local copy/skew faults. The IMU latch clears only after a
successful queued snapshot carrying ID 12; a missing motor status keeps it.

Examples (wire bytes shown in transmission order):

- Left count 100: word 0x0641, bytes `41 06`.
- Left count 0: word 0x0001, bytes `01 00` (zero bytes are valid).
- Forward, 30 percent, moving: data 0x04f1, record 0x4f1b, bytes `1b 4f`.
- Timeout flag: word 0x020c, bytes `0c 02`.
- Magnetic X zero: word 0x8008, bytes `08 80`.

## APIs and handoff

`subsystem1/task_telemetry.c` reads bounded snapshots from all five owners and
produces the private mailbox record with `telemetry_encode`. `s1_binary_send`
copies that record into a two-entry RTOS queue. The network owner alone calls
MQTT, using `telemetry_edr_record` to convert it into independent two-byte
observations. Producers never call lwIP or publish text/JSON sensor topics.

B2 retains `s2_motion_encode` and `s2_encoder_encode` as EDR16 adapter APIs;
each returns two independent two-byte readings. `s2_encoder_snapshot` is the
separate full-precision 13-byte internal handoff used by terrain/odometry.
The former multi-topic text queue and `s1_telemetry_record` API are retired.

## Robot events

Discrete events for the dashboard's event log use their own topic,
`Robopico/events/edr16/v1`, one two-byte record per MQTT message in the same
EDR16 shape (low nibble = event ID, high 12 bits = value). They are not
telemetry: the vehicle controller (and the MQTT owner, for link events) calls
`s1_event_post`, subsystem 1 uses a 160-byte event buffer
(20 two-byte records with kernel overhead, `subsystem1/telemetry.c`)
and the network owner publishes one per poll after command replies
(`mqtt_utk.c`). lwIP keeps a dead connection until its keep-alive watchdog
closes it (~90 s), so once the broker has acknowledged nothing for 3 s the
events stay in the queue rather than going to lwIP, which would drop them
unsent; a full queue drops new events. After 10 s without a Wi-Fi link
(`cyw43_utk.c`) the network owner leaves and re-joins the access point and
closes the MQTT session at once, so the 3 s retry reconnects when the link
returns. The network owner holds its link event separately and publishes
it first, before the queued outage events. A stalled first join uses a
20 s leave/rejoin bound; a later outage uses 10 s. Event IDs are not sensor IDs: decode
event records with `decode_event`, never `edr_decode`/`decode_packet` (which
would, for example, reject a hump value above 180 as a servo angle). Layout (`app_program/shared/events.h`,
`tools/edr_decode.py decode_event`):

| ID | Event | 12-bit value |
|---|---|---|
| 1 | Barcode attempt (one per finished read) | bits 0-1 attempt (1 initial scan, 2 reverse scan, 3 final attempt); bits 2-6 bars+spaces read (0..31, a full `*X*` is 29); bits 7-11 letter (0 failed, 1..26 = A..Z) |
| 2 | Instruction decoded | bits 0-4 letter 1..26; bits 5-7 turn (0 none/not mapped, 1 left, 2 right, 3 straight, 4 U-turn) |
| 3 | Hump | bits 0-2 kind (1 detected: tilt in 0.1 deg; 2 crest, slowing down: this hump's height so far in 2 mm; 3 level again: the completed height latched by S4 for this traversal in 2 mm, or 0 if no valid hump completed); bits 3-11 value |
| 4 | Obstacle | bits 0-1 kind (1 detected: distance cm of the confirming forward echo; 2 scanned: width cm; 3 bypass started: turn-out heading deg; 0 avoidance over: seconds since detection); bits 2-3 side (1 left, 2 right) or, for kind 0, how (0 ended by a fault or STOP, 1 line found on the return, 2 on-the-spot search, 3 obstacle went away); bits 4-11 value |
| 5 | Link | bits 0-1 kind (1 reconnected to the broker after N s without it, counted from the first publish it never acknowledged; 2 first connection, N s after boot); bits 2-11 seconds |

An instruction (ID 2) is posted after the barcode attempt (ID 1) that decoded
it. The dashboard receiver adds its own rows: robot offline (no data for 3 s)
and back online, its broker link, each new controller fault code from
status subtype 0, and rising impact/IMU-fault flags. Each flag stays visible
for 2 s. A flat-top hump keeps one detected/crest/level cycle; a plain
descent is DESCENT with no hump height.

## Migration and limits

Update firmware and Python/dashboard together. The old source/kind/length
frames and 50-byte MQTT aggregate are rejected. Sensor IDs 1-15 are unchanged;
ID 0 adds the independently decodable mission status fields above.

Full 32-bit encoder counts, local timestamps, validity, leases and freshness
remain in local control APIs. B3's internal navigation event now uses 14 bytes:
kind/command, symbol code (0 none, 1-26 = A-Z), then sequence, operating time
and generation as three little-endian uint32 values. The former two-bit symbol
field could not carry the A-Z codebook. Native structures are never copied
onto MQTT or the event queue.

EDR16 encoder counts wrap 4095 to 0; reverse count -1 becomes 4095. Inferring a
signed delta is ambiguous beyond 2047 counts between received observations,
and reboot is indistinguishable from wrap. Telemetry is not a control input.
No per-reading source timestamp or checksum is transmitted. QoS 0 can drop
records; related readings are not atomic. The publisher bounds internal age
and the receiver marks each field stale after 5 s. Keep all control/safety
decisions onboard.
Free-text events are not transmitted, and no compass or magnetometer heading
is: the only heading sent is the relative turn-out angle in the EV_OBSTACLE
bypass event. Demo/import data remain explicitly separate from live MQTT.

## Verification

`check_data.cmd` / `check_data.sh` run the Python codec/dashboard tests, exhaustive
C codec, current B1 queue and B2 mock-hardware profiles using native GCC/Clang.
`python -B testing/integration/run.py --compiler clang` also verifies the real
snapshot-to-EDR adapter through Python and browser field routing, mission
policy, barcode decoding, queues, sonar and MQTT port. These are software
checks; they do not establish physical performance or a live broker link.

Mission status values are validated by both shared codecs; reserved enum values,
unused flag bits and values outside the documented source ranges are rejected.
Servo ID 3 is emitted only with quality bit 5 set (successful, non-fault B5
status copy). This is the commanded angle, not measured position feedback.
Motor ID 12 is emitted only with motor availability and preserves timeout (32)
versus other motor errors (64), using the same mapping as B2's direct adapter,
plus the IMU bit (4) described above.
Dashboard and audio ignore retained records and clear observations on disconnect
and reconnect. The private snapshot now uses 433 bits (barcode widened to 5 bits for A-Z, plus 5-bit barcode bar count and 2-bit decode result, two 10-bit half-widths, a 2-bit bypass side, a 2-bit sensor-fault cause, a 4-bit controller fault 0..8 and a 1-bit IMU fault; phase maximum 13) in 55 bytes, the last 7 bits zero padding; its
schema includes a two-bit motor error class. Rebuild firmware components together.

Checkpoint 8c changes no MQTT bit layout or command length. S4's protected
internal cache is now 71 bytes (564 bits), with 32-bit impact/fault counters.
The checked terrain result stays 12 bytes. Latched faults/impacts clear
incompatible tilt/height validity bits before encoding; validation remains
strict. The app-state fault count is local and is not a new wire field.

ACKs use the existing six bytes. STOP-superseded commands get result 1;
each coalesced STOP gets result 0. Network replies include the last VC
phase/fault. A full ACK buffer drops and counts the oldest reply. USB
sequence bit 7 separates it from dashboard sequences 0-127.
