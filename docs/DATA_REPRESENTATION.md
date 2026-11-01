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
codec is `tools/edr_decode.py`. Buddy 1's packed16 header is a compatibility alias,
and the dashboard adapter imports the common Python codec. Do not add another
independent packing scheme in a buddy's folder.

| ID | Binary | Owner / reading | Meaning of the 12-bit data |
|---|---|---|---|
| 1 | 0001 | B2 left encoder / motor 1 | cumulative forward count modulo 4096 |
| 2 | 0010 | B2 right encoder / motor 2 | cumulative forward count modulo 4096 |
| 3 | 0011 | B5 servo | whole degrees, 0–180 |
| 4 | 0100 | B5 ultrasonic | whole centimetres, 0–4095 |
| 5 | 0101 | B3 left IR | raw reading, 0–4095; digital mode 0/1 |
| 6 | 0110 | B3 right IR | raw reading, 0–4095; digital mode 0/1 |
| 7 | 0111 | B3 barcode IR | raw reading, not decoded text |
| 8 | 1000 | B4 magnetometer X | milligauss + 2048 |
| 9 | 1001 | B4 magnetometer Y | milligauss + 2048 |
| 10 | 1010 | B4 magnetometer Z | milligauss + 2048 |
| 11 | 1011 | B2 motor status | command[2:0], power[9:3], state[11:10] |
| 12 | 1100 | B2 motor fault flags | fault bit mask, 0–127; bits 7–11 zero |
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
| 0 | phase in low nibble, controller fault in high nibble |
| 1 | barcode in bits 2:0 (0 none, 1 A, 2 B, 3 C, 4 D, 5 Z); navigation in bits 5:3 (0 none, 1 left, 2 right, 3 straight, 4 U-turn) |
| 2 | line state bits 2:0; physical IR black mask bits 5:3 |
| 3 | pitch in whole degrees + 128 |
| 4 | roll in two-degree units + 128 |
| 5 | terrain: 0 flat, 1 ascent, 2 crest, 3 descent, 4 shock, 5 invalid |
| 6 | current completed hump height, in 5 mm units |
| 7 | highest completed hump this run, in 5 mm units |
| 8 | obstacle width, in 5 mm units |
| 9 | quality bits: width valid, shock, armed, bypass validated, odometry valid, servo status valid (bit 5) |
| 10 | availability bits: motor, encoder, IR, acceleration, sonar, tilt, height, magnetic field |
| 11 | observation sequence modulo 256 |
| 12 | dropped internal observations, saturated at 255 |
| 13 | signed vehicle speed in 20 mm/s units + 128 |
| 14 | signed line error in 10-unit increments + 128 |
| 15 | scan state, matching the subsystem5 enum |

Controller phases and faults match `vehicle_policy.h`. Sensor values are sent
only when their source is usable; mission availability/quality records clear
invalid dashboard values. Each desktop reading expires after one second,
independently of traffic from other sensors. Arrival time is not source time.

The telemetry task samples at 5 Hz. The network task sends at most four
records per poll, retries the same record on backpressure, and discards an
unfinished observation once its internal timestamp reaches 500 ms. Up to 31
messages describe one observation; MQTT QoS 0 does not guarantee all arrive.
The sequence field is diagnostic, not an atomic-frame assembly protocol.

The 50-byte `telemetry_codec` record is a private RTOS mailbox format; its
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

## Migration and limits

Update firmware and Python/dashboard together. The old source/kind/length
frames and 50-byte MQTT aggregate are rejected. Sensor IDs 1-15 are unchanged;
ID 0 adds the independently decodable mission status fields above.

Full 32-bit encoder counts, local timestamps, validity, leases and freshness
remain in local control APIs. B3's internal navigation event now uses 14 bytes:
kind/command, symbol code (0 none, 1-4 A-D, 5 Z), then sequence, operating time
and generation as three little-endian uint32 values. The former two-bit symbol
field could not carry all four commands. Native structures are never copied
onto MQTT or the event queue.

EDR16 encoder counts wrap 4095 to 0; reverse count -1 becomes 4095. Inferring a
signed delta is ambiguous beyond 2047 counts between received observations,
and reboot is indistinguishable from wrap. Telemetry is not a control input.
No per-reading source timestamp or checksum is transmitted. QoS 0 can drop
records; related readings are not atomic. The publisher bounds internal age
and the receiver expires each field. Keep all control/safety decisions onboard.
Free-text events and heading are not transmitted; demo/import data remain
explicitly separate from live MQTT.

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
versus other motor errors (64), using the same mapping as B2's direct adapter.
Dashboard and audio ignore retained records and clear observations on disconnect
and reconnect. The private snapshot now uses 398 bits, still 50 bytes; its
schema includes a two-bit motor error class. Rebuild firmware components together.
