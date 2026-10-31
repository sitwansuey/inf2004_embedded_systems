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

ID zero is reserved and rejected. All fifteen nonzero IDs are assigned. A new
reading type requires a versioned schema decision, not reuse of an existing ID.
Out-of-range inputs are rejected without changing output buffers. Offset-binary
magnetic/acceleration helpers accept -2048…2047. Magnetic display is (data-2048)/10
uT; acceleration display is (data-2048)*9.80665/1000 m/s². The latter covers the
current +/-2 g acquisition range to the stated integer bounds; clipped or
out-of-range samples are skipped. No float printf is required on the Pico.

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

`edr_pack` / `edr_unpack` handle uint16_t words. `edr_encode` / `edr_decode`
handle two-byte wire records and exact lengths. They return false on errors.
`edr_motor_pack` produces only the 12-bit motor DATA, not a full record.

`s2_motion_encode` returns four bytes: ID 11 followed by ID 12.
`s2_encoder_encode` returns four bytes: ID 1 followed by ID 2.
Each is a pair of independent two-byte records, NOT a 32-bit EDR packet.
Their error returns leave the caller's output untouched. Encoder startup with
no valid sample returns E_OBJ; disabled encoders return E_NOSPT.

```c
uint8_t records[EDR_ENCODER_SIZE];
if (s2_encoder_encode(records, sizeof(records)) == E_OK) {
    ER left = s1_telemetry_record(records, EDR_RECORD_SIZE);
    ER right = s1_telemetry_record(records + EDR_RECORD_SIZE, EDR_RECORD_SIZE);
    /* Check both results; queue-full means drop/retry later, never block control. */
}
```

All buddies may include `../shared/edr.h`, encode their assigned reading, and
call Buddy 1's `s1_telemetry_record(bytes, 2)` in task context. That validates and
copies the record into the nonblocking RTOS queue. Convenience APIs accept an
ID/raw value, magnetic mG, or acceleration mg. Servo/ultrasonic owners must
supply real valid measurements; this checkout has no public getters for them.

B1's 250 ms task reads B2's adapters and fresh usable B3/B4 snapshots. It sends
up to 13 records per pass. B1 alone bridges the queue into the existing network
owner; only cyw43_utk_task calls lwIP/MQTT. MQTT drains at most four queued
messages per visit. The 16-slot queue still accommodates legacy text/events,
so its storage is not two bytes per slot. Binary wire payloads are two bytes.

## Migration and limits

This replaces the old source/kind/version/length envelope and its 9-byte motor
and 15-byte encoder messages. Old frame/variable-payload APIs were removed;
old packets are rejected by the current decoder. Firmware and dashboard must
be updated together. The earlier B1-only packed16 MQTT topic is retired.

Full 32-bit encoder counters, local timestamps, validity, command leases and
sensor freshness checks remain in local snapshot/control APIs. B3's local
13-byte navigation-event queue also retains its sequence/time/generation; it
is an internal control interface, not external EDR telemetry. Do not narrow
those for control. EDR16 encoder counts wrap 4095 to 0; reverse count -1 becomes
4095. Inferring a signed delta is ambiguous beyond 2047 counts between received
observations, and reboot is indistinguishable from wrap. Telemetry is not an
odometry or movement-command input.

There is no wire timestamp, sequence number, validity bit or checksum in this
layout. Skip invalid samples; never send a fake zero for no-echo/startup. The
dashboard shows latest received values, not proof of fresh data. QoS 0 can drop
records; older queued readings can arrive after reconnect. Motor status/fault
and left/right readings are not delivered atomically. Keep control/safety on
local interfaces. The firmware retains legacy text helper APIs, but the dashboard subscribes only
to the EDR16 topic and ignores individual scalar/event topics. Free-text events
and derived values need a future schema extension; they cannot fit unassigned
readings into the current fully allocated ID table. Explicit browser imports
and demo data remain separate from MQTT.

## Verification

`check_data.cmd` / `check_data.sh` run the Python codec/dashboard tests, C codec
and B1 queue tests, then all eight B2 mock-hardware/watchdog profiles. They need
a native GCC/Clang compiler. Python tests cover all 65,536 words plus malformed
lengths, ranges, fixed byte vectors and receiver routing. See
`testing/data_representation/TEST_LOG.md` for what was actually run.
