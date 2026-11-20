# Buddy 1: integrated telemetry and commands

`task_telemetry.c` samples copied owner snapshots at 5 Hz, including controller,
encoder, IR/barcode, IMU/terrain, servo and ultrasonic data. The private 55-byte
codec preserves internal precision and timestamp/availability. A two-slot RTOS
message buffer copies these records to the existing single network owner.

`telemetry_edr.h` translates that snapshot to independent uint16 readings:
low four bits identify the source, upper twelve bits hold data. Each MQTT
publication contains exactly two little-endian bytes on
`Robopico/sensors/edr16/v1`. IDs 1/2 are the wheel encoders; ID 0 carries mission
status subtypes. No old envelope or 55-byte aggregate is published.

ID 12 carries the shared fault bits: motor timeout 0x20 / error 0x40, and the
IMU fault 0x04 when any vehicle-controller S4 result since the last snapshot
had one (I/O error, sample older than 100 ms, clipped or overrun sample,
IMU/encoder skew; a clipped sample is also an impact, quality bit 1). The VC
also counts them (`app_state_t.imu_fault_count`), so a fault in a tick this
task never read still sets the bit. Status subtype 12 is the internal drop
count modulo 256: it wraps like the sequence (subtype 11), so receivers count
drops as differences between records; every drop is one atomic increment
(both cores drop).

A new shock count sends an extra snapshot at once, at most one per 200 ms
(`S1_SHOCK_EXTRA_MS`); fault, width and side changes are never limited. A
change counts as sent only once its snapshot is queued (`s1_binary_send`
E_OK), and the IMU fault only with a snapshot that carries ID 12 (motor
status available); otherwise the next 20 ms loop builds another snapshot.
A full two-slot queue drops its oldest snapshot, and the network takes the
newest one, dropping any older one; each drop is counted and hands its
impact and IMU fault bits to the newer snapshot.

The network service sends at most four messages per poll and retries the same
record on backpressure, discarding observations older than 500 ms. A snapshot
abandoned part-way is counted as dropped, and its unsent impact or IMU fault
goes on in the next one. It owns all lwIP/MQTT calls and reconnect timers.
Periodic state/sequence is the heartbeat.

Robot events (`app_program/shared/events.h`: barcode attempts, decoded
instructions, humps, obstacles, broker link) are separate from telemetry. The
vehicle controller calls `s1_event_post`, which copies one two-byte
EDR16-format record into a 160-byte message buffer (20 two-byte records with kernel overhead) without blocking (a full
buffer drops and counts it). The network owner takes one per poll with
`s1_event_take`, after command ACKs, and publishes it on
`Robopico/events/edr16/v1`. Once the broker has acknowledged nothing for 3 s
it leaves them queued, so a short link drop does not lose them. The broker
link event (EV_LINK) is held by the network owner itself and published first
after a (re)connection, so a queue full of outage events cannot drop it.

Commands arrive on `csc02/command` or the USB hook, using the shared checked
six-byte command format; USB sequences have bit 7 set, so their ACKs never
match a dashboard command (0..127). Every valid frame is answered: a command
queued before a STOP the VC then handled gets result 1 when the VC drains it,
and the first of two STOPs offered before the VC takes one gets result 0 at
once. Replies the network side makes (`s1_command_reply`: RESET needed, queue
full) carry the VC's last published phase and fault. A full ACK buffer drops
its oldest reply. STOP bypasses a full queue. Timed ARM remains for
supervised tests; START runs an onboard mission without continued remote input.
MAP (opcode 4) changes which turn a decoded barcode letter requests, in RAM.
Local sensor faults, watchdogs and STOP still apply. `app_program/robot_config.h`
holds the robot's mission profile (motors enabled, measured calibration); the
firmware still boots DISARMED and drives only after RESET then START (or a
timed ARM).

The dashboard subscribes to the EDR16 telemetry topic, the robot event topic
and the command ACK topic; it unmarshals each telemetry record and routes it to
local display keys, and turns each event record into an event-log row. Status flags hide unavailable observations;
the latest value is kept and marked stale after 5 s without an update. Detailed units
and subtype mappings are in `docs/DATA_REPRESENTATION.md`.

Tests: `python -B testing/data_representation/run.py --compiler clang` and
`python -B testing/integration/run.py --compiler clang`. GCC is also supported.
The latter includes production C adapter -> Python -> browser routing checks.

S4's 71-byte acquisition cache includes monotonic impact and accel-fault
counters. The VC counts every increment since its last read, plus local
copy/skew faults. It clears incompatible tilt/height validity before packing
a latched fault or impact. The wire format remains unchanged. Hump level
events use the completed-height latch for that traversal, or zero.
