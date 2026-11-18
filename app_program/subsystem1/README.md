# Buddy 1: integrated telemetry and commands

`task_telemetry.c` samples copied owner snapshots at 5 Hz, including controller,
encoder, IR/barcode, IMU/terrain, servo and ultrasonic data. The private 54-byte
codec preserves internal precision and timestamp/availability. A two-slot RTOS
message buffer copies these records to the existing single network owner.

`telemetry_edr.h` translates that snapshot to independent uint16 readings:
low four bits identify the source, upper twelve bits hold data. Each MQTT
publication contains exactly two little-endian bytes on
`Robopico/sensors/edr16/v1`. IDs 1/2 are the wheel encoders; ID 0 carries mission
status subtypes. No old envelope or 54-byte aggregate is published.

The network service sends at most four messages per poll and retries the same
record on backpressure, discarding observations older than 500 ms. It owns all
lwIP/MQTT calls and reconnect timers. Periodic state/sequence is the heartbeat.

Robot events (`app_program/shared/events.h`: barcode attempts, decoded
instructions, humps, obstacles, broker link) are separate from telemetry. The
vehicle controller, and the network owner for its link events, call
`s1_event_post`, which copies one two-byte EDR16-format record into a 16-entry
message buffer without blocking (a full buffer drops and counts it). The
network owner takes one per poll with `s1_event_take`, after command ACKs, and
publishes it on `Robopico/events/edr16/v1`. Once the broker has acknowledged
nothing for 3 s it leaves them queued, so a short link drop does not lose them.

Commands arrive on `csc02/command` or the USB hook, using the shared checked
six-byte command format. STOP bypasses a full queue. Timed ARM remains for
supervised tests; START runs an onboard mission without continued remote input.
MAP (opcode 4) changes which turn a decoded barcode letter requests, in RAM.
Local sensor faults, watchdogs and STOP still apply. `app_program/robot_config.h`
holds the robot's mission profile (motors enabled, measured calibration); the
firmware still boots DISARMED and drives only after RESET then START (or a
timed ARM). See `docs/MISSION_SETUP.md` before physical operation.

The dashboard subscribes to the EDR16 telemetry topic, the robot event topic
and the command ACK topic; it unmarshals each telemetry record and routes it to
local display keys, and turns each event record into an event-log row. Status flags hide unavailable observations;
the latest value is kept and marked stale after 5 s without an update. Detailed units
and subtype mappings are in `docs/DATA_REPRESENTATION.md`.

Tests: `python -B testing/data_representation/run.py --compiler clang` and
`python -B testing/integration/run.py --compiler clang`. GCC is also supported.
The latter includes production C adapter -> Python -> browser routing checks.
