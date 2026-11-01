# Buddy 1: integrated telemetry and commands

`task_telemetry.c` samples copied owner snapshots at 5 Hz, including controller,
encoder, IR/barcode, IMU/terrain, servo and ultrasonic data. The private 50-byte
codec preserves internal precision and timestamp/availability. A two-slot RTOS
mailbox copies these records to the existing single network owner.

`telemetry_edr.h` translates that snapshot to independent uint16 readings:
low four bits identify the source, upper twelve bits hold data. Each MQTT
publication contains exactly two little-endian bytes on
`Robopico/sensors/edr16/v1`. IDs 1/2 are the wheel encoders; ID 0 carries mission
status subtypes. No old envelope or 50-byte aggregate is published.

The network service sends at most four messages per poll and retries the same
record on backpressure, discarding observations older than 500 ms. It owns all
lwIP/MQTT calls and reconnect timers. Periodic state/sequence is the heartbeat.

Commands arrive on `csc02/command` or the USB hook, using the shared checked
six-byte command format. STOP bypasses a full queue. Timed ARM remains for
supervised tests; START runs an onboard mission without continued remote input.
Local sensor faults, watchdogs and STOP still apply. Default motor/calibration
locks remain enabled. See `docs/MISSION_SETUP.md` before physical operation.

The dashboard subscribes only to the EDR16 topic, unmarshals each record and
routes it to local display keys. Status flags and one-second per-field expiry
prevent unavailable/stale observations from looking current. Detailed units
and subtype mappings are in `docs/DATA_REPRESENTATION.md`.

Tests: `python -B testing/data_representation/run.py --compiler clang` and
`python -B testing/integration/run.py --compiler clang`. GCC is also supported.
The latter includes production C adapter -> Python -> browser routing checks.
