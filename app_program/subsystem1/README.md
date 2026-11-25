# Subsystem 1: telemetry, events and commands

S1 samples the robot's state, queues telemetry and events, accepts commands,
and drives the heartbeat LED. See [TEAM.md](../../TEAM.md) for ownership,
tasks, pins, interfaces IF-03, IF-08, IF-10 and IF-12 to IF-21, and the
kernel object budget.
The [data table in TEAM.md](../../TEAM.md#data-to-the-dashboard-through-s1)
lists each owner's data and the interface S1 reads. Every dashboard value
goes through S1 and the MQTT broker. S1 also writes the telemetry-loop
watchdog heartbeat (IF-20). The dashboard has its own [run guide](../external_systems/Dashboard/README.md).

## Telemetry path

`task_telemetry.c` reads copied controller, motor, encoder, IR/barcode,
IMU/terrain and sonar/servo status. It normally builds a snapshot every
200 ms. New controller faults, widths and bypass sides trigger extra
snapshots. New shocks can trigger extras at most once per 200 ms
(`S1_SHOCK_EXTRA_MS`).

`telemetry_codec.c` packs 433 data bits into a private 55-byte record with
seven zero padding bits. `s1_binary_send()` copies it into a two-snapshot
message buffer. `telemetry_edr_record()` in `telemetry_edr.h` converts it into
up to 33 independent two-byte EDR16 records. The private snapshot is never
sent over MQTT.

The network task in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` publishes
those records on `Robopico/sensors/edr16/v1`. It sends at most four telemetry
records per poll, after a pending command reply and one event. It retries
the same telemetry record when the network cannot accept it. An unfinished
snapshot that reaches 500 ms is dropped and counted.

A full telemetry queue drops its oldest snapshot. `s1_telemetry_take()`
selects the newest current one. Discarded snapshots pass impact and IMU-fault
flags to the newer snapshot; the network task also carries unsent flags
from an abandoned snapshot. The drop counter is incremented atomically.
Status subtype 12 sends its low eight bits, so receivers count differences
modulo 256. Subtype 11 is the observation sequence modulo 256.

ID 12 carries motor timeout `0x20`, other motor error `0x40`, and IMU fault
`0x04` (an S4 fault, a failed copy, a stale or unusable sample, a clipped or
overrun sample, or IMU/encoder skew). S4's counters and the controller's
fault count preserve short events between telemetry reads. The IMU-fault latch clears only after a queued
snapshot with motor availability, which allows ID 12 to be sent. A clipped
accelerometer sample is also an impact. Availability and quality flags tell
the receiver which observations are usable. The controller clears tilt and
height validity when a result carries an IMU fault or impact, and clears
height for any fault. Status subtypes 0 (state) and 11 (sequence) are sent
in every snapshot and act as the periodic heartbeat.

## Robot events

`shared/events.h` defines two-byte records for barcode attempts, decoded
instructions, humps and obstacles. The controller posts them through
`s1_event_post()`, which never blocks. The event buffer is sized as
`S1_EVENT_CAPACITY` (16) x 10 bytes = 160 bytes; the kernel stores each
two-byte message in 8 bytes, so it actually holds 20. A full buffer drops
new events and counts them.

The network task takes one event per poll with `s1_event_take()` and
publishes it on `Robopico/events/edr16/v1`. After 3 s without a successful
publish completion, it holds queued events until the link recovers. This is
a bounded queue, not a guarantee of delivery. The network task creates broker
link events itself and holds the latest one separately, sending it before
queued robot events after a connection.

Hump events distinguish the controller's wheel-distance/tilt estimate
(kinds 4 and 5) from the S4 fallback (kinds 2 and 3). The terrain
telemetry fields still use the height from S4's terrain code, which the
controller runs. Obstacle events also report scan retries.
The full layouts and units are in
[DATA_REPRESENTATION.md](../../docs/DATA_REPRESENTATION.md).

## Commands

Commands arrive on `csc02/command` or through `app_usb_rx_byte()` in `usb_control.c`.
Both use the checked six-byte format in `network_bridge.h`. USB keys are
`s` STOP, `a` ARM at 30% with a 500 ms lease, `r` RESET and `g` autonomous
START at 30%. USB sequence numbers have bit 7 set; the dashboard uses 0–127.

The four-entry command buffer holds commands for the controller. STOP uses
a separate atomic word and bypasses a full buffer. Commands queued before
a handled STOP are rejected with result 1. The controller rejects queued
commands aged 100 ms or more. ARM uses a lease; START runs onboard without continued
remote input. MAP changes future barcode actions in RAM.

Replies on `csc02/command/ack` carry sequence, opcode, result, controller
phase and fault. Coalesced STOPs each receive an accepted reply (result 0);
the first gets its reply as soon as a later STOP replaces it. A full reply
buffer drops its oldest reply and counts it. Network-generated replies use
the last published controller phase and fault.

After an MQTT (re)connection or a STOP received over MQTT, the network task
answers ARM and START with "RESET needed" until a RESET has been
passed to the controller's command queue. A USB STOP does not set this
gate. See [MQTT_TOPICS.md](../external_systems/Dashboard/MQTT_TOPICS.md#command-topic)
for the frames and result codes.

## Verification

Run from the repository root with Python 3 and native GCC or Clang:

```sh
python -B testing/data_representation/run.py
python -B testing/integration/run.py
```

Both accept `--compiler PATH`. The first includes S1 queue/codec and dashboard
Python checks. The second includes the production snapshot-to-EDR16 path
through the Python receiver and browser field routing. Node.js is needed for
browser checks. See [testing/README.md](../../testing/README.md) for dependencies
and recording rules, and [S1's test record](../../testing/subsystem1/TEST_LOG.md)
for results tied to tested commits.
