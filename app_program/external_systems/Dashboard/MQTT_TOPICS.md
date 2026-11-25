# One MQTT topic, multiple dashboard fields

**Subscribed wire topic:** `Robopico/sensors/edr16/v1`

Each payload is exactly two bytes, low byte first. `(word & 15)` identifies the
reading; `(word >> 4)` supplies the data. The receiver validates before storing
any values. All mapping below happens locally after unmarshalling; these are
not additional MQTT subscriptions or publications.

| ID | Dashboard card | Display fields / conversion |
|---|---|---|
| 0 | Mission/terrain/navigation cards | subtype-specific status; see wire contract |
| 1 | Left wheel encoder | encoder_left, count modulo 4096 |
| 2 | Right wheel encoder | encoder_right, count modulo 4096 |
| 3 | Servo Motor | servo_angle_deg, 0–180 degrees |
| 4 | Ultrasonic Ranger | distance_cm |
| 5 | Left IR | ir_left, raw |
| 6 | Right IR | ir_right, raw |
| 7 | Barcode IR | ir_barcode, raw |
| 8–10 | Magnetometer | mag_x_ut / mag_y_ut / mag_z_ut = (data−2048)/10 |
| 11 | Motor control | direction, power_percent, state |
| 12 | Motor control, Accelerometer | fault_flags, bitmask 0–127: motor timeout 32 / error 64, IMU fault 4 (also `accelerometer/imu_fault`) |
| 13–15 | Accelerometer | accel_x_mps2 / accel_y_mps2 / accel_z_mps2 = (data−2048)*9.80665/1000 |

The browser keeps legacy-shaped display keys such as
`Robopico/sensors/left_encoder/encoder_left`. Their only purpose is selecting
fields in existing cards. The Python receiver does not subscribe to those keys.

ID 11 extracts command = data & 7, power = (data >> 3) & 127, state = data >> 10.
Validation rejects command >6, power >100 and state >2. Zero bytes are valid.
Wrong lengths, invalid servo angles and invalid fault masks are rejected.
ID zero carries the mission-status subtype and value.

ID 0 obstacle and scan fields (local display keys):

| Subtype / value | Display key | Conversion |
|---|---|---|
| 0, low nibble | `Robopico/Actuators/vehicle_controller/state` | phase name; 13 = `Reverse` |
| 0, high nibble | `Robopico/Actuators/vehicle_controller/fault_code` | controller fault 0–8; 8 = lifted (9–15 rejected) |
| 8, 0–100 | `Robopico/sensors/ultrasonic_ranger/width_cm` | value × 0.5 cm (total width) |
| 8, 128–191 | `Robopico/sensors/ultrasonic_ranger/width_left_cm` | (value & 63) × 0.8 cm, centre to left edge |
| 8, 192–255 | `Robopico/sensors/ultrasonic_ranger/width_right_cm` | (value & 63) × 0.8 cm, centre to right edge |
| 9, bits 5–0 | `Robopico/Actuators/vehicle_controller/quality` | value & 63 (also `safety_state` bit 2, `accelerometer/impact` bit 1) |
| 9, bits 7–6 | `Robopico/Actuators/vehicle_controller/fault_cause` | `None/other`, `Sonar`, `Tilt`, `Odometry`: the cause with fault code 1 (sensor; `None/other` means an avoidance-step check such as an invalid manoeuvre distance; side/pan timeouts use Sonar). With fault code 2 (line) the same codes mean 0 other, 1 line data stale (gap of 250 ms or more, or no TRACK handoff), 2 S3 fault event (or an S3 sampler latch that keeps coming back for 1 s although the controller re-sends TRACK every 100 ms), 3 line not found (the on-the-spot search after a turn, a bypass, or `BOARD_LINE_LOST_MM` of tracking with neither straddle IR on black); the browser shows those names. 0 with any other fault |
| 12 | `Robopico/Actuators/vehicle_controller/dropped` | the robot's internal drop count modulo 256 (wraps, like the subtype 11 sequence); the receiver shows the drops of the last 60 s from the differences between records. The first record after a broker reconnect or a robot silence is only a baseline |
| 15, bits 7–6 | `Robopico/Actuators/servo_motor/bypass_side` | `None`, `Left`, `Right` |

The robot reports the widths with each scanned edge reduced by the sonar beam's
half-angle (`S5_BEAM_HALF_DEG`, 20 deg): the HC-SR04 keeps hearing an obstacle's
corner about that far past its real edge. The bypass plan uses the measured
edges. The obstacle "scanned" event shows the same corrected width.

Subtype 8 values 101–127 and bypass side 3 are rejected. The three width
fields and the bypass side are the last obstacle scan's results: the receiver
keeps them until the next scan reports new ones. A quality record with bit 0
clear, a broker reconnect, a robot silence or a bypass side of `None` (no plan
now) does not clear them.

The prior packed16/v1 topic, individual sensor topics and JSON topics are
not consumed. Demo/JSON import remain explicit browser-only features.

## Event topic (Events tab)

**Subscribed:** `Robopico/events/edr16/v1`. One robot event per message, two
bytes, little-endian, in the same EDR16 shape (low nibble = event ID, high 12
bits = value); see `app_program/shared/events.h`. The robot uses a 160-byte event buffer (20 two-byte records with kernel
overhead), so events from a short link drop arrive after it reconnects: once the
broker has acknowledged nothing for 3 s the robot keeps them queued instead of
handing them to lwIP (only those handed over in the last ~3 s before an outage
can be lost). The link event (ID 5) is not queued with them: the robot holds
it separately and publishes it first after a (re)connection, so a queue full
of outage events cannot drop it.

| ID | Event | 12-bit value |
|---|---|---|
| 1 | Barcode attempt (one per finished read) | bits 0-1 attempt (1 initial scan, 2 reverse scan, 3 final attempt); bits 2-6 bars+spaces read (0..31, a full `*X*` is 29); bits 7-11 letter (0 failed, 1..26 = A..Z) |
| 2 | Instruction decoded | bits 0-4 letter 1..26; bits 5-7 turn (0 none/not mapped, 1 left, 2 right, 3 straight, 4 U-turn) |
| 3 | Hump | bits 0-2 kind (1 detected: tilt in 0.1 deg; 2 crest, slowing down: this hump's height so far in 2 mm; 3 level again: S4's completed height for this traversal in 2 mm, or 0 when no valid hump completed); bits 3-11 value |
| 4 | Obstacle | bits 0-1 kind (1 detected: distance cm; 2 scanned: width cm; 3 bypass started: turn-out heading deg; 0 avoidance over: seconds since detection); bits 2-3 side (1 left, 2 right) or, for kind 0, how (0 ended by a fault or STOP, 1 line found on the return, 2 on-the-spot search, 3 obstacle went away); bits 4-11 value |
| 5 | Link | bits 0-1 kind (1 reconnected to the broker after N s without it, counted from the first publish it never acknowledged; 2 first connection, N s after boot); bits 2-11 seconds |

An instruction (ID 2) follows the barcode attempt (ID 1) that decoded it.
The receiver keeps the last 200 rows and adds its own: "Robot offline" when
robot data stops for 3 s (Wi-Fi or broker link lost), "Robot back online",
"Robot stopped: fault N (name)" once per new controller fault code (status
subtype 0), "Impact reported" / "IMU fault reported" once per rising edge
(each is also shown true for 2 s), and its own broker link lost/restored.
Supported physical sensors not yet connected remain blank. ID zero supplies decoded navigation, height/maximum, controller state and
other documented derived fields. Free-text events are not transmitted.

Full bit layout: [DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md).

## Command topic (dashboard buttons only)

`dashboard.py` publishes non-retained QoS 0 frames on `csc02/command` and
waits for the matching reply on `csc02/command/ack`. Request (6 bytes):
`C1 opcode b2 b3 b4 seq`; ACK (6 bytes): `C2 seq opcode result phase fault`
(result 0 accepted, 1 rejected, 2 Start needs Reset, 3 expired; phase 0–13,
fault 0–8, 8 = lifted). Commands older than 100 ms on the robot are answered
with result 3. Every valid six-byte frame gets an ACK: a command queued
before a STOP the robot then handled is answered 1 (superseded), a full
command queue 1, and of two STOPs that arrive before the robot takes one the
first is answered 0 at once (the robot stops once, for both). Replies the
robot's network side makes itself carry the controller's last published
phase and fault. The dashboard numbers its frames 0–127; sequence bit 7
marks a command typed on the USB console, whose ACK is also published here
and is ignored by the dashboard. When no broker is reachable and the reply
buffer fills, the robot drops its oldest reply (counted in status subtype 12).

| Opcode | Request | Behaviour |
|---|---|---|
| 0 | `C1 00 00 00 00 SS` | STOP |
| 1 | `C1 01 DD LL LL SS` | timed ARM, demand 1–50, lease 1–1000 ms (not sent by the dashboard) |
| 2 | `C1 02 00 00 00 SS` | RESET while stopped |
| 3 | `C1 03 14 00 00 SS` | autonomous START, demand 20 |
| 4 | `C1 04 LL AA 00 SS` | MAP: barcode letter LL (1–26 = A–Z) performs action AA at the next junction |

Opcode 4 actions: 0 ignore (read, no turn), 1 left, 2 right, 3 forward,
4 U-turn. `LL = 0, AA = 0` restores the built-in codebook (A U-turn, F left,
Z right; others no turn) for every letter; any other out-of-range pair is
rejected (result 1). MAP is accepted in any phase and only changes future
decodes. Mappings live in robot RAM, so after a robot reboot press **Send all**
(it writes all 26 letters A..Z from the dashboard's table, one ACKed frame
each, with no codebook reset first).
The dashboard's `barcode_map` (in `/api/telemetry`) changes only on result 0.

A flat-top hump posts one detected/crest/level cycle. The level height is
latched at traversal completion, so a later jolt cannot substitute a stale
height. Plain descents have no hump height. S4 impact/fault counters cover
samples between controller reads. The two-slot telemetry queue selects the
newest snapshot and carries impact/IMU bits across discarded snapshots.
Shock-triggered extras are limited to one per 200 ms. These changes keep
the existing EDR16 layout and the 2 s dashboard flag hold.
