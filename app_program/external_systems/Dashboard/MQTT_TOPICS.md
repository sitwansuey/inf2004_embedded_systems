# MQTT topics and dashboard fields

The topic selects the decoder. All publications below use QoS 0 and are
non-retained. The dashboard subscribes to telemetry, events and replies;
the robot subscribes to commands.

| Topic | Publisher → receiver | Payload |
|---|---|---|
| `Robopico/sensors/edr16/v1` | Robot → dashboard | One two-byte EDR16 telemetry record |
| `Robopico/events/edr16/v1` | Robot → dashboard | One two-byte event record |
| `csc02/command` | Dashboard → robot | Six-byte command |
| `csc02/command/ack` | Robot → dashboard | Six-byte reply |

## Telemetry fields

Each record is little-endian: `id = word & 15`, `data = word >> 4`.
`packed16.py` maps it to local display keys. Those keys do not cause extra
subscriptions or publications. The receiver ignores retained messages.

| ID | Dashboard card | Field / conversion |
|---|---|---|
| 0 | Vehicle controller, navigation and terrain | Mission-status subtype; see the subtype table below and [DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md) |
| 1 | Left wheel encoder | `encoder_left`, count modulo 4096 |
| 2 | Right wheel encoder | `encoder_right`, count modulo 4096 |
| 3 | Servo Motor | `servo_angle_deg`, commanded angle 0–180° |
| 4 | Ultrasonic Ranger | `distance_cm`, whole centimetres |
| 5 | Left IR | `ir_left`, raw 0–4095 |
| 6 | Right IR | `ir_right`, raw 0–4095 |
| 7 | Barcode IR | `ir_barcode`; digital 0/1 with `BOARD_IR_MODE` 3 (the mission setting) |
| 8–10 | Magnetometer | `mag_x_ut`, `mag_y_ut`, `mag_z_ut`: `(data - 2048) / 10` µT |
| 11 | Motor control | `direction`, `power_percent`, `state` |
| 12 | Motor control, Accelerometer | `fault_flags`; timeout `0x20`, motor error `0x40`, IMU fault `0x04` |
| 13–15 | Accelerometer | `accel_x_mps2`, `accel_y_mps2`, `accel_z_mps2`: `(data - 2048) * 9.80665 / 1000` m/s² |

Each 55-byte snapshot is published as up to 33 two-byte records. The telemetry
queue carries impact and IMU-fault bits across replaced or abandoned
snapshots so that the next snapshot can still report them.

ID 11 uses `command = data & 7`, `power = (data >> 3) & 127` and
`state = data >> 10`. Commands above 6, power above 100 and states above 2
are rejected. ID 12 accepts only bits 0–6. Zero bytes are valid.

ID 0 stores the subtype in data bits 11–8 and its value in bits 7–0. Selected
local keys are:

| Subtype / bits | Local display key | Meaning |
|---|---|---|
| 0, low nibble | `Robopico/Actuators/vehicle_controller/state` | Phase 0–13; 13 = Reverse |
| 0, high nibble | `Robopico/Actuators/vehicle_controller/fault_code` | Fault 0–8; 8 = lifted |
| 8, value 0–100 | `Robopico/sensors/ultrasonic_ranger/width_cm` | Total width, value × 0.5 cm |
| 8, value 128–191 | `Robopico/sensors/ultrasonic_ranger/width_left_cm` | Left half-width, `(value & 63) × 0.8 cm` |
| 8, value 192–255 | `Robopico/sensors/ultrasonic_ranger/width_right_cm` | Right half-width, `(value & 63) × 0.8 cm` |
| 9, bits 0–5 | `Robopico/Actuators/vehicle_controller/quality` | Bit 0 width valid, 1 impact, 2 armed, 3 bypass validated (build setting), 4 odometry valid, 5 servo status valid |
| 9, bits 6–7 | `Robopico/Actuators/vehicle_controller/fault_cause` | Fault 1: other, Sonar, Tilt, Odometry. Fault 2: other, line data stale, S3 fault event, line not found |
| 12 | `Robopico/Actuators/vehicle_controller/dropped` | Wire counter modulo 256; receiver displays differences over 60 s |
| 15, bits 6–7 | `Robopico/Actuators/servo_motor/bypass_side` | None, Left or Right |

Quality bit 2 also drives
`Robopico/Actuators/vehicle_controller/safety_state` (Armed/Stopped), and
bit 1 drives `Robopico/sensors/accelerometer/impact`.
A side-check or forward-pan timeout reports fault cause Sonar. Cause 0 with
fault 1 means another check, for example an avoidance step with an invalid manoeuvre distance.

Subtype 8 values 101–127 and bypass side 3 are rejected. Displayed widths
use `s5_width_reported()`, which subtracts `S5_BEAM_HALF_DEG` (20°) from each
measured edge angle before calculating its half-width. The bypass plan keeps
the measured bounds and clearance. The obstacle-scanned event uses the same
corrected width, rounded to centimetres.

The receiver keeps widths and the chosen side as the last scan's results.
Clearing quality bit 0, reconnecting, robot silence or a later side of None
does not erase them. New scan results replace them. Other live fields are
cleared when unavailable or when the broker disconnects or reconnects. The
browser marks readings stale after 5 s.

## Event topic

`Robopico/events/edr16/v1` uses the same two-byte shape but a separate ID
space. Decode with `decode_event()` in `tools/edr_decode.py`. The event value is
the upper 12 bits. The source of the layout is
`app_program/shared/events.h`.

| ID | Event | Value layout |
|---|---|---|
| 1 | Barcode attempt | Bits 0–1 attempt (1 initial scan, 2 reverse re-read, 3 final slow pass); bits 2–6 bars and spaces read, 0–31 (a full `*X*` is 29); bits 7–11 letter, 0 failed, 1–26 A–Z |
| 2 | Instruction | Bits 0–4 letter 1–26; bits 5–7 action 0 none, 1 left, 2 right, 3 straight, 4 U-turn; bits 8–11 zero |
| 3 | Hump | Bits 0–2 kind; bits 3–11 value. Kind 1: tilt in 0.1°. Kinds 2/3: S4 crest/completed height in 2 mm, zero if unavailable. Kinds 4/5: controller crest/peak estimate in 2 mm |
| 4 | Obstacle | Bits 0–1 kind; bits 2–3 side or completion reason; bits 4–11 value. Kind 1: detected range cm. Kind 2 with side 1/2: scanned width cm; side 0: retry number 1–3. Kind 3: bypass heading degrees. Kind 0: elapsed seconds and completion reason |
| 5 | Broker link | Bits 0–1 kind: 1 reconnected, 2 first connection; bits 2–11 seconds, saturated at 1023 (kind 1: since the first publish the broker did not acknowledge; kind 2: since boot) |

Obstacle sides are 1 left and 2 right. For completion kind 0, bits 2–3 mean
0 fault/STOP, 1 line found on return, 2 spot search, 3 obstacle went away.
The instruction event follows the attempt that decoded it. Crest and level
events use kinds 4/5 when the controller has a height estimate (wheel
distance and tilt) and kinds 2/3 with S4's height otherwise; the dashboard
marks estimate rows. The terrain telemetry heights still come from S4.

Robot events use S1's bounded buffer. After 3 s without a publish completing,
the network task holds queued events. Link events are held separately
and sent first after connection. QoS 0 and a full queue can still lose events.
The receiver adds its own offline/online, broker, controller-fault and
impact/IMU-fault rows and keeps the last 200 rows. Impact and IMU-fault flags
remain visible for 2 s.

Full bit layout, validation and limits:
[DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md).

## Command topic

Requests on `csc02/command` are six bytes: `C1 opcode b2 b3 b4 seq`.
Replies on `csc02/command/ack` are `C2 seq opcode result phase fault`.
Byte values in the frame examples below are hexadecimal.

| Opcode | Request | Meaning |
|---|---|---|
| 0 | `C1 00 00 00 00 SS` | STOP |
| 1 | `C1 01 DD L0 L1 SS` | Timed ARM: demand 1–50 decimal; lease 1–1000 ms in little-endian bytes L0/L1 |
| 2 | `C1 02 00 00 00 SS` | RESET while stopped |
| 3 | `C1 03 14 00 00 SS` | Dashboard START at demand 20 decimal; protocol accepts 1–50 |
| 4 | `C1 04 LL AA 00 SS` | MAP letter LL to action AA |

MAP uses letter 1–26 for A–Z and action 0 Ignore, 1 Left, 2 Right, 3 Forward,
4 U-turn. Letter/action 0/0 restores A U-turn, F Left and Z Right. MAP changes
future decodes in RAM and is accepted in any phase. **Send all** sends 26
acknowledged mappings from the receiver's table without a preceding reset.

Reply results are 0 accepted, 1 rejected, 2 RESET needed before ARM/START,
and 3 expired. Phase is 0–13 and fault is 0–8. Queued commands aged 100 ms
or more expire. STOP bypasses the command queue; commands superseded by a
handled STOP are rejected. A six-byte frame that fails the command checks
(for example an out-of-range MAP pair) or finds the command queue full gets
result 1; frames of any other length get no reply. Coalesced STOPs each
receive an accepted reply. A full reply buffer drops its oldest reply and
counts the drop.

The dashboard uses sequence numbers 0–127. USB frames set bit 7; their replies
also appear here and are ignored by the dashboard. USB keys `s`, `a`, `r`
and `g` use this same command path; USB ARM/START demand is 30 decimal and
ARM's lease is 500 ms. The dashboard does not send ARM. After an MQTT
connection or a received MQTT STOP, the network task gates ARM/START
until RESET has been passed to the controller's command queue. Controller
readiness is checked separately and can still reject a command.
