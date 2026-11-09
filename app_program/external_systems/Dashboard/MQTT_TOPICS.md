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
| 12 | Motor control | fault_flags, bitmask 0–127 |
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
| 8, 0–100 | `Robopico/sensors/ultrasonic_ranger/width_cm` | value × 0.5 cm (total width) |
| 8, 128–191 | `Robopico/sensors/ultrasonic_ranger/width_left_cm` | (value & 63) × 0.8 cm, centre to left edge |
| 8, 192–255 | `Robopico/sensors/ultrasonic_ranger/width_right_cm` | (value & 63) × 0.8 cm, centre to right edge |
| 9, bits 5–0 | `Robopico/Actuators/vehicle_controller/quality` | value & 63 (also `safety_state` bit 2, `accelerometer/impact` bit 1) |
| 9, bits 7–6 | `Robopico/Actuators/vehicle_controller/fault_cause` | `None/other`, `Sonar`, `Tilt`, `Odometry` (non-zero only while fault code 1, sensor; with fault code 1, `None/other` means an avoidance-step check such as a side-check timeout) |
| 15, bits 7–6 | `Robopico/Actuators/servo_motor/bypass_side` | `None`, `Left`, `Right` |

Subtype 8 values 101–127 and bypass side 3 are rejected. The three width
fields are retained between reports while the stream is live and quality
bit 0 is set; a quality record with bit 0 clear, or a disconnect, clears them.

The prior packed16/v1 topic, individual sensor topics and JSON/event topics
are not consumed. Demo/JSON import remain explicit browser-only features.
Supported physical sensors not yet connected remain blank. ID zero supplies decoded navigation, height/maximum, controller state and
other documented derived fields. Free-text events are not transmitted.

Full bit layout: [DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md).
