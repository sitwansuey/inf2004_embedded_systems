# One MQTT topic, multiple dashboard fields

**Subscribed wire topic:** `Robopico/sensors/edr16/v1` (QoS 0)

Python owns the MQTT TCP connection. The React/NexusUI browser app polls the
same-origin `/api/telemetry` endpoint; it needs no MQTT WebSocket connection or
additional broker subscriptions.

Each payload is exactly two bytes, low byte first. `(word & 15)` identifies the
reading; `(word >> 4)` supplies the data. The receiver validates before storing
any values. All mapping below happens locally after unmarshalling; these are
not additional MQTT subscriptions or publications.

| ID | Dashboard reading | Display fields / conversion |
|---|---|---|
| 1 | Left wheel encoder | `encoder_left`, count modulo 4096 |
| 2 | Right wheel encoder | `encoder_right`, count modulo 4096 |
| 3 | Servo motor | `servo_angle_deg`, 0-180 degrees |
| 4 | Ultrasonic ranger | `distance_cm` |
| 5 | Left IR | `ir_left`, raw |
| 6 | Right IR | `ir_right`, raw |
| 7 | Barcode IR | `ir_barcode`, raw |
| 8-10 | Magnetometer | `mag_x_ut` / `mag_y_ut` / `mag_z_ut` = (data - 2048) / 10, microtesla |
| 11 | Motor control | `direction`, `power_percent`, `state` |
| 12 | Motor control | `fault_flags`, bitmask 0-127 |
| 13-15 | Accelerometer | `accel_x_mps2` / `accel_y_mps2` / `accel_z_mps2` = (data - 2048) * 9.80665 / 1000, m/s^2 |

`packed16.py` preserves local display keys such as
`Robopico/sensors/left_encoder/encoder_left` and
`Robopico/Actuators/motors/power_percent`. The receiver does not subscribe to
those keys. Topic names and display keys are case-sensitive.

ID 11 extracts command = data & 7, power = (data >> 3) & 127, state = data >> 10.
Validation rejects command > 6, power > 100 and state > 2. Direction values are
`STOP`, `FORWARD`, `REVERSE`, `LEFT`, `RIGHT`, `PIVOT_RIGHT` and `PIVOT_LEFT`;
states are `STOPPED`, `MOVING` and `FAULT`. Commanded power describes PWM output,
not measured wheel speed. ID 11 expands into three display fields, producing
17 supported fields across the 15 IDs.

Zero-valued readings are valid when their ID permits them. ID zero, wrong
lengths, invalid servo angles and invalid fault masks are rejected. The prior
`packed16/v1` topic, individual sensor topics and JSON/event topics are not
consumed.

The HTTP API stores the latest value per field, its receiver sequence and its
desktop `received_at` timestamp in Unix seconds. The frontend converts that
reception time to Unix milliseconds for telemetry history. Chart history reflects
what the browser observes from latest-value polling; intermediate MQTT readings
can be coalesced. EDR16 does not carry a robot sampling timestamp, so reception
age must not be presented as sensor measurement age. Changing `receiver_id`
indicates a receiver restart and resets browser live history and sequence
tracking.

Demo/JSON import remain explicit browser-only modes and do not publish MQTT
messages. Supported physical sensors not yet connected remain unavailable.
Measured speed, absolute heading, pitch/roll, terrain estimates, decoded barcode
text and controller events are absent from this schema. Displaying them from
live telemetry requires a future schema change; all nonzero IDs are currently
used.

Full bit layout: [DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md).
