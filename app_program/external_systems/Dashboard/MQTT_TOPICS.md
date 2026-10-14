# Dashboard MQTT topics

Subscribe to both filters (case-sensitive):

```text
Robopico/sensors/#
Robopico/Actuators/#
```

Each dashboard section has its own subtopic, with one leaf topic for each value:

| Section | Topic prefix |
| --- | --- |
| Left IR | `Robopico/sensors/left_ir` |
| Right IR | `Robopico/sensors/right_ir` |
| Barcode IR | `Robopico/sensors/barcode_ir` |
| Left wheel encoder | `Robopico/sensors/left_encoder` |
| Right wheel encoder | `Robopico/sensors/right_encoder` |
| Accelerometer | `Robopico/sensors/accelerometer` |
| Magnetometer | `Robopico/sensors/magnetometer` |
| Ultrasonic Ranger | `Robopico/sensors/ultrasonic_ranger` |
| Terrain & hump analysis | `Robopico/sensors/terrain` |
| Servo Motor | `Robopico/Actuators/servo_motor` |
| Motor control | `Robopico/Actuators/motors` |
| Vehicle controller | `Robopico/Actuators/vehicle_controller` |

The controller is grouped under Actuators as control status. These topics carry status, not movement
commands. Each exact leaf topic appears below its reading on the dashboard and
in `window.dashboard.topicBindings`.

## Payloads

Publish one scalar per leaf: a number, boolean, text, or `null` for unavailable.
Units appear in field names. For example:

| Topic | Example payload |
| --- | --- |
| `Robopico/sensors/left_ir/ir_left` | `650` |
| `Robopico/sensors/barcode_ir/barcode` | `RIGHT` |
| `Robopico/sensors/accelerometer/accel_x_mps2` | `0.12` |
| `Robopico/sensors/ultrasonic_ranger/distance_cm` | `32` |
| `Robopico/Actuators/servo_motor/servo_angle_deg` | `60` |
| `Robopico/Actuators/motors/power_percent` | `35` |

Strings that resemble JSON scalars should be JSON-quoted to preserve their type,
for example barcode `"001"`. Event messages use
`Robopico/Actuators/vehicle_controller/events` with a JSON object containing
`time`, `source`, `level`, and `message`.

## Dashboard adapter

The Python MQTT receiver automatically forwards messages to the browser. Manual routing tests can call:

```javascript
window.dashboard.receiveMqtt(topic, payload);
```

Updates merge into the displayed MQTT data without clearing other readings.
Unknown topics return `false`; recognized messages return `true`. Invalid reading
objects/arrays throw an error. Demo, reset and JSON import clear accumulated MQTT
data, so sample values cannot masquerade as readings after the first MQTT update.

To test the routing in the browser console without a broker:

```javascript
dashboard.receiveMqtt('Robopico/sensors/left_ir/ir_left', '650');
dashboard.receiveMqtt('Robopico/Actuators/servo_motor/servo_angle_deg', '60');
```

Run `dashboard.py` to subscribe automatically to localhost:1883 and serve the
browser at http://127.0.0.1:8001. Install `requirements.txt` first. Use `--broker`
and `--mqtt-port` for other addresses. Firmware still needs to publish sensor
values on these topics; existing firmware heartbeats are unchanged. Browser
updates poll latest readings every 250 ms. Samples superseded between polls are
not plotted or archived. The receiver caches readings for new browser tabs;
publish with retain if new receiver processes also need the latest values.
