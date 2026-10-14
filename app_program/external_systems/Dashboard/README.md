# Buddy 1 dashboard shell

For live MQTT updates, run from the repository root:

```powershell
python -m pip install -r app_program/external_systems/requirements.txt
python app_program/external_systems/dashboard.py --broker localhost --mqtt-port 1883
```

Then visit http://127.0.0.1:8001. The server subscribes automatically to
`Robopico/sensors/#` and `Robopico/Actuators/#`. It reconnects and resubscribes
after a disconnect. Start your broker separately. Authentication uses optional
`MQTT_USERNAME` and `MQTT_PASSWORD` environment variables. The page polls the
receiver every 250 ms for latest values, not every individual intermediate sample.
It displays received status and measurements; it does not send movement commands.
Opening `index.html` directly supports demo/import only.
Load demo is explicitly synthetic; Reset clears it. New MQTT updates replace demo
data. Refresh reloads the receiver’s cached latest readings.

## Display contract

MQTT values use individual topics below `Robopico/sensors` and
`Robopico/Actuators`. See `MQTT_TOPICS.md` for section prefixes, payloads and the
`window.dashboard.receiveMqtt(topic, payload)` routing interface.

The dashboard has separate sections for Left IR, Right IR and Barcode IR sensors, each wheel
encoder, the accelerometer, magnetometer, and Ultrasonic Ranger.
Additional sections show terrain/hump analysis,
Servo Motor, motor commands and the vehicle controller.
Barcode IR uses `navigation.ir_barcode` for its raw reading and
`navigation.barcode` for decoded data.
Each card reports how many of its fields were supplied in the current snapshot.
This indicates snapshot completeness, not a live connection or sensor freshness.
Optional magnetometer fields are `terrain.mag_x_ut`, `mag_y_ut`, `mag_z_ut` in µT;
their calibration and scaling must be agreed during firmware integration.

Import JSON accepts a snapshot with optional `network`, `motion`, `navigation`,
`terrain`, `obstacle`, `system` objects and an `events` array. Missing values show
an em dash, including unavailable measurements. Units are in field names.
The exact field list is in `dashboard.js`. This is a proposed dashboard contract,
not a format already published by the firmware. Example:

```json
{
  "motion": {"state": "MOVING", "direction": "FORWARD", "power_percent": 35},
  "terrain": {"valid": false},
  "system": {"state": "IDLE", "fault_code": 0},
  "events": [{"time": "00:12", "source": "Buddy 1", "level": "INFO", "message": "Connected"}]
}
```

Manual snapshots can also be supplied through
`window.dashboard.setTelemetry(packet)`. This currently labels them as imported
snapshots; add real connection/freshness handling when implementing live transport.
Only the newest 100 supplied events are displayed. Import replaces the snapshot.
The displayed receive time is the laptop time, not a robot measurement timestamp.
Latency must be supplied by a future measured transport; it is not calculated here.

## Comparison with the Week 6 review

The review defines Buddy 1 as WiFi communication, commands, telemetry and heartbeat
recovery (FR12), with the dashboard displaying speed, direction, sensor data and
robot status (FR13). Its external data list also includes line/barcode, IMU
orientation and validity, motion/impact, hump peaks, obstacle geometry/bypass,
and system fault/state codes. The shell covers these categories.

The checked-in implementation only partly matches that design:

| Area | Current evidence | Remaining work |
| --- | --- | --- |
| Buddy 1 | `app_program/subsystem1/` is the GP19 LED heartbeat example | Application telemetry, command routing and communication recovery |
| MQTT | `mqtt_utk.c` sends `hb <count>` and subscribes to its own topic | Agree telemetry and command topics/payloads; bridge subsystem data |
| Buddy 2 | Open-loop PWM, expiring commands and status API | Encoders, measured speed/distance and PID remain pending per its README |
| Buddies 3 / 4 / 5 | Statistics consumer, console monitor and empty template | Planned line/barcode, IMU and obstacle/orchestrator implementations |
| Ownership | `TEAM.md` still lists example assignments | Record actual owners, pins, priorities and interfaces |
| RTOS priority | This repository uses micro T-Kernel: 1 is highest | Review document assumes FreeRTOS higher-number-is-higher priorities; reconcile before implementation |
| Network ownership | `TEAM.md` assigns radio/lwIP/MQTT to `cyw43_utk_task` | Application tasks must exchange queued data with that service; do not call MQTT directly from a new task |
| Timing | Review targets 10 ms telemetry and delivery within 1 s | Measure firmware-to-dashboard timing after integration; shell does not prove these targets |

Some retained example consumers still reference the old producer interface while
Subsystem 2 has changed to motors. This review does not establish that the whole
firmware builds; resolve the shared interfaces and run the board build separately.
The dashboard work changes no firmware or shared ownership files.

The template's submission and AI-review prompts were treated as document content,
not instructions to execute during this task.
