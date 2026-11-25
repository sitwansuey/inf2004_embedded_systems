# MQTT robot dashboard

The dashboard shows telemetry and robot events and sends RESET, START, STOP
and barcode MAP commands. Python receives MQTT and serves the page; the
browser polls `/api/telemetry`. See [TEAM.md](../../../TEAM.md) for ownership
and [MQTT_TOPICS.md](MQTT_TOPICS.md) for topics and command frames. Every
value shown reaches the dashboard through S1; see the
[data routes in TEAM.md](../../../TEAM.md#data-to-the-dashboard-through-s1).

## Run

From the repository root:

```powershell
python -m pip install -r app_program/external_systems/Dashboard/requirements.txt
python app_program/external_systems/Dashboard/dashboard.py --broker localhost --mqtt-port 1883
```

Open <http://127.0.0.1:8001>. Start the MQTT broker separately. Replace
`localhost` with its address if it runs on another computer. The robot's
`config/wifi_secrets.h` must use the same reachable broker; see the
[root build guide](../../../README.md). Keep the repository layout because
`packed16.py` imports `tools/edr_decode.py`.

`--port` changes the HTTP port; the server binds to `127.0.0.1`. Robot
commands are accepted only from the page opened at `http://127.0.0.1:<port>/`
or `http://localhost:<port>/`. The page uses local HTML, CSS and JavaScript
assets and needs no frontend build step or CDN.

To check the receiver without a robot (broker running, from the repository root):

```python
from paho.mqtt import publish
from tools.edr_decode import MQTT_TOPIC, encode_packet
publish.single(MQTT_TOPIC, payload=encode_packet(1, 100), hostname="localhost", port=1883)
```

The Left wheel encoder card then shows 100. Publish the two raw bytes `41 06`,
not the text "4106". If the broker requires a login, pass its username and
password in the `auth` argument to `publish.single()`.

## Broker access

The dashboard reads `MQTT_USERNAME` and `MQTT_PASSWORD` from the environment.
`--username` can override the username; the password still comes from
`MQTT_PASSWORD`. The robot uses optional macros with the same names in its
secrets file. Without credentials, both can connect anonymously if the
broker allows it.

The firmware does not check who published a command. On an open broker any
device on the same network can publish RESET and then START on
`csc02/command`. To prevent this, make the broker require a login and let
only the dashboard account publish commands. Until you do, the robot and the
dashboard connect anonymously.

| Account | Publish | Subscribe |
|---|---|---|
| Dashboard | `csc02/command` | `csc02/command/ack`, `Robopico/sensors/edr16/v1`, `Robopico/events/edr16/v1` |
| Robot | `csc02/command/ack`, `Robopico/sensors/edr16/v1`, `Robopico/events/edr16/v1` | `csc02/command` |

With mosquitto 2.x:

1. Create a password file with one account for each (`-c` creates the file,
   so use it only the first time):

   ```powershell
   mosquitto_passwd -c C:\mosquitto\passwd dashboard
   mosquitto_passwd C:\mosquitto\passwd robopico
   ```

2. Create `C:\mosquitto\acl` with the permissions in the table:

   ```text
   user dashboard
   topic write csc02/command
   topic read csc02/command/ack
   topic read Robopico/sensors/edr16/v1
   topic read Robopico/events/edr16/v1

   user robopico
   topic read csc02/command
   topic write csc02/command/ack
   topic write Robopico/sensors/edr16/v1
   topic write Robopico/events/edr16/v1
   ```

3. Add these lines to `mosquitto.conf` and restart the broker:

   ```text
   listener 1883
   allow_anonymous false
   password_file C:\mosquitto\passwd
   acl_file C:\mosquitto\acl
   ```

4. Robot: set `MQTT_USERNAME "robopico"` and `MQTT_PASSWORD` in
   `config/wifi_secrets.h`, then rebuild and flash. Dashboard: set
   `MQTT_USERNAME=dashboard` and `MQTT_PASSWORD` in the environment before
   starting `dashboard.py`.

Passwords travel in clear text on port 1883. This keeps other devices on the
network from sending commands, but it does not protect against someone
capturing Wi-Fi traffic. If the robot cannot log in, its console prints
`mqtt: connection status 4` (bad user name or password) or `5` (not
authorised) and retries every 3 s.

## Receiving and display

The receiver subscribes at QoS 0 to exactly three topics:
`Robopico/sensors/edr16/v1`, `Robopico/events/edr16/v1` and
`csc02/command/ack`. Telemetry and event payloads are each two binary bytes.
Command replies are six bytes. Retained messages and unrelated topics are
ignored.

For telemetry, `decode_packet()` in `tools/edr_decode.py` checks the ID and
range and converts units. `decode_messages()` in `packed16.py` maps the
result to local display keys. These keys are not extra MQTT topics. Malformed packets are
counted and do not replace valid readings. Decode robot events with
`decode_event()`, not the sensor decoder.

The receiver keeps the latest reading. The browser waits 250 ms between
polls and marks readings stale after 5 s without an update. Availability and quality
records clear unusable live values. Broker disconnects and reconnects clear
live observations. The last scan's total width, left/right half-widths and
chosen side remain until new scan results replace them, including across a
reconnect or robot silence. A later side of None does not erase a stored side.
If `dashboard.py` restarts, the browser clears its readings because the
receiver ID changes.

The receiver logs the robot offline after 3 s without telemetry or events,
then logs it back online when data returns. It keeps the last 200 event rows.
It also logs broker changes, new controller faults and rising impact/IMU-fault
flags. Each impact and IMU-fault flag stays visible for 2 s. Reception time is
the computer's time, not the robot's measurement time. The 2 s hold keeps an
impact or IMU fault visible when it occurs in just one 200 ms snapshot,
shorter than the browser's 250 ms poll. A false value received during the
hold waits until it ends. `/api/telemetry` also reports received and rejected
packet counts and the last reception time.

The Vehicle controller card shows **Dropped, last 60 s** from differences in
the robot's modulo-256 drop counter. The first record after reconnect or
robot silence establishes a baseline. It counts what the robot itself
discarded (replaced or abandoned telemetry snapshots, events or commands that
found a full queue, and replies pushed out of a full reply buffer), not MQTT
messages lost on the network.

## Page layout

The Robot test card sits between the tab bar and the tab content, so it shows on every tab:

- **Overview:** power, range, servo, motor state, receiver health and subsystem coverage.
- **Barcodes:** mapping form, last barcode, A-Z codebook and read history.
- **Live views:** proximity radar, line sensors and acceleration.
- **Trends:** 60 s of browser-session encoder, line-sensor and acceleration history.
- **Sensors:** display fields filtered by subsystem.
- **Events:** robot and receiver events, newest first.

The URL hash keeps the selected tab, for example
<http://127.0.0.1:8001/#barcodes>. **Load demo** and **Import JSON** pause live
updates until **Resume live**. **Clear display** clears the browser display
and history, then reads the receiver snapshot again. Demo and imported
events show only while live updates are paused. Clicking a subsystem row
on Overview opens its sensors.

## Robot commands and barcode actions

With the robot ready to run, press **Reset robot**, then **Start test**.
START requests an autonomous mission at 20% demand. **STOP** ends motion.
The robot replies on `csc02/command/ack`; the page reports acceptance,
refusal or a missing reply. The robot boots DISARMED and needs RESET before
START. After an MQTT (re)connect or an MQTT STOP, the
network task refuses ARM and START until a RESET has been passed to the
controller's command queue; the controller can still refuse that RESET.
The robot's local faults and watchdog still apply.

The Barcode actions form sends MAP (opcode 4). Actions are Ignore, Left,
Right, Forward and U-turn. Reset to defaults restores A U-turn, F Left and
Z Right; other letters have no turn. A decoded action runs at the next
junction. Clicking a letter loads its current action into the form; **Send**
applies it. The table changes only after an accepted reply.

Mappings live in robot RAM. After a robot reboot, **Send all** sends the
receiver's table as 26 acknowledged A-Z frames. It does not reset the codebook
first. `POST /api/command` accepts actions `reset`, `start`, `stop`, `map`,
`map_reset` and `send_all`. A mapping request is, for example,
`{"action":"map","letter":"A","turn":"left"}`. See
[the command format](MQTT_TOPICS.md#command-topic).

Hump crest and level events carry the controller's height estimate (from
wheel distance and tilt) when it has one, and the event row says so.
Otherwise they carry S4's height, using different event kinds. The terrain
cards always show S4's terrain result, worked out by the VC with S4's
terrain code. Widths shown to the user include the
sonar beam correction.
The layouts and units are in
[DATA_REPRESENTATION.md](../../../docs/DATA_REPRESENTATION.md).

## Tests and evidence

Run from the repository root:

```powershell
python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py" -v
```

- `test_commands.py` checks frames, sequence/opcode matching, refusal, MAP,
  Send all and the local command API's request checks.
- `test_events.py` checks event decoding and text, invalid events, offline and
  broker events, fault rows and the event limit. Its firmware-packing check
  needs `gcc` on `PATH`.
- `test_packed16.py` checks topics, units, status, invalidation, held flags,
  drop counts, barcode history and HTTP output. With Node.js on `PATH`, it
  also runs `test_dashboard_display.cjs` against the page code.
- `test_obstacle_width.py` checks width pages and scan-result retention. Its
  C-producer check needs `testing/integration/.build/width_avoidance` (`.exe`
  on Windows), built by `python -B testing/integration/run.py --compiler gcc`
  or `make -C testing/integration test`. Without it, that case is skipped.

`testing/data_representation/run.py` discovers all Python tests here.
`testing/integration/run.py` also checks the production C snapshot through
EDR16, Python and, when Node.js is available, browser routing. Check skipped
or omitted cases before reporting a complete run.

The S1 log covers the four Python test files, data representation and
integration runners, and manual broker and robot checks. Interface and
whole-robot dashboard evidence are in the integration and system logs.
Live broker delivery, physical accuracy and course performance require
hardware evidence. [S1 log](../../../testing/subsystem1/TEST_LOG.md) ·
[Integration log](../../../testing/integration/TEST_LOG.md) ·
[System log](../../../testing/system/TEST_LOG.md)
