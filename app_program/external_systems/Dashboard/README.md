# Single-topic EDR16 dashboard

The dashboard subscribes (QoS 0) to three topics: telemetry on
`Robopico/sensors/edr16/v1`, robot events on `Robopico/events/edr16/v1` and
command replies on `csc02/command/ack`. Every telemetry and event payload is
exactly two binary bytes. There are no wildcard
subscriptions and no per-sensor MQTT subscriptions. Messages on other topics
are ignored.

## Marshalling and unmarshalling

1. Firmware `edr_encode(id, data, bytes, 2)` marshals bits 15–4 = data and bits
   3–0 = ID into two little-endian bytes. B1 copies and publishes those bytes.
2. Python `tools/edr_decode.py::decode_packet` unmarshals the bytes, validates
   the ID/range and converts signed offsets into physical units.
3. `packed16.py::decode_messages` maps each reading to a local display field.
   ID 11 expands to motor direction, power and state. No broker republishing
   takes place; the apparent sensor topic names are internal display keys.
4. `dashboard.py` stores the latest values and serves `/api/telemetry`.
   JavaScript polls this endpoint and updates the matching sensor cards.

The receiver rejects malformed/old/text/JSON payloads on the wire topic without
replacing valid readings. Its API includes received/rejected packet counts,
receiver identity and last reception time. Browser state resets if the receiver
restarts, allowing sequence numbers to start again. Reception time is a desktop
time, not a robot measurement timestamp. Every reading keeps its latest value
until a new one replaces it (the page marks it stale after 5 seconds without an
update). Status availability/quality records and an MQTT reconnect clear
invalid data.

## Run

From the repository root:

```powershell
python -m pip install -r app_program/external_systems/Dashboard/requirements.txt
python app_program/external_systems/Dashboard/dashboard.py --broker localhost --mqtt-port 1883
```

Open http://127.0.0.1:8001. Start your broker separately; use its reachable LAN
address if it is on another computer. The Pico's local wifi_secrets.h must
point to the same broker. Optional MQTT_USERNAME/MQTT_PASSWORD environment
variables (or `--username`, with the password still from MQTT_PASSWORD)
provide dashboard credentials. Keep the repository layout because the
receiver imports the canonical codec from tools/.

## Broker access

MQTT does not tell the robot who published a command, and the firmware does
not check. On an open broker any device on the same network can publish
RESET then START on `csc02/command`. To stop that, make the broker require a
login and let only the dashboard account publish commands. Nothing changes
until you do this: with no login defined, the robot and the dashboard
connect anonymously as before.

1. Create a password file with one account for the dashboard and one for the
   robot (mosquitto 2.x; `-c` creates the file, so use it only the first time):

   ```powershell
   mosquitto_passwd -c C:\mosquitto\passwd dashboard
   mosquitto_passwd C:\mosquitto\passwd robopico
   ```

2. Create an ACL file, `C:\mosquitto\acl`:

   ```text
   # Only the dashboard may command the robot.
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

3. Point the broker at both files (`mosquitto.conf`) and restart it:

   ```text
   listener 1883
   allow_anonymous false
   password_file C:\mosquitto\passwd
   acl_file C:\mosquitto\acl
   ```

4. Robot: in `config/wifi_secrets.h` set `MQTT_USERNAME "robopico"` and
   `MQTT_PASSWORD` to that account's password, then rebuild and flash.
   Dashboard: set `MQTT_USERNAME=dashboard` and `MQTT_PASSWORD` in the
   environment before starting `dashboard.py`.

A client with no login, or with the robot's login, is then refused on
`csc02/command`. The passwords travel in clear text on plain port 1883, so
this keeps out other devices on the robot hotspot, not someone sniffing the
Wi-Fi. If the robot cannot log in, its console prints
`mqtt: connection status 4` or `5` (bad login, not authorised) every 3 s.

An explicit test publication (requires a running broker; simulated value):

```python
from paho.mqtt import publish
from tools.edr_decode import MQTT_TOPIC, encode_packet
publish.single(MQTT_TOPIC, payload=encode_packet(1, 100), hostname="localhost", port=1883)
# Payload bytes 41 06 -> Left wheel encoder card: 100.
```

Publish the raw bytes, not the four-character string "4106". Firmware reads
servo/ultrasonic and other owner snapshots. ID 0 adds decoded barcode/navigation,
controller state and fault (0–8, 8 = lifted), speed, tilt, scan state and
hump/current-run maximum to their cards. ID 12 also carries the IMU fault bit
(Accelerometer card, IMU fault). Status subtype 12 is the robot's drop count
modulo 256; the Vehicle controller card shows "Dropped, last 60 s" from the
differences (the first record after a broker reconnect or a robot silence is
only a baseline). The Sensors tab lists only fields the
robot transmits. Event text is not transmitted: robot events arrive as two-byte
records on `Robopico/events/edr16/v1` and `packed16.describe_event` turns them
into log rows.
See `docs/DATA_REPRESENTATION.md` for the ID 0 subtype table and units.

## Layout

The page copies the layout and look of main's React dashboard, rebuilt in
plain HTML/CSS/JavaScript served by `dashboard.py`: no build step, no npm and
no fonts or CDN files, so it works offline on the robot hotspot. The Robot
test card (Reset robot, Start test, STOP, the robot's safety state and tiles
for mission state, motion, terrain validity, fault code, barcode letter and
instruction) stays above every tab. The tabs are:

- Overview: power, range and servo dials, motor card, receiver health and
  subsystem coverage (click a row to open its sensors).
- Barcodes: Barcode actions form, last barcode, Codebook A–Z and barcode reads.
- Live views: proximity radar, line sensor bars and acceleration.
- Trends: 60 s of this browser session for the wheel encoders, line sensors
  (left/right IR and line error) and acceleration.
- Sensors: every display field, filterable by subsystem.
- Events: the receiver's event log, newest first (last 200 rows): robot
  events from `Robopico/events/edr16/v1` (barcode attempts, decoded
  instructions, humps, obstacles, Wi-Fi/broker reconnects, decoded by
  `packed16.describe_event`), plus the receiver's own rows: robot offline (no
  data for 3 s) / back online, each new controller fault code, each new
  impact or IMU fault, and its broker link. Demo and imported events show
  only while live updates are paused.

Impact (quality bit 1) and IMU fault (ID 12 bit 0x04) are true in a single
200 ms snapshot, shorter than the page's 250 ms poll. `dashboard.py` shows
each one as Yes for 2 s after it is reported (a false value arriving
meanwhile waits) and logs one event row per rising edge.

The selected tab is kept in the URL hash, for example
http://127.0.0.1:8001/#barcodes. Load demo and Import JSON pause live updates
until Resume live. Clear display drops this browser's readings and history and
re-reads the receiver snapshot. Readings are marked stale after 5 seconds
without an update.

## Robot commands and barcode actions

The Robot test buttons send RESET/START/STOP frames on `csc02/command` and
show the robot's ACK from `csc02/command/ack`. The Barcode actions panel sends
opcode 4 (MAP) to choose what a decoded letter does at the next junction:
Forward, Left, Right, U-turn or Ignore (read, no turn). Reset to defaults
restores A U-turn, F Left, Z Right. The Codebook A–Z grid on the Barcodes tab
shows the mapping last accepted by the robot; letters absent from it are read
without a turn, and a dot marks the last letter read. Clicking a letter only
loads it and its current action into the form; Send applies it. Mappings live
in robot RAM; after a robot reboot press Send all.
`POST /api/command` accepts `{"action":"map","letter":"A","turn":"left"}`,
`{"action":"map_reset"}` and `{"action":"send_all"}`. Frame layout:
[MQTT_TOPICS.md](MQTT_TOPICS.md#command-topic-dashboard-buttons-only).

## Tests

```powershell
python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py" -v
```

- `test_events.py`: every robot event (barcode attempts, instructions, hump,
  obstacle, link) to its log text, invalid records, the 200-row log, robot
  offline/back online, new fault codes, the receiver's own broker link, and
  (with gcc) the real `shared/events.h` packing decoded by
  `tools/edr_decode.py`.
- `test_packed16.py`: exact-topic subscription/reconnection, all 15 IDs,
  physical units, status subtypes (including fault 8, lifted), malformed
  packets, ignored unrelated topics, barcode read history, the 2 s hold and
  event row for impact and IMU fault, HTTP JSON and field
  routing. With Node.js on PATH it also runs `test_dashboard_display.cjs`,
  which executes the real dashboard JavaScript with a minimal DOM and checks
  that all 18 decoded fields appear on the correct cards.
- `test_commands.py`: Reset/Start/STOP frames, ACK matching and validation,
  sequences 0..127 (a USB command's ACK, bit 7, is ignored),
  MAP, Reset to defaults and Send all, and the origin and JSON checks on
  `POST /api/command`.
- `test_obstacle_width.py`: width pages, retention and clearing. Its C producer
  case is skipped until `make -C testing/integration test` has built
  `.build/width_avoidance`.

CI (`testing/data_representation/run.py`, stage 1) discovers and runs every
`test_*.py` in this folder; `test_obstacle_width.py`'s C producer case is
skipped unless the integration `width_avoidance` build exists.

These are simulated tests, not a live Pico/broker validation. See
[MQTT_TOPICS.md](MQTT_TOPICS.md) for routing.

Checkpoint 8c reports a flat-top hump as one event cycle. Its level row uses
S4's completed-height latch, or shows not measured if none completed. A
plain descent gets the controller's slope cap without a measured hump.
Impact and IMU-fault counters cover brief samples between controller reads;
the telemetry queue preserves those flags when it discards older snapshots.
Impact extras are limited to one per 200 ms. The wire layout stays EDR16 v1.
A side-check or forward-pan timeout now shows sensor fault cause Sonar.

The robot retries a stalled first Wi-Fi join after 20 s without UP, then
waits 1 s between leave and join. After an earlier UP, the no-link bound
is 10 s. Rejected join requests retry every 5 s. The reconnect event is
sent before the outage backlog. These paths still need a hardware AP-off
test. The browser display check ran with VS Code's bundled Node runtime
(18 fields). No repository tooling code changed for W10.
