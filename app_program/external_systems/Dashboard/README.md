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
variables provide dashboard credentials. Keep the repository layout because
the receiver imports the canonical codec from tools/.

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
hump/current-run maximum to their cards. The Sensors tab lists only fields the
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
  data for 3 s) / back online, each new controller fault code, and its broker
  link. Demo and imported events show only while live updates are paused.

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
  packets, ignored unrelated topics, barcode read history, HTTP JSON and field
  routing. With Node.js on PATH it also runs `test_dashboard_display.cjs`,
  which executes the real dashboard JavaScript with a minimal DOM and checks
  that all 17 decoded fields appear on the correct cards.
- `test_commands.py`: Reset/Start/STOP frames, ACK matching and validation,
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
