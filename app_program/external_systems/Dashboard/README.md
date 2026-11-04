# PicoCar NexusUI dashboard

The dashboard uses React and [NexusUI 2.1.6](https://github.com/nexus-js/ui).
NexusUI widgets display key readings and are read-only: dials for commanded
power, obstacle distance and servo angle; a multislider for the three IR
sensors; a position pad and slider rail for acceleration; and a sequencer row
for the motor fault bits. NexusUI has no time-series widget, so the history
lanes are drawn on canvas in the same style.

The views are split into tabs: Overview, Live views, Trends, Sensors and
Events. The selected tab is kept in the URL hash, for example
`http://127.0.0.1:8001/#sensors`. Every box uses the same navy card style.
Cyan, mint and purple appear only inside graphs, while small coloured dots show
reading status: received, snapshot, stale or fault. The original sensor cards,
subsystem filter, event history, JSON import, demo and live receiver remain
available. The Overview's subsystem list opens the Sensors tab filtered to that
subsystem. The Python receiver owns the MQTT
connection and serves both the browser assets and `/api/telemetry`. The browser
does not connect to the broker, so an MQTT WebSocket listener is not required.

The receiver subscribes only to `Robopico/sensors/edr16/v1` at QoS 0. Each MQTT
payload is exactly two binary bytes. There are no wildcard or per-sensor MQTT
subscriptions. Messages on other topics are ignored.

## Build and run

Install Node.js 20 or later and Python. From the repository root, install the
locked frontend dependencies and build the browser assets:

```powershell
npm --prefix app_program/external_systems/Dashboard ci
npm --prefix app_program/external_systems/Dashboard run build
python -m pip install -r app_program/external_systems/Dashboard/requirements.txt
```

Use `npm install` in place of `npm ci` only when creating or intentionally
updating the dependency lockfile. Then start the dashboard with the existing
Python command:

```powershell
python app_program/external_systems/Dashboard/dashboard.py --broker localhost --mqtt-port 1883
```

Open http://127.0.0.1:8001. Run the broker separately. If it is on another
computer, replace `localhost` with its reachable LAN address. The Pico's local
`wifi_secrets.h` must point to the same broker. Optional `MQTT_USERNAME` and
`MQTT_PASSWORD` environment variables provide dashboard credentials. Keep the
repository layout because the receiver imports the canonical codec from
`tools/`.

Rebuild after changing frontend source or pulling frontend changes. The build
writes `assets/dashboard.js` and `assets/dashboard.css`; Python serves them
through `index.html`. Node.js is needed for building and frontend checks. A
runtime computer can use those built assets with the Python service without
Node.js or `node_modules`. Copy the built `assets/` directory along with the
updated dashboard files and required repository codec; do not copy only the
HTML shell. Open the HTTP URL rather than `index.html` directly so the browser
can reach `/api/telemetry`.

For frontend development, run this in another terminal while Python is running:

```powershell
npm --prefix app_program/external_systems/Dashboard run dev
```

This watches and rebuilds the assets. It does not start a second web server;
refresh the Python-served page after changes.

## What the readings mean

Live readings come from the EDR16 schema: encoder counts, servo angle,
ultrasonic range, raw IR, magnetic field, acceleration, and motor direction,
commanded power, state and fault bits. Commanded power is PWM output, not
measured speed. Encoder counts wrap modulo 4096. See
[MQTT_TOPICS.md](MQTT_TOPICS.md) for exact ID routing and units.

The receiver exposes its identity, received/rejected packet counts and desktop
reception times. Charts use these reception times and retain browser-session
history; they are not robot sampling times or a recording of every MQTT packet.
The HTTP API contains the latest value per field, so readings between browser
polls can be coalesced. EDR16 includes no sensor timestamp or freshness metadata.
A recent reception therefore cannot establish when the sensor measured a value.
When the receiver restarts, the browser resets sequence tracking and live
history for the new receiver.

Missing readings remain unavailable. The dashboard does not derive speed,
heading, pitch, roll, terrain height, barcode text or controller events from the
current wire schema. Supported sensors that are not connected remain blank.
The proximity view plots the supplied ultrasonic range at the supplied servo
angle; a single reading is not a complete scan or map. The vehicle view shows
an orientation only when a heading is supplied by a demo or imported snapshot.
Live EDR16 data does not supply an absolute heading. Encoder history shows
counts, not calculated motor RPM or calibrated speed.

Demo and JSON import are explicit browser-only modes, visibly separate from live
MQTT telemetry. Their synthetic or imported values do not prove a robot
connection and are not published to the broker. Return to live mode to resume
viewing receiver data. This dashboard displays telemetry; it does not send motor
or navigation commands.

## Marshalling and unmarshalling

1. Firmware `edr_encode(id, data, bytes, 2)` packs data into bits 15-4 and the ID
   into bits 3-0, producing two little-endian bytes. B1 publishes those bytes.
2. `tools/edr_decode.py::decode_packet` validates the length, ID and range,
   then converts signed offsets into physical units.
3. `packed16.py::decode_messages` maps readings to local display fields. ID 11
   expands into motor direction, power and state. These keys are not additional
   MQTT topics consumed by the receiver; no broker republishing takes place.
4. `dashboard.py` stores the latest readings and serves `/api/telemetry`.
   The React frontend polls this endpoint and supplies NexusUI with numeric data.

Malformed, old-format, text and JSON wire payloads are rejected without replacing
valid readings.

An explicit test publication requires a running broker and supplies a simulated
value:

```python
from paho.mqtt import publish
from tools.edr_decode import MQTT_TOPIC, encode_packet

publish.single(MQTT_TOPIC, payload=encode_packet(1, 100), hostname="localhost", port=1883)
# Payload bytes 41 06 -> Left wheel encoder: 100.
```

Publish the raw bytes, not the four-character string `"4106"`.

## Checks

From the repository root:

```powershell
python -B -m unittest discover -s app_program/external_systems/Dashboard -p test_packed16.py -v
npm --prefix app_program/external_systems/Dashboard test
npm --prefix app_program/external_systems/Dashboard run build
```

For browser checks, install Chromium once and run the frontend's end-to-end
suite from the dashboard folder:

```powershell
cd app_program/external_systems/Dashboard
npx playwright install chromium
npm run test:e2e
```

These checks use simulated inputs. A live Pico, broker, sensor calibration and
network delivery still need hardware validation. Keep the pinned NexusUI
version and dependency lockfile, and review API changes before upgrading.
