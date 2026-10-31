# Single-topic EDR16 dashboard

The dashboard subscribes only to `Robopico/sensors/edr16/v1` (QoS 0). Every MQTT
payload is exactly two binary bytes. There are no wildcard subscriptions and
no per-sensor MQTT subscriptions. Messages on other topics are ignored.

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
time, not a robot measurement timestamp; EDR16 has no freshness metadata.

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

Publish the raw bytes, not the four-character string "4106". Servo/ultrasonic
IDs are supported by the receiver, but their owners still need to provide
actual measurements. Derived speed, heading, barcode text, controller state
and event text are not defined by the current 15-ID schema and remain blank
unless supplied by an explicit demo/import. No values are fabricated.

## Tests

```powershell
python -B -m unittest discover -s app_program/external_systems/Dashboard -p test_packed16.py -v
```

Tests cover exact-topic subscription/reconnection, all 15 IDs, physical units,
malformed packets, ignored unrelated topics, HTTP JSON, and field routing.
With Node.js on PATH, they also execute the real dashboard JavaScript using a
minimal DOM and verify all 17 display fields. These are simulated tests, not a
live Pico/broker validation. See [MQTT_TOPICS.md](MQTT_TOPICS.md) for routing.
