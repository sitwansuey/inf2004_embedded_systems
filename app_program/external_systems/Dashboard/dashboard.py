<<<<<<< HEAD
"""Unmarshal binary EDR16 from one MQTT topic and serve dashboard readings."""
=======
"""Local, read-only dashboard for the single csc02/telemetry binary topic."""
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
import argparse
import json
import os
import threading
import time
<<<<<<< HEAD
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from packed16 import decode_messages, PACKED_TOPIC

TOPICS = [(PACKED_TOPIC, 0)]
=======
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from decoder import decode
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e

class BrokerBridge:
    def __init__(self):
        self.lock = threading.Lock()
<<<<<<< HEAD
        self.status = "Connecting to MQTT broker"
        self.readings = {}
        self.sequence = 0
        self.receiver_id = uuid.uuid4().hex
        self.received_packets = 0
        self.rejected_packets = 0
        self.last_received_at = None

    def set_status(self, status):
        with self.lock:
            self.status = status
        print(status, flush=True)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.set_status(f"MQTT rejected: {reason_code}")
            return
        self.set_status("MQTT connected; subscribing")
        result, _ = client.subscribe(TOPICS)
        if result != 0:
            self.set_status(f"MQTT subscription request failed: {result}")

    def on_subscribe(self, client, userdata, mid, reason_codes, properties):
        self.set_status("MQTT subscription rejected" if any(c.is_failure for c in reason_codes) else "MQTT subscribed")

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.set_status("MQTT disconnected; reconnecting")

    def on_connect_fail(self, client, userdata):
        self.set_status("MQTT broker unavailable; retrying")

    def on_message(self, client, userdata, message):
        # One wire topic. Names returned by decode_messages are local display
        # keys, never additional subscriptions or broker republishes.
        if message.topic != PACKED_TOPIC:
            return
        try:
            messages = decode_messages(message.payload)
        except (TypeError, ValueError):
            with self.lock:
                self.rejected_packets += 1
            return
        with self.lock:
            self.received_packets += 1
            self.last_received_at = time.time()
            for topic, payload in messages:
                self.sequence += 1
                self.readings[topic] = {
                    "topic": topic, "payload": payload, "sequence": self.sequence
                }

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "mqtt_topic": PACKED_TOPIC,
                "receiver_id": self.receiver_id,
                "received_packets": self.received_packets,
                "rejected_packets": self.rejected_packets,
                "last_received_at": self.last_received_at,
                "messages": sorted(self.readings.values(), key=lambda item: item["sequence"]),
            }

class DashboardHandler(SimpleHTTPRequestHandler):
=======
        self.latest = None
        self.received = None
        self.status = "Waiting for MQTT"
        self.rejected = 0

    def on_connect(self, client, userdata, flags, reason, properties):
        self.status = str(reason)
        if not reason.is_failure:
            client.subscribe("csc02/telemetry", qos=0)

    def on_message(self, client, userdata, message):
        if message.topic != "csc02/telemetry":
            return
        try:
            values = decode(message.payload)
        except ValueError:
            with self.lock:
                self.rejected += 1
            return
        with self.lock:
            self.latest = values
            self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            age = None if self.received is None else round((time.monotonic() - self.received) * 1000)
            return {"status": self.status, "age_ms": age, "stale": age is None or age > 1000,
                    "rejected": self.rejected, "telemetry": self.latest}

class Handler(SimpleHTTPRequestHandler):
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
    def __init__(self, *args, bridge, **kwargs):
        self.bridge = bridge
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path != "/api/telemetry":
            return super().do_GET()
<<<<<<< HEAD
        data = json.dumps(self.bridge.snapshot()).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
=======
        data = json.dumps(self.bridge.snapshot()).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

<<<<<<< HEAD
    def log_message(self, format, *args):
        if self.path != "/api/telemetry":
            super().log_message(format, *args)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    args = parser.parse_args()
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.exit(1, "Install MQTT support with: python -m pip install -r app_program/external_systems/Dashboard/requirements.txt\n")
    bridge = BrokerBridge()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if args.username:
        client.username_pw_set(args.username, os.getenv("MQTT_PASSWORD"))
    client.on_connect = bridge.on_connect
    client.on_subscribe = bridge.on_subscribe
    client.on_disconnect = bridge.on_disconnect
    client.on_connect_fail = bridge.on_connect_fail
    client.on_message = bridge.on_message
    client.reconnect_delay_set(min_delay=1, max_delay=10)
    handler = partial(DashboardHandler, bridge=bridge, directory=str(Path(__file__).parent))
    with ThreadingHTTPServer(("127.0.0.1", args.port), handler) as server:
        client.connect_async(args.broker, args.mqtt_port, keepalive=60)
        client.loop_start()
        print(f"Dashboard: http://127.0.0.1:{args.port}", flush=True)
        print(f"Broker: {args.broker}:{args.mqtt_port}; topic: {PACKED_TOPIC}", flush=True)
=======
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--port", type=int, default=8001)
    args = parser.parse_args()
    import paho.mqtt.client as mqtt
    bridge = BrokerBridge()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if os.getenv("MQTT_USERNAME"):
        client.username_pw_set(os.environ["MQTT_USERNAME"], os.getenv("MQTT_PASSWORD"))
    client.on_connect, client.on_message = bridge.on_connect, bridge.on_message
    client.reconnect_delay_set(min_delay=1, max_delay=10)
    handler = partial(Handler, bridge=bridge, directory=str(Path(__file__).parent))
    with ThreadingHTTPServer(("127.0.0.1", args.port), handler) as server:
        client.connect_async(args.broker, args.mqtt_port, 60)
        client.loop_start()
        print(f"Read-only dashboard: http://127.0.0.1:{args.port}")
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            client.disconnect()
            client.loop_stop()

if __name__ == "__main__":
    main()
