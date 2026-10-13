"""Serve the dashboard and subscribe to MQTT on localhost:1883."""
import argparse
import json
import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TOPICS = [("Robopico/sensors/#", 0), ("Robopico/Actuators/#", 0)]

class BrokerBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = "Connecting to MQTT broker"
        self.readings = {}
        self.events = []
        self.sequence = 0

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
        if len(message.payload) > 16384:
            return
        try:
            payload = message.payload.decode("utf-8")
        except UnicodeDecodeError:
            return
        with self.lock:
            self.sequence += 1
            item = {"topic": message.topic, "payload": payload, "sequence": self.sequence}
            if message.topic == "Robopico/Actuators/vehicle_controller/events":
                self.events.append(item)
                self.events = self.events[-100:]
            elif message.topic in self.readings or len(self.readings) < 256:
                self.readings[message.topic] = item

    def snapshot(self):
        with self.lock:
            return {"status": self.status, "messages": sorted([*self.readings.values(), *self.events], key=lambda item: item["sequence"])}

class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, bridge, **kwargs):
        self.bridge = bridge
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path != "/api/telemetry":
            return super().do_GET()
        data = json.dumps(self.bridge.snapshot()).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

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
        parser.exit(1, "Install MQTT support with: python -m pip install -r app_program/external_systems/requirements.txt\n")
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
        print(f"Broker: {args.broker}:{args.mqtt_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            client.disconnect()
            client.loop_stop()

if __name__ == "__main__":
    main()
