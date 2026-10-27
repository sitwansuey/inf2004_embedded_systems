"""Local, read-only dashboard for the single csc02/telemetry binary topic."""
import argparse
import json
import os
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from decoder import decode

class BrokerBridge:
    def __init__(self):
        self.lock = threading.Lock()
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
    def __init__(self, *args, bridge, **kwargs):
        self.bridge = bridge
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path != "/api/telemetry":
            return super().do_GET()
        data = json.dumps(self.bridge.snapshot()).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

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
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            client.disconnect()
            client.loop_stop()

if __name__ == "__main__":
    main()
