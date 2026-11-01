"""Unmarshal binary EDR16 from one MQTT topic and serve dashboard readings."""
import argparse
import json
import os
import threading
import time
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from packed16 import decode_messages, PACKED_TOPIC

TOPICS = [(PACKED_TOPIC, 0)]

def availability_mask(topic):
    """Only live observations are invalidated; decoded command history persists."""
    field=topic.rsplit("/",1)[-1]
    if field in ("encoder_left","encoder_right"): return 2
    if field in ("ir_left","ir_right","ir_barcode","line_state","black_mask","line_error"): return 4
    if field.startswith("accel_"): return 8
    if field.startswith("mag_"): return 128
    if field in ("pitch_deg","roll_deg"): return 32
    if field=="hump_height_cm": return 64
    if field=="distance_cm": return 16
    if "/motors/" in topic and field in ("direction","state","power_percent","fault_flags"): return 1
    return 0

class BrokerBridge:
    def __init__(self):
        self.lock = threading.Lock()
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

    def reset_observations(self):
        # Send null updates so an already-open browser clears its cards too.
        with self.lock:
            for reading in self.readings.values():
                self.sequence += 1
                reading.update(payload=None, sequence=self.sequence)
            self.last_received_at = None

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.reset_observations()
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
        self.reset_observations()
        self.set_status("MQTT disconnected; reconnecting")

    def on_connect_fail(self, client, userdata):
        self.set_status("MQTT broker unavailable; retrying")

    def on_message(self, client, userdata, message):
        # One wire topic. Names returned by decode_messages are local display
        # keys, never additional subscriptions or broker republishes.
        if message.topic != PACKED_TOPIC or getattr(message, "retain", False):
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
            if len(message.payload)==2 and (int.from_bytes(message.payload,"little") & 0xF00F)==0x9000:
                quality=int.from_bytes(message.payload,"little")>>4 & 255
                for topic, reading in self.readings.items():
                    required=1 if topic.endswith("/width_cm") else 16 if topic.endswith("/speed_mps") else 32 if topic.endswith("/servo_angle_deg") else 0
                    if required and not quality&required:
                        self.sequence+=1;reading.update(payload=None,sequence=self.sequence)
            # A status record invalidates affected readings even before expiry.
            if len(message.payload)==2 and (int.from_bytes(message.payload,"little") & 0xF00F)==0xA000:
                available=int.from_bytes(message.payload,"little")>>4 & 255
                for topic, reading in self.readings.items():
                    required=availability_mask(topic)
                    if required and not available&required:
                        self.sequence+=1;reading.update(payload=None,sequence=self.sequence)
            for topic, payload in messages:
                self.sequence += 1
                self.readings[topic] = {
                    "topic": topic, "payload": payload, "sequence": self.sequence,
                    "received_monotonic": time.monotonic()
                }

    def snapshot(self):
        with self.lock:
            now=time.monotonic()
            for reading in self.readings.values():
                if reading["payload"] is not None and now-reading["received_monotonic"]>1.0:
                    self.sequence+=1
                    reading.update(payload=None,sequence=self.sequence)
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
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            client.disconnect()
            client.loop_stop()

if __name__ == "__main__":
    main()
