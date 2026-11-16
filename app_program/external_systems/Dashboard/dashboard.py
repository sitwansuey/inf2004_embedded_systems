"""Unmarshal binary EDR16 from one MQTT topic and serve dashboard readings."""
import argparse
import json
import os
import string
import threading
import time
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from packed16 import decode_messages, PACKED_TOPIC

ACK_TOPIC = "csc02/command/ack"
START_DEMAND = 20  # Start power percent (the teammates' working follower runs at 20)
TOPICS = [(PACKED_TOPIC, 0), (ACK_TOPIC, 0)]
COMMAND_TOPIC = "csc02/command"
# Scan results retained between reports and cleared by quality bit 0.
WIDTH_TOPICS = ("/width_cm", "/width_left_cm", "/width_right_cm")
# Opcode 4 action byte (s3_navigation_t order) and the firmware codebook.
MAP_TURNS = {"ignore": 0, "left": 1, "right": 2, "forward": 3, "uturn": 4}
MAP_LABELS = ("Ignore", "Left", "Right", "Forward", "U-turn")
DEFAULT_BARCODE_MAP = {"A": "U-turn", "F": "Left", "Z": "Right"}
FAULTS = ("none", "sensor", "line", "shock", "transport", "run timeout", "uncalibrated", "motor", "lifted")

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
        self.last_received_monotonic = 0.0
        self.width_valid = False
        self.command_client = None
        self.command_lock = threading.Lock()
        self.command_sequence = 0
        self.pending_commands = {}
        self.ack_condition = threading.Condition(self.lock)
        # One row per finished barcode read, newest last (see track_barcode).
        self.barcode_history = []
        self.barcode_read = None
        self.motor_direction = None
        self.motor_power = 0
        self.barcode_letter = None
        self.barcode_turn = None
        # Letter -> action label as last acknowledged by the robot (RAM only there).
        self.barcode_map = dict(DEFAULT_BARCODE_MAP)

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
            self.last_received_monotonic = 0.0
            self.width_valid = False

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
        if message.topic == ACK_TOPIC:
            if getattr(message, "retain", False):
                return
            data = message.payload
            if len(data) != 6 or data[0] != 0xC2 or data[2] > 4 or data[3] > 3 or data[4] > 13 or data[5] >= len(FAULTS):
                return
            with self.ack_condition:
                entry = self.pending_commands.get(data[1])
                if entry is not None and entry["opcode"] == data[2]:
                    entry["reply"] = tuple(data[3:])
                    self.ack_condition.notify_all()
            return
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
            self.last_received_monotonic = time.monotonic()
            if len(message.payload)==2 and (int.from_bytes(message.payload,"little") & 0xF00F)==0x8000:
                self.width_valid=True  # Producer sends width only for a valid scan.
            if len(message.payload)==2 and (int.from_bytes(message.payload,"little") & 0xF00F)==0x9000:
                quality=int.from_bytes(message.payload,"little")>>4 & 63  # bits 6..7 = fault cause
                self.width_valid=bool(quality&1)
                for topic, reading in self.readings.items():
                    required=1 if topic.endswith(WIDTH_TOPICS) else 16 if topic.endswith("/speed_mps") else 32 if topic.endswith("/servo_angle_deg") else 0
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
                self.track_barcode(topic, payload)

    def track_barcode(self, topic, payload):
        """Turn the live barcode fields into one history row per read.

        A read runs from decode_result "Reading..." to Successful/Unsuccessful.
        While it runs, keep the highest element count and how the robot drove
        (any REVERSE -> reverse pass, power <= 12% -> slow pass). Reads under
        9 elements are line crossings, not barcodes. Caller holds self.lock.
        """
        field = "/".join(topic.rsplit("/", 2)[-2:])
        now = time.time()
        read = self.barcode_read
        if field == "motors/direction":
            self.motor_direction = payload
            if read is not None: read["directions"].add(payload)
        elif field == "motors/power_percent":
            try: self.motor_power = int(payload)
            except (TypeError, ValueError): return
            if read is not None and self.motor_power: read["max_power"] = max(read["max_power"], self.motor_power)
        elif field == "barcode_ir/elements_read" and read is not None:
            read["elements"] = max(read["elements"], int(payload))
        elif field in ("barcode_ir/barcode", "barcode_ir/navigation"):
            if field.endswith("barcode"): self.barcode_letter = payload
            else: self.barcode_turn = payload
            # The letter and turn can arrive just after the result.
            last = self.barcode_history[-1] if self.barcode_history else None
            if last and last["result"] == "Successful" and now - last["epoch"] <= 2.0:
                last["letter" if field.endswith("barcode") else "turn"] = payload
        elif field == "barcode_ir/decode_result":
            if payload == "Reading...":
                if read is None:
                    self.barcode_read = {"start": now, "elements": 0,
                                         "directions": {self.motor_direction} - {None},
                                         "max_power": self.motor_power}
            elif read is not None:
                self.barcode_read = None
                if payload not in ("Successful", "Unsuccessful") or read["elements"] < 9:
                    return
                directions = read["directions"]
                kind = ("Reverse" if "REVERSE" in directions else
                        "Slow" if 0 < read["max_power"] <= 12 else "Normal")
                ok = payload == "Successful"
                self.barcode_history.append({
                    "epoch": now, "time": time.strftime("%H:%M:%S", time.localtime(now)),
                    "pass": kind, "elements": min(read["elements"], 31),
                    "seconds": round(now - read["start"], 1), "result": payload,
                    "letter": self.barcode_letter if ok else None,
                    "turn": self.barcode_turn if ok else None})
                del self.barcode_history[:-30]

    def send_command(self, action, letter=None, turn=None):
        opcodes = {"stop": 0, "reset": 2, "start": 3, "map": 4, "map_reset": 4, "send_all": 4}
        if action not in opcodes:
            raise ValueError("Expected reset, start, stop, map, map_reset or send_all.")
        if action == "map":
            if not isinstance(letter, str) or len(letter) != 1 or letter not in string.ascii_letters:
                raise ValueError("Letter must be one character A-Z.")
            if not isinstance(turn, str) or turn not in MAP_TURNS:
                raise ValueError("Turn must be forward, left, right, uturn or ignore.")
            letter = letter.upper()
        elif letter is not None or turn is not None:
            raise ValueError("Only map takes a letter and turn.")
        locked = action != "stop"
        if locked and not self.command_lock.acquire(blocking=False):
            raise RuntimeError("A command is already being sent.")
        try:
            if action == "send_all":
                return self.send_all_maps()
            if action == "map":
                label = MAP_LABELS[MAP_TURNS[turn]]
                result, phase, fault = self.exchange(4, ord(letter) - 64, MAP_TURNS[turn])
            else:
                result, phase, fault = self.exchange(opcodes[action], START_DEMAND if action == "start" else 0)
            if result == 3:
                raise RuntimeError("RoboPico rejected an expired command. Try again after checking its state.")
            if opcodes[action] == 4:
                if result != 0:
                    what = f"{letter} = {label}" if action == "map" else "the codebook reset"
                    raise RuntimeError(f"RoboPico rejected {what}. Check the letter/action and firmware.")
                with self.lock:
                    if action == "map":
                        self.barcode_map[letter] = label
                    else:
                        self.barcode_map = dict(DEFAULT_BARCODE_MAP)
                    table = dict(self.barcode_map)
                message = (f"RoboPico set barcode {letter} to {label}." if action == "map" else
                           "RoboPico restored the built-in codebook: A U-turn, F Left, Z Right.")
                return {"action": action, "acknowledged": True, "phase": phase, "fault": fault,
                        "barcode_map": table, "message": f"{message} Phase {phase}, fault {FAULTS[fault]}."}
            if result == 2:
                raise RuntimeError("RoboPico rejected Start: press Reset robot first after MQTT reconnect.")
            if result != 0:
                raise RuntimeError(f"RoboPico rejected {action}. Phase {phase}, fault {FAULTS[fault]}. Reset requires stopped motors; Start requires reset and readiness.")
            return {"action": action, "acknowledged": True, "phase": phase, "fault": fault,
                    "message": f"RoboPico accepted {action}. Phase {phase}, fault {FAULTS[fault]}."}
        finally:
            if locked:
                self.command_lock.release()

    def send_all_maps(self):
        """Write every letter's final action (A..Z) directly, one ACKed frame at a
        time. Each write is idempotent and there is no reset first, so a lost
        ACK or a stop part-way never leaves the robot on the default codebook
        while the table shows overrides; just press Send all again."""
        with self.lock:
            table = dict(self.barcode_map)
        frames = [(ord(k) - 64, MAP_LABELS.index(table.get(k, DEFAULT_BARCODE_MAP.get(k, "Ignore"))), k)
                  for k in string.ascii_uppercase]
        for sent, (letter, action, name) in enumerate(frames):
            what = f"{name} = {MAP_LABELS[action]}"
            try:
                result, phase, fault = self.exchange(4, letter, action)
            except RuntimeError as error:
                raise RuntimeError(f"Send all stopped at {what} ({sent} of {len(frames)} sent): {error}") from None
            if result != 0:
                raise RuntimeError(f"Send all stopped: RoboPico rejected {what} ({sent} of {len(frames)} sent).")
        changed = sum(1 for k, v in table.items() if DEFAULT_BARCODE_MAP.get(k) != v)
        return {"action": "send_all", "acknowledged": True, "phase": phase, "fault": fault,
                "sent": len(frames), "barcode_map": table,
                "message": f"RoboPico accepted all 26 letters ({changed} differ from the built-in codebook)."}

    def exchange(self, opcode, demand=0, value=0):
        """Publish one command frame and wait for its ACK: (result, phase, fault)."""
        client = self.command_client
        if client is None or not client.is_connected():
            raise RuntimeError("MQTT is disconnected.")
        with self.ack_condition:
            sequence = self.command_sequence
            self.command_sequence = (sequence + 1) & 255
            entry = {"opcode": opcode, "reply": None}
            self.pending_commands[sequence] = entry
        try:
            frame = bytes((0xC1, opcode, demand, value, 0, sequence))
            # QoS 0 avoids retrying a stale START after a reconnect.
            info = client.publish(COMMAND_TOPIC, frame, qos=0, retain=False)
            if info.rc != 0:
                raise RuntimeError("MQTT rejected the command.")
            info.wait_for_publish(timeout=2)
            if not info.is_published():
                raise RuntimeError("Command delivery is unconfirmed. Check telemetry before retrying.")
            with self.ack_condition:
                if not self.ack_condition.wait_for(lambda: entry["reply"] is not None, timeout=3):
                    raise RuntimeError("Sent to MQTT, but RoboPico did not acknowledge. Check its power, connection and firmware.")
                return entry["reply"]
        finally:
            with self.ack_condition:
                self.pending_commands.pop(sequence, None)

    def snapshot(self):
        with self.lock:
            # Every reading keeps its latest value until a new one arrives
            # (the page marks it stale after a while); only an explicit robot
            # validity flag or a broker reconnect clears it.
            return {
                "status": self.status,
                "mqtt_topic": PACKED_TOPIC,
                "receiver_id": self.receiver_id,
                "received_packets": self.received_packets,
                "rejected_packets": self.rejected_packets,
                "last_received_at": self.last_received_at,
                "messages": sorted(self.readings.values(), key=lambda item: item["sequence"]),
                "barcode_history": [dict(row) for row in self.barcode_history],
                "barcode_map": dict(sorted(self.barcode_map.items())),
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

    def send_json(self, status, result):
        data = json.dumps(result).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        if self.path != "/api/command":
            self.send_json(404, {"error": "Unknown command endpoint."})
            return
        # Require a loopback Host and a matching browser origin. A missing
        # Origin is accepted only with a matching same-origin Referer.
        host = self.headers.get("Host", "")
        allowed_hosts = {f"127.0.0.1:{self.server.server_port}",
                         f"localhost:{self.server.server_port}"}
        origin = self.headers.get("Origin")
        if origin is None:
            referer = urlsplit(self.headers.get("Referer", ""))
            origin = f"{referer.scheme}://{referer.netloc}"
        if host not in allowed_hosts or origin != f"http://{host}":
            self.send_json(403, {"error": "Command origin rejected. Open this dashboard at "
                            f"http://127.0.0.1:{self.server.server_port}/ and reload the page."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 128 or self.headers.get_content_type() != "application/json":
                raise ValueError("Expected a small JSON command.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or not isinstance(payload.get("action"), str):
                raise ValueError("Expected one action field.")
            if payload["action"] == "map":
                if set(payload) != {"action", "letter", "turn"}:
                    raise ValueError("Expected action, letter and turn fields.")
                result = self.bridge.send_command("map", payload["letter"], payload["turn"])
            elif set(payload) != {"action"}:
                raise ValueError("Expected one action field.")
            else:
                result = self.bridge.send_command(payload["action"])
            status = 200
        except (ValueError, json.JSONDecodeError) as error:
            result, status = {"error": str(error)}, 400
        except (RuntimeError, OSError) as error:
            result, status = {"error": str(error)}, 503
        self.send_json(status, result)

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
    bridge.command_client = client
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
