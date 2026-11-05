"""Unmarshal binary EDR16 from one MQTT topic and serve dashboard readings."""
import argparse
import json
import os
import threading
import time
import uuid
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from packed16 import decode_messages, PACKED_TOPIC
from barcode_diagnostic import decode as decode_barcode_diagnostic, TOPIC as BARCODE_DIAGNOSTIC_TOPIC

ACK_TOPIC = "csc02/command/ack"
MAP_TOPIC = "csc02/barcode/map"
MAP_ACK_TOPIC = "csc02/barcode/map/ack"
MAP_ACTIONS = ("forward", "uturn", "left", "right")
TOPICS = [(PACKED_TOPIC, 0), (ACK_TOPIC, 0), (BARCODE_DIAGNOSTIC_TOPIC, 1), (MAP_ACK_TOPIC, 1)]
COMMAND_TOPIC = "csc02/command"
START_POWER_PERCENT = 20

def validate_mapping(mapping):
    if not isinstance(mapping,dict) or set(mapping)!=set(MAP_ACTIONS):
        raise ValueError("Movement map needs forward, uturn, left and right letters.")
    letters=[mapping[k] for k in MAP_ACTIONS]
    if any(not isinstance(v,str) or len(v)!=1 or not "A"<=v<="Z" for v in letters) or len(set(letters))!=4:
        raise ValueError("Movement map needs four distinct uppercase letters A-Z.")
    return {k:mapping[k] for k in MAP_ACTIONS}

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
    def __init__(self, mapping_file=None):
        self.lock = threading.Lock()
        self.status = "Connecting to MQTT broker"
        self.readings = {}
        self.sequence = 0
        self.receiver_id = uuid.uuid4().hex
        self.received_packets = 0
        self.rejected_packets = 0
        self.last_received_at = None
        self.command_client = None
        self.command_lock = threading.Lock()
        self.command_sequence = 0
        self.pending_commands = {}
        self.ack_condition = threading.Condition(self.lock)
        self.barcode_diagnostics = None
        self.barcode_history = deque(maxlen=50)
        self.diagnostic_received = None
        self.mapping_file=Path(mapping_file) if mapping_file else None
        self.saved_mapping=None
        self.mapping_error=None
        self.pending_map=None
        if self.mapping_file and self.mapping_file.exists():
            self.saved_mapping=validate_mapping(json.loads(self.mapping_file.read_text(encoding="utf-8-sig")))

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
            self.diagnostic_received = None

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
        if message.topic==MAP_ACK_TOPIC:
            if getattr(message,"retain",False):return
            data=message.payload
            if len(data)!=6 or data[0]!=0xC4 or data[1]>4:return
            mapping=None
            if data[1]==0:
                try:mapping=validate_mapping(dict(zip(MAP_ACTIONS,map(chr,data[2:]))))
                except ValueError:return
            with self.ack_condition:
                if mapping is not None:self.saved_mapping=mapping
                if self.pending_map and (data[2:]==self.pending_map["letters"] or data[1]!=0 and data[2:]==bytes(4)):
                    self.pending_map["reply"]=data[1];self.ack_condition.notify_all()
            if mapping is not None and self.mapping_file:
                try:
                    temporary=self.mapping_file.with_suffix(self.mapping_file.suffix+".tmp")
                    temporary.write_text(json.dumps(mapping,indent=2)+"\n",encoding="utf-8")
                    os.replace(temporary,self.mapping_file)
                    self.mapping_error=None
                except OSError as error:self.mapping_error=str(error)
            return
        if message.topic == BARCODE_DIAGNOSTIC_TOPIC:
            if getattr(message,"retain",False):return
            try:record=decode_barcode_diagnostic(message.payload)
            except (TypeError,ValueError):
                with self.lock:self.rejected_packets+=1
                return
            with self.lock:
                self.diagnostic_received=time.monotonic()
                previous=self.barcode_diagnostics
                self.barcode_diagnostics=record
                if not previous or (record["sequence"],record["time_ms"])!=(previous["sequence"],previous["time_ms"]):
                    self.barcode_history.append(record)
                self.received_packets+=1;self.last_received_at=time.time()
            return
        if message.topic == ACK_TOPIC:
            if getattr(message, "retain", False):
                return
            data = message.payload
            if len(data) != 6 or data[0] != 0xC2 or data[2] > 3 or data[3] > 3 or data[4] > 12 or data[5] > 7:
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

    def send_command(self, action):
        opcodes = {"stop": 0, "reset": 2, "start": 3}
        if action not in opcodes:
            raise ValueError("Expected reset, start or stop.")
        locked = action != "stop"
        if locked and not self.command_lock.acquire(blocking=False):
            raise RuntimeError("A command is already being sent.")
        try:
            if action=="reset" and self.mapping_file:
                # STOP also cancels this boot's automatic start before a map
                # is loaded. RESET stays last and never requests motion.
                self._send_action("stop")
                self.restore_saved_mapping()
            result=self._send_action(action)
            if action=="reset" and self.mapping_file:
                result["message"]="Movement mapping restored. "+result["message"]
            return result
        finally:
            if locked:self.command_lock.release()

    def restore_saved_mapping(self):
        mapping=self.saved_mapping
        if self.mapping_file and self.mapping_file.exists():
            mapping=validate_mapping(json.loads(self.mapping_file.read_text(encoding="utf-8-sig")))
        if mapping is None:
            raise RuntimeError("No saved movement map. Send the instructor's mapping via MQTT first.")
        letters="".join(mapping[k] for k in MAP_ACTIONS).encode("ascii")
        entry={"letters":letters,"reply":None}
        with self.ack_condition:self.pending_map=entry
        try:
            info=self.command_client.publish(MAP_TOPIC,json.dumps(mapping),qos=1,retain=False)
            if info.rc!=0:raise RuntimeError("MQTT rejected the movement map.")
            info.wait_for_publish(timeout=2)
            if not info.is_published():raise RuntimeError("Movement map delivery is unconfirmed; robot remains stopped.")
            with self.ack_condition:
                if not self.ack_condition.wait_for(lambda:entry["reply"] is not None,timeout=3):
                    raise RuntimeError("RoboPico did not acknowledge the movement map; robot remains stopped.")
                if entry["reply"]!=0:raise RuntimeError("RoboPico rejected the movement map; robot remains stopped.")
        finally:
            with self.ack_condition:self.pending_map=None

    def _send_action(self,action):
        opcodes={"stop":0,"reset":2,"start":3}
        try:
            client = self.command_client
            if client is None or not client.is_connected():
                raise RuntimeError("MQTT is disconnected.")
            with self.ack_condition:
                sequence = self.command_sequence
                self.command_sequence = (sequence + 1) & 255
                entry = {"opcode": opcodes[action], "reply": None}
                self.pending_commands[sequence] = entry
            frame = bytes((0xC1, opcodes[action], START_POWER_PERCENT if action == "start" else 0,
                           0, 0, sequence))
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
                result, phase, fault = entry["reply"]
            faults = ("none", "sensor", "line", "shock", "transport", "run timeout", "uncalibrated", "motor")
            if result == 2:
                raise RuntimeError("RoboPico rejected Start: press Reset robot first after Stop or MQTT reconnect, then Start. Stop requires another Reset.")
            if result == 3:
                raise RuntimeError("RoboPico rejected an expired command. Try again after checking its state.")
            if result != 0:
                if action == "start" and phase == 0 and fault == 0:
                    raise RuntimeError("RoboPico is not ready to Start. Check that the MQTT letter mapping is loaded, sensors and encoders are fresh, and GP28 is on WHITE.")
                raise RuntimeError(f"RoboPico rejected {action}. Phase {phase}, fault {faults[fault]}. Reset requires stopped motors; Start requires reset and readiness.")
            return {"action": action, "acknowledged": True, "phase": phase, "fault": fault,
                    "message": f"RoboPico accepted {action}. Phase {phase}, fault {faults[fault]}."}
        finally:
            if "sequence" in locals():
                with self.ack_condition:
                    self.pending_commands.pop(sequence, None)

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
                "barcode_diagnostics": self.barcode_diagnostics,
                "barcode_diagnostics_age_s": now-self.diagnostic_received if self.diagnostic_received is not None else None,
                "barcode_diagnostic_history": list(self.barcode_history),
                "movement_mapping": self.saved_mapping,
                "mapping_save_error": self.mapping_error,
                "mapping_restore_on_reset": bool(self.mapping_file),
                "start_power_percent": START_POWER_PERCENT,
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

    def do_POST(self):
        if self.path != "/api/command":
            self.send_error(404)
            return
        # Accept commands only from the local dashboard's own browser origin.
        origin = f"http://127.0.0.1:{self.server.server_port}"
        if self.headers.get("Origin") != origin or self.headers.get("Host") != origin[7:]:
            self.send_error(403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 128 or self.headers.get_content_type() != "application/json":
                raise ValueError("Expected a small JSON command.")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or set(payload) != {"action"} or not isinstance(payload["action"], str):
                raise ValueError("Expected one action field.")
            result = self.bridge.send_command(payload["action"])
            status = 200
        except (ValueError, json.JSONDecodeError) as error:
            result, status = {"error": str(error)}, 400
        except (RuntimeError, OSError) as error:
            result, status = {"error": str(error)}, 503
        data = json.dumps(result).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
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
    parser.add_argument("--mapping-file",type=Path,help="PC-side saved MQTT map; Reset stops, reapplies it and resets without starting")
    args = parser.parse_args()
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.exit(1, "Install MQTT support with: python -m pip install -r app_program/external_systems/Dashboard/requirements.txt\n")
    bridge = BrokerBridge(args.mapping_file)
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
