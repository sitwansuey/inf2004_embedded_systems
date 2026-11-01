#!/usr/bin/env python3
"""Speak robot warnings from the project's single EDR16 MQTT topic.

Run on Raspberry Pi OS: python tools/robot_audio.py --broker BROKER_IP
Keep this file beside tools/edr_decode.py. No GPIO access or driving commands.
"""
import argparse
from dataclasses import dataclass
import logging
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

from edr_decode import MQTT_TOPIC, decode_packet

LOG = logging.getLogger("robot_audio")
MESSAGES = {
    "obstacle": "Warning. Obstacle detected.",
    "off_course": "Warning. Off course. No line detected by the front sensors.",
}


@dataclass
class Alert:
    since: object = None
    sample: object = None
    count: int = 0
    last_spoken: float = -math.inf

    def observe(self, active, sample, now):
        # Missing/stale observations are unknown, never evidence of no line.
        if not active:
            self.since, self.sample, self.count = None, None, 0
        elif sample != self.sample:
            if self.since is None:
                self.since = now
            self.sample = sample
            self.count += 1

    def due(self, now, confirm, repeat):
        return (self.since is not None and self.count >= 2
                and now - self.since >= confirm
                and now - self.last_spoken >= repeat)


class Monitor:
    """Thread-safe MQTT observations; decisions run in the main thread."""
    def __init__(self, obstacle_cm=20, clear_cm=25, stale=1.0,
                 confirm=0.3, repeat=5.0, all_phases=False):
        self.lock = threading.Lock()
        self.obstacle_cm, self.clear_cm = obstacle_cm, clear_cm
        self.stale, self.confirm, self.repeat = stale, confirm, repeat
        self.all_phases = all_phases
        self.reset()

    def reset(self):
        with self.lock:
            self.values = {}
            self.alerts = {name: Alert() for name in MESSAGES}
            self.obstacle_active = False

    def receive(self, topic, payload, now=None, retained=False):
        if topic != MQTT_TOPIC or retained:
            return False
        try:
            record = decode_packet(payload)
        except (TypeError, ValueError):
            return False
        now = time.monotonic() if now is None else now
        sensor, value = record["id"], record["data"]
        key = None
        if sensor == 4:
            key = "distance"
        elif sensor in (5, 6):
            key = "left_ir" if sensor == 5 else "right_ir"
        elif sensor == 0:
            subtype, value = record["subtype"], record["status_value"]
            if subtype == 0:
                key = "phase"
            elif subtype == 2:
                key = "line"
            elif subtype == 10:
                key = "available"
        if key is not None:
            with self.lock:
                self.values[key] = (value, now)
        return True

    def _fresh(self, key, now):
        item = self.values.get(key)
        return item if item is not None and 0 <= now - item[1] <= self.stale else None

    def ready(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            available = self._fresh("available", now)
            distance = self._fresh("distance", now)
            if available and available[0] & 16 and distance and distance[0] > 0:
                limit = self.clear_cm if self.obstacle_active else self.obstacle_cm
                self.obstacle_active = distance[0] <= limit
                self.alerts["obstacle"].observe(self.obstacle_active, distance[1], now)
            else:
                self.obstacle_active = False
                self.alerts["obstacle"].observe(False, None, now)

            line = self._fresh("line", now)
            phase = self._fresh("phase", now)
            # TRACK=1, REACQUIRE=10, or FAULT=12 with LINE fault=2.
            # Leaving the line during an intentional turn/bypass is expected.
            phase_allowed = self.all_phases or (phase is not None and (
                (phase[0] & 15) in (1, 10) or phase[0] == (12 | (2 << 4))))
            usable = (available and available[0] & 4 and line
                      and self._fresh("left_ir", now) and self._fresh("right_ir", now))
            if usable and phase_allowed:
                state, black_mask = line[0] & 7, line[0] >> 3
                # GP26/GP27 = physical IR bits 0/1. Ignore barcode bit 2.
                missing = state in (2, 3) and (black_mask & 3) == 0
                self.alerts["off_course"].observe(missing, line[1], now)
            else:
                self.alerts["off_course"].observe(False, None, now)

            ready = [name for name, alert in self.alerts.items()
                     if alert.due(now, self.confirm, self.repeat)]
            # Obstacle wins ties; afterwards the older pending warning wins.
            return sorted(ready, key=lambda name: self.alerts[name].last_spoken)

    def announced(self, name, now=None):
        with self.lock:
            self.alerts[name].last_spoken = time.monotonic() if now is None else now


class Audio:
    """One nonblocking player, no queued historical warnings or shell commands."""
    def __init__(self, obstacle_wav=None, line_wav=None, device=None, dry_run=False):
        self.files = {"obstacle": obstacle_wav, "off_course": line_wav}
        self.device, self.dry_run = device, dry_run
        self.process, self.started = None, 0.0
        self.speaker, self.player = shutil.which("espeak-ng"), shutil.which("aplay")
        if dry_run:
            return
        for filename in self.files.values():
            if filename and not filename.is_file():
                raise ValueError(f"Audio file not found: {filename}")
        if any(self.files.values()) and not self.player:
            raise ValueError("Install WAV playback: sudo apt install alsa-utils")
        if not all(self.files.values()) and not self.speaker:
            raise ValueError("Install spoken warnings: sudo apt install espeak-ng")
        if device and not all(self.files.values()):
            raise ValueError("--audio-device requires both WAV files; speech uses the OS default output")

    def busy(self):
        if self.process is None:
            return False
        code = self.process.poll()
        if code is not None:
            if code:
                LOG.error("Audio player exited with code %s; check the Pi audio output", code)
            self.process = None
            return False
        if time.monotonic() - self.started > 15:
            LOG.error("Audio playback timed out")
            self.close()
            return False
        return True

    def play(self, name):
        LOG.warning("%s", MESSAGES[name])
        if self.dry_run:
            return
        filename = self.files[name]
        if filename:
            command = [self.player, "-q"]
            if self.device:
                command += ["-D", self.device]
            command += [str(filename.resolve())]
        else:
            command = [self.speaker, "-s", "155", MESSAGES[name]]
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL)
        self.started = time.monotonic()

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1)
            self.process = None


def connect_client(monitor, args):
    import paho.mqtt.client as mqtt

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if args.username:
        client.username_pw_set(args.username, os.getenv("MQTT_PASSWORD"))
    if args.tls:
        client.tls_set(ca_certs=str(args.ca_cert) if args.ca_cert else None)

    def on_connect(client, userdata, flags, reason, properties):
        monitor.reset()
        if reason.is_failure:
            LOG.error("MQTT connection rejected: %s", reason)
            return
        result, _ = client.subscribe(MQTT_TOPIC, qos=0)
        if result != mqtt.MQTT_ERR_SUCCESS:
            LOG.error("MQTT subscription request failed: %s", result)
        else:
            LOG.info("Connected; subscribing to %s", MQTT_TOPIC)

    def on_subscribe(client, userdata, mid, reasons, properties):
        if any(reason.is_failure for reason in reasons):
            LOG.error("Broker rejected telemetry subscription")
        else:
            LOG.info("Audio monitor ready; waiting for fresh EDR16 observations")

    def on_disconnect(client, userdata, flags, reason, properties):
        monitor.reset()
        LOG.warning("MQTT disconnected; warnings paused until fresh data returns")

    def on_message(client, userdata, message):
        monitor.receive(message.topic, message.payload, retained=message.retain)

    client.on_connect, client.on_subscribe = on_connect, on_subscribe
    client.on_disconnect, client.on_message = on_disconnect, on_message
    client.on_connect_fail = lambda client, userdata: LOG.warning("MQTT unavailable; retrying")
    client.reconnect_delay_set(min_delay=1, max_delay=10)
    client.connect_async(args.broker, args.port, keepalive=30)
    return client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="localhost", help="Same broker used by the Pico/dashboard")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    parser.add_argument("--tls", action="store_true", help="Use verified TLS; set --port as needed")
    parser.add_argument("--ca-cert", type=Path)
    parser.add_argument("--obstacle-cm", type=int, default=20)
    parser.add_argument("--clear-cm", type=int, default=25, help="Obstacle hysteresis clearance")
    parser.add_argument("--stale-seconds", type=float, default=1.0)
    parser.add_argument("--confirm-seconds", type=float, default=0.3)
    parser.add_argument("--repeat-seconds", type=float, default=5.0)
    parser.add_argument("--all-phases", action="store_true", help="Also warn about line loss while parked/turning/bypassing")
    parser.add_argument("--obstacle-wav", type=Path)
    parser.add_argument("--off-course-wav", type=Path)
    parser.add_argument("--audio-device", help="ALSA device for WAV files, e.g. plughw:1,0")
    parser.add_argument("--dry-run", action="store_true", help="Log warnings without audio")
    parser.add_argument("--test-audio", action="store_true", help="Play both warnings once without MQTT")
    args = parser.parse_args()
    if not 1 <= args.obstacle_cm < args.clear_cm <= 4095:
        parser.error("Require 1 <= obstacle-cm < clear-cm <= 4095")
    if not 1 <= args.port <= 65535:
        parser.error("Invalid broker port")
    if args.ca_cert and not args.tls:
        parser.error("--ca-cert requires --tls")
    if (not all(math.isfinite(n) for n in (args.stale_seconds, args.confirm_seconds, args.repeat_seconds))
            or args.stale_seconds <= 0 or args.confirm_seconds < 0 or args.repeat_seconds <= 0):
        parser.error("Timing values must be finite: stale/repeat > 0 and confirm >= 0")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        audio = Audio(args.obstacle_wav, args.off_course_wav, args.audio_device, args.dry_run)
    except ValueError as error:
        parser.error(str(error))
    client = None
    try:
        if args.test_audio:
            for name in MESSAGES:
                audio.play(name)
                while audio.busy():
                    time.sleep(0.1)
            return
        monitor = Monitor(args.obstacle_cm, args.clear_cm, args.stale_seconds,
                          args.confirm_seconds, args.repeat_seconds, args.all_phases)
        try:
            client = connect_client(monitor, args)
        except ImportError:
            parser.error("Install MQTT support: python -m pip install 'paho-mqtt>=2.1,<3'")
        client.loop_start()
        LOG.info("Monitoring %s:%d; obstacle threshold %d cm", args.broker, args.port, args.obstacle_cm)
        while True:
            ready = monitor.ready()
            if not audio.busy() and ready:
                try:
                    audio.play(ready[0])
                except OSError as error:
                    LOG.error("Cannot start audio: %s", error)
                monitor.announced(ready[0])
            time.sleep(0.1)
    except KeyboardInterrupt:
        LOG.info("Audio monitor stopped")
    finally:
        if client is not None:
            client.disconnect()
            client.loop_stop()
        audio.close()


if __name__ == "__main__":
    main()
