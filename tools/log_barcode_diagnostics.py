"""Passive JSONL logging of barcode diagnostics over MQTT. Sends no commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import threading
import paho.mqtt.client as mqtt
from barcode_diagnostic import TOPIC, decode

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8084)
    p.add_argument("--output", type=Path, required=True)
    a=p.parse_args();done=threading.Event()
    signal.signal(signal.SIGINT,lambda *args:done.set())
    with a.output.open("a",encoding="utf-8",buffering=1) as log:
        client=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        def message(c,u,m):
            if m.retain:return
            record={"received_at":datetime.now().astimezone().isoformat()}
            try:record.update(decode(m.payload))
            except ValueError as error:record.update(error=str(error),payload_hex=m.payload.hex())
            log.write(json.dumps(record)+"\n")
        client.on_connect=lambda c,u,f,r,pr:c.subscribe(TOPIC,qos=1) if not r.is_failure else None
        client.on_message=message
        client.connect(a.broker,a.port,30);client.loop_start()
        try:done.wait()
        finally:client.disconnect();client.loop_stop()
if __name__=="__main__":main()
