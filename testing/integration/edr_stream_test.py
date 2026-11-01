"""Production C snapshot -> 2-byte MQTT records -> Python -> browser cards."""
import json, os, shutil, subprocess, sys
from pathlib import Path
from types import SimpleNamespace
R=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/"app_program/external_systems/Dashboard"))
from dashboard import BrokerBridge
from packed16 import PACKED_TOPIC
exe=R/"testing/integration/.build"/("edr_stream.exe" if os.name=="nt" else "edr_stream")
lines=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout.splitlines()
assert len(lines)==31
bridge=BrokerBridge()
for line in lines:
    payload=bytes.fromhex(line);assert len(payload)==2
    bridge.on_message(None,None,SimpleNamespace(topic=PACKED_TOPIC,payload=payload))
snapshot=bridge.snapshot()
assert snapshot['received_packets']==31 and snapshot['rejected_packets']==0
values={m['topic'].rsplit('/',1)[-1]:m['payload'] for m in snapshot['messages']}
for key,value in {'encoder_left':'100','encoder_right':'4095','ir_left':'111','ir_right':'222',
                  'barcode':'D','navigation':'U-turn','hump_height_cm':12.5,'run_max_height_cm':45.5,
                  'width_cm':10.5,'servo_angle_deg':'60','pitch_deg':-12,'roll_deg':4,
                  'hump_state':'Crest','speed_mps':-0.36,'sequence':1,'dropped':255}.items():
    assert values[key]==value,(key,values[key],value)
node=shutil.which('node')
if node:
    subprocess.run([node,str(R/'app_program/external_systems/Dashboard/test_dashboard_display.cjs')],
                   input=json.dumps(snapshot),text=True,check=True)
print('EDR16 production snapshot through Python/dashboard PASS')
