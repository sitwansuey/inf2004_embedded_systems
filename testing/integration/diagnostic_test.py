"""Production C frame -> production Python decoder/dashboard; malformed frames."""
from pathlib import Path
import json
import subprocess
import sys
from types import SimpleNamespace
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/"tools"))
sys.path.insert(0,str(root/"app_program/external_systems/Dashboard"))
from barcode_diagnostic import decode, TOPIC
from dashboard import BrokerBridge
payload=bytes.fromhex(subprocess.check_output([str(Path(".build/diagnostic").resolve())]).decode().strip())
d=decode(payload)
assert len(payload)==192 and d["sequence"]==2**32-1 and d["result_id"]==3
assert d["left_counts"]==-123 and d["acquisition_error"]==-50 and d["right_counts"]==456
assert d["decoded_symbol"]=="Z" and d["stored_symbol"]=="Z" and d["reverse"] and d["junction_abort"]
assert d["reason"]=="pattern" and d["sample_age_ms"]==10 and len(d["runs_ms"])==29
assert d["runs_ms"]==[40 if n%2==0 else 120 for n in range(29)]
for bits,direction in ((0,"none"),(64,"right"),(192,"left")):
    correction=bytearray(payload);correction[69]=bits
    assert decode(correction)["scan_line_correction"]==direction
for length in (0,1,191,193):
    try:decode((payload+b"\x00")[:length])
    except ValueError:pass
    else:raise AssertionError("accepted wrong length")
for offset,value in ((0,0),(1,2),(2,3),(3,8),(64,13),(65,8),(66,101),(67,5),(68,64),(69,128),(70,4),(72,8),(73,27),(74,8),(75,7)):
    broken=bytearray(payload);broken[offset]=value
    try:decode(broken)
    except ValueError:pass
    else:raise AssertionError((offset,value))
b=BrokerBridge();m=SimpleNamespace(topic=TOPIC,payload=payload,retain=True)
b.on_message(None,None,m);assert b.snapshot()["barcode_diagnostics"] is None
m.retain=False;b.on_message(None,None,m);b.on_message(None,None,m)
assert b.snapshot()["barcode_diagnostics"]["stored_symbol"]=="Z" and len(b.barcode_history)==1
b.reset_observations();assert b.snapshot()["barcode_diagnostics_age_s"] is None
assert b.snapshot()["barcode_diagnostics"]["result_id"]==3
subprocess.run(["node",str(root/"app_program/external_systems/Dashboard/test_dashboard_display.cjs")],
               input=json.dumps(b.snapshot()).encode(),check=True)
print("C/Python barcode diagnostics: exact framing, signed data,29 widths, malformed ranges, retained/dedup/history PASS")
