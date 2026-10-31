"""Cross-language wire checks with boundary, random, and malformed packets."""
import importlib.util
import json
import random
import subprocess
from pathlib import Path
R = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("decoder", R/"app_program/external_systems/Dashboard/decoder.py")
decoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)
random.seed(2004)
fields = decoder.SCHEMA
cases = [{f["name"]: f["min"] for f in fields}, {f["name"]: f["max"] for f in fields}]
for _ in range(300):
    cases.append({f["name"]: random.randint(f["min"], f["max"]) for f in fields})
invalid = []
for f in fields:
    for value in (f["min"]-1, f["max"]+1):
        case = cases[0].copy()
        case[f["name"]] = value
        invalid.append(case)
all_cases = cases + invalid
run = subprocess.run([str(R/"testing/integration/.build/protocol")], input="\n".join(" ".join(str(c[f["name"]]) for f in fields) for c in all_cases), text=True, capture_output=True, check=True)
lines = run.stdout.splitlines()
assert len(lines) == len(all_cases)
for case, output in zip(cases, lines):
    raw = bytes.fromhex(output)
    assert decoder.encode(case) == raw
    assert decoder.decode(raw) == case
assert all(line == "REJECT" for line in lines[len(cases):])
for raw in [b"", bytes(decoder.SIZE-1), bytes(decoder.SIZE+1), b"\xff" * decoder.SIZE]:
    try:
        decoder.decode(raw)
        raise AssertionError("Malformed frame accepted")
    except ValueError:
        pass
golden={"schema_bits":decoder.BITS,"bytes":decoder.SIZE,"minimum":lines[0],"maximum":lines[1],"random_cases":300,"rejected_bounds":len(invalid)}
(R/"testing/integration/protocol_golden.json").write_text(json.dumps(golden,indent=2)+"\n")
print(f"INT-CR1-05 protocol: {len(cases)} C/Python vectors, {len(invalid)} bound rejections, malformed length/version/padding PASS")
