"""Independent Python/C receiver agreement on actual production result codec."""
import importlib.util
from pathlib import Path
import random
import subprocess

ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location("buddy4_result",ROOT/"tools/buddy4_result.py")
decoder=importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)
exe=Path(__file__).parent/".build/result_test"
if exe.with_suffix(".exe").exists():exe=exe.with_suffix(".exe")
rng=random.Random(2004)
valid=invalid=0
for n in range(256):
    v=[rng.randrange(lo,hi+1) for _,_,lo,hi in decoder.FIELDS]
    if n<8:
        v=[hi if n&1 else lo for _,_,lo,hi in decoder.FIELDS]
    word=offset=0
    for value,(_,bits,minimum,_) in zip(v,decoder.FIELDS):
        word|=(value-minimum)<<offset;offset+=bits
    payload=word.to_bytes(12,"little")
    try:
        expected=decoder.decode(payload)
    except ValueError:
        expected=None
    result=subprocess.run([str(exe),payload.hex()],capture_output=True,text=True)
    if expected is None:
        assert result.returncode!=0;invalid+=1
    else:
        keys=["calculated_ms","pitch_cdeg","roll_cdeg","height_mm","maximum_mm","terrain","flags","faults"]
        assert result.returncode==0 and list(map(int,result.stdout.split()))==[expected[k] for k in keys]
        assert decoder.decode_frame(bytes((0x54,12))+payload)==expected
        valid+=1
for bit in range(93,96):
    payload=(1<<bit).to_bytes(12,"little")
    assert subprocess.run([str(exe),payload.hex()],capture_output=True).returncode!=0
    try:decoder.decode(payload)
    except ValueError:invalid+=1
    else:raise AssertionError("padding accepted")
print(f"S4-U31 result receivers: {valid} valid and {invalid} rejected C/Python vectors PASS")
