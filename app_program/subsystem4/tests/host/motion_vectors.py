"""Independent Python/C agreement for B4 motion boundaries and malformed data."""
import importlib.util
from pathlib import Path
import random
import subprocess
root=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('motion',root/'app_program/external_systems/Dashboard/buddy4_motion.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
exe=Path(__file__).parent/'.build/monitoring_test'
if exe.with_suffix('.exe').exists():exe=exe.with_suffix('.exe')
rng=random.Random(4404);accepted=rejected=0
for case in range(400):
    word=offset=0
    for _,bits,lo,hi in module.FIELDS:
        value=(lo if case==0 else hi if case==1 else rng.randint(lo,hi))
        word|=(value-lo)<<offset;offset+=bits
    if case>=393:word|=1<<(73+case-393)
    payload=word.to_bytes(10,'little')
    try:expected=module.decode(payload)
    except ValueError:expected=None
    result=subprocess.run([str(exe),payload.hex()],capture_output=True,text=True)
    if expected is None:
        assert result.returncode!=0;rejected+=1
    else:
        assert result.returncode==0
        assert list(map(int,result.stdout.split()))==list(expected.values())
        accepted+=1
print(f'S4-U40 PASS motion C/Python receivers: {accepted} accepted, {rejected} rejected vectors')
