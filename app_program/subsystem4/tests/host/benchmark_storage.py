"""Compare the actual checkpoint data implementation with compact storage."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--compiler",default=os.environ.get("CC","gcc"))
parser.add_argument("--report",type=Path,required=True)
args=parser.parse_args()
results={}
with tempfile.TemporaryDirectory(prefix="s4-cr1-benchmark-") as directory:
    before=Path(directory)
    inputs={}
    for name in ("sensor_data.c","sensor_data.h","drv_lsm303dlhc.h"):
        content=subprocess.check_output(["git","show","373ad6d:app_program/subsystem4/"+name],cwd=ROOT)
        (before/name).write_bytes(content)
        inputs[name]=hashlib.sha256(content).hexdigest()
    for label,folder,flags in (("checkpoint",before,["-DS4_CHECKPOINT_BENCHMARK"]),
                              ("compact",ROOT/"app_program/subsystem4",[])):
        exe=before/(label+(".exe" if os.name=="nt" else ""))
        subprocess.run([args.compiler,"-std=c11","-O2","-Wall","-Wextra","-Werror",*flags,
                        "-I",str(folder),str(HERE/"storage_benchmark.c"),str(folder/"sensor_data.c"),
                        "-o",str(exe)],check=True)
        results[label]=[json.loads(subprocess.check_output([str(exe)],text=True)) for _ in range(3)]
    assert results["checkpoint"][0]["checksum"]==results["compact"][0]["checksum"]
report={"scope":"Native host only; excludes RTOS/I2C and is not ARM timing",
        "checkpoint":"373ad6d","checkpoint_input_sha256":inputs,"results":results}
args.report.write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print(json.dumps(results,indent=2))
