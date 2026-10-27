"""Compare production odometry before/after storage changes, without hardware.

Run under a host environment with gcc. Requires the recorded baseline Git
commit. Outputs timings plus an identical behavior checksum, not RP2040 WCET.
Build products stay in the ignored host .build directory.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BASELINE = "5f0b8c7"
WORK = ROOT / "app_program/subsystem4/tests/host/.build/odometry_benchmark"
HARNESS = r'''
#include "odometry.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#ifdef BASELINE
#define SPEED(s) ((s).speed_mm_s)
#define DELTA(s) ((s).delta_um)
#define TURN(s) ((s).turn_mdeg)
#else
#define SPEED(s) odom_speed_mm_s(&(s))
#define DELTA(s) odom_delta_um(&(s))
#define TURN(s) odom_turn_mdeg(&(s))
#endif
int main(void) {
    odom_config_t c={100000,100,40};odom_t s={0};
    uint32_t rng=1,left=UINT32_MAX-1000U,right=left;
    uint64_t hash=UINT64_C(1469598103934665603);
    const unsigned steps=2000000U;
    odom_step(&s,&c,0,left,right,1);
    clock_t start=clock();
    for(unsigned n=1;n<=steps;++n) {
        rng=rng*1664525U+1013904223U;
        left+=(uint32_t)((int32_t)(rng%4001U)-2000);
        rng=rng*1664525U+1013904223U;
        right+=(uint32_t)((int32_t)(rng%4001U)-2000);
        odom_step(&s,&c,n,left,right,n%997U!=0U);
        uint32_t fields[]={s.time_ms,s.left,s.right,(uint32_t)s.distance_um,
            (uint32_t)SPEED(s),(uint32_t)DELTA(s),(uint32_t)TURN(s),
            s.initialized,s.valid};
        for(unsigned k=0;k<sizeof(fields)/sizeof(fields[0]);++k) {
            hash^=fields[k];hash*=UINT64_C(1099511628211);
        }
    }
    double ns=(double)(clock()-start)*1e9/CLOCKS_PER_SEC/steps;
    printf("%llu %.3f\n",(unsigned long long)hash,ns);
    return 0;
}
'''


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    reference = WORK / "reference"
    names = ["subsystem2/odometry.c", "subsystem2/odometry.h", "shared/edr.c", "shared/edr.h"]
    hashes = {}
    for name in names:
        data = subprocess.check_output(["git", "show", f"{BASELINE}:app_program/{name}"], cwd=ROOT)
        destination = reference / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    harness = WORK / "benchmark.c"
    harness.write_text(HARNESS, encoding="utf-8")
    result = {"baseline_commit": subprocess.check_output(["git", "rev-parse", BASELINE], cwd=ROOT, text=True).strip(),
              "baseline_sha256": hashes,
              "current_sha256": {name: hashlib.sha256((ROOT / "app_program" / name).read_bytes()).hexdigest() for name in names},
              "iterations_per_run": 2000000,
              "scope": "Native host step plus decode/checksum workload; not target timing", "runs": {}}
    compiler = os.environ.get("CC", "gcc")
    result["compiler"] = subprocess.check_output([compiler, "--version"], text=True).splitlines()[0]
    checksums = set()
    for name, source in [("before", reference), ("after", ROOT / "app_program")]:
        binary = WORK / (name + (".exe" if os.name == "nt" else ""))
        args = [compiler, "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(source / "subsystem2")]
        if name == "before":
            args.append("-DBASELINE")
        subprocess.run(args + [str(harness), str(source / "subsystem2/odometry.c"),
                              str(source / "shared/edr.c"), "-o", str(binary)], check=True)
        records = []
        for _ in range(3):
            checksum, ns = subprocess.check_output([str(binary)], text=True).split()
            checksums.add(checksum)
            records.append({"checksum": checksum, "ns_per_iteration": float(ns)})
        result["runs"][name] = records
    if len(checksums) != 1:
        raise RuntimeError("Before/after odometry behavior checksums differ")
    result["behavior_checksum_agrees"] = True
    (ROOT / "testing/subsystem4/ready-odometry-benchmark.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
