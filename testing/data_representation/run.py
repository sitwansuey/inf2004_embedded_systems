#!/usr/bin/env python3
"""Run the hardware-free pre-integration gate for the shared data contract."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
INPUTS = (
    "app_program/shared/edr.h",
    "app_program/shared/edr.c",
    "tools/edr_decode.py",
    "app_program/subsystem1/packed16.h",
    "app_program/subsystem1/tests/host/packed16_test.c",
    "app_program/subsystem1/tests/host/telemetry_host_test.c",
    "app_program/external_systems/Dashboard/test_packed16.py",
    "app_program/external_systems/Dashboard/test_dashboard_display.cjs",
    "app_program/external_systems/Dashboard/dashboard.js",
    "app_program/subsystem1/telemetry.c",
    "app_program/subsystem1/task_telemetry.c",
    "app_program/external_systems/Dashboard/packed16.py",
    "app_program/external_systems/Dashboard/dashboard.py",
    "testing/data_representation/vectors.json",
    "testing/data_representation/test_edr.c",
    "testing/data_representation/run.py",
    "app_program/subsystem2/subsystem2.c",
    "app_program/subsystem2/subsystem2.h",
    "app_program/subsystem2/subsystem2_test.c",
    "app_program/subsystem2/s2_data.c",
    "app_program/subsystem2/s2_data.h",
    "app_program/subsystem2/tests/host/run.py",
    "app_program/subsystem2/tests/host/subsystem2_host_test.c",
    "app_program/subsystem2/tests/host/app.h",
    "app_program/subsystem2/tests/host/board_pins.h",
    "app_program/subsystem2/tests/host/tk/syslib.h",
)


def input_fingerprint() -> dict:
    """Normalize checkout line endings so teammates can compare test inputs."""
    hashes = {}
    paths = set(INPUTS)
    paths.update(path.relative_to(ROOT).as_posix()
                 for path in HERE.glob("test_*.py"))
    for relative in sorted(paths):
        content = (ROOT / relative).read_text(encoding="utf-8")
        if relative.endswith("vectors.json"):
            content = json.dumps(json.loads(content), sort_keys=True,
                                 separators=(",", ":"))
        hashes[relative] = hashlib.sha256(content.encode("utf-8")).hexdigest()
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode("utf-8"))
    return {"sha256": digest.hexdigest(), "files": hashes}


def revision_info() -> dict:
    """Git is optional for teammates testing an exported source directory."""
    try:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                  capture_output=True, text=True, check=True)
        status = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                                capture_output=True, text=True, check=True)
        return {"commit": revision.stdout.strip(),
                "working_tree_modified": bool(status.stdout.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "working_tree_modified": None}


def run_stage(name: str, action, stages: list) -> None:
    print(f"\n[CHECK] {name}", flush=True)
    stage = {"name": name, "passed": False}
    stages.append(stage)
    started = time.monotonic()
    try:
        action()
        stage["passed"] = True
    finally:
        stage["seconds"] = round(time.monotonic() - started, 3)
    print(f"[PASS] {name}", flush=True)


def write_c_vectors(path: Path) -> None:
    """Translate fixture data to C literals without using the Python codec."""
    data = json.loads((HERE / "vectors.json").read_text())
    lines = ["static const vector_t vectors[] = {"]
    for valid, key in ((True,"valid_packets"),(False,"invalid_packets")):
        for v in data[key]:
            packet=bytes.fromhex(v["hex"])
            values=", ".join(f"0x{x:02x}U" for x in packet) or "0U"
            lines.append("{{"+values+"}, "+str(len(packet))+"U, "+str(valid).lower()+
                         ", "+str(v.get("id",0))+"U, "+str(v.get("data",0))+"U},")
    lines.append("};")
    path.write_text("\n".join(lines),encoding="utf-8")


def run_codec_tests(compiler: str) -> None:
    # tempfile owns cleanup; no recursive deletion of a workspace path.
    with tempfile.TemporaryDirectory(prefix="edr-tests-") as directory:
        temporary = Path(directory)
        write_c_vectors(temporary / "edr_test_vectors.h")
        executable = temporary / ("edr_test.exe" if os.name == "nt" else "edr_test")
        command = [compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-Wconversion", "-Wsign-conversion", "-Wshadow", "-Wstrict-prototypes",
                   "-pedantic", "-I", str(ROOT / "app_program/shared"), "-I", str(temporary),
                   str(ROOT / "app_program/shared/edr.c"), str(HERE / "test_edr.c"),
                   "-o", str(executable)]
        print("C compiler:", compiler, flush=True)
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=ROOT, check=True)

    # S1 queue/binary-length regression tests use the same shared header.
    with tempfile.TemporaryDirectory(prefix="s1-edr-tests-") as directory:
        for name in ("packed16_test", "telemetry_host_test"):
            executable = Path(directory) / (name + (".exe" if os.name == "nt" else ""))
            subprocess.run([compiler,"-std=c99","-Wall","-Wextra","-Werror",
                str(ROOT / "app_program/subsystem1/tests/host" / (name+".c")),
                "-o",str(executable)],check=True)
            subprocess.run([str(executable)],check=True)


def run_python_tests():
    for directory, pattern in ((HERE,"test_*.py"),
            (ROOT / "app_program/external_systems/Dashboard", "test_packed16.py")):
        subprocess.run([sys.executable,"-B","-m","unittest","discover",
                        "-s",str(directory),"-p",pattern,"-v"],cwd=ROOT,check=True)


def run_motor_tests(compiler: str) -> None:
    path = ROOT / "app_program/subsystem2/tests/host/run.py"
    spec = importlib.util.spec_from_file_location("edr_motor_tests", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.run_tests(compiler)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("CC", "gcc"),
                        help="native GCC/Clang executable (not ARM cross GCC)")
    parser.add_argument("--report", type=Path,
                        help="write a shareable JSON result, including failures")
    arguments = parser.parse_args()
    report = {"schema_version": 2, "passed": False, "stages": [],
              "started_utc": datetime.now(timezone.utc).isoformat(),
              "python": platform.python_version(), "platform": sys.platform,
              "revision": revision_info()}
    result = 1
    sys.dont_write_bytecode = True
    try:
        compiler = shutil.which(arguments.compiler)
        if compiler is None:
            raise ValueError(f"Native C compiler not found: {arguments.compiler}. "
                             "Install GCC/Clang and add it to PATH, or use "
                             "--compiler /path/to/compiler.")
        version = subprocess.run([compiler, "--version"], capture_output=True,
                                 text=True, check=True)
        report["compiler"] = {"name": Path(compiler).name,
                              "version": version.stdout.splitlines()[0]}
        report["inputs"] = input_fingerprint()
        print("EDR pre-integration check (no Pico, SDK or broker required)", flush=True)
        run_stage("Python wire contract and dashboard decoder",
                  run_python_tests, report["stages"])
        run_stage("C99 codec, golden packets and exhaustive 16-bit records",
                  lambda: run_codec_tests(compiler), report["stages"])
        run_stage("Buddy 2 adapters and watchdog (eight host profiles)",
                  lambda: run_motor_tests(compiler), report["stages"])
        if input_fingerprint() != report["inputs"]:
            raise ValueError("Test inputs changed during the run; rerun the check.")
        report["passed"] = True
        result = 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        report["error"] = str(error)
        print(f"\nEDR CHECK FAIL: {error}", file=sys.stderr, flush=True)
    if arguments.report is not None:
        try:
            arguments.report.write_text(json.dumps(report, indent=2) + "\n",
                                        encoding="utf-8")
            print(f"Report: {arguments.report}", flush=True)
        except OSError as error:
            print(f"EDR CHECK FAIL: cannot write report: {error}", file=sys.stderr)
            result = 1
    if result == 0:
        print("\nEDR CHECK PASS: all three stages passed. "
              "Run again after your integration changes.", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
