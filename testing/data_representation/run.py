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
    "app_program/subsystem5/s5_data.c",
    "app_program/subsystem5/s5_data.h",
    "app_program/subsystem5/subsystem5.c",
    "app_program/subsystem5/subsystem5.h",
    "app_program/subsystem5/s5_policy.c",
    "app_program/subsystem5/s5_scan.c",
    "app_program/subsystem5/s5_width.c",
    "testing/subsystem5/test_data.c",
    "testing/subsystem5/host_app.h",
    "tools/edr_decode.py",
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
    vectors = json.loads((HERE / "vectors.json").read_text())
    lines = ["/* Generated only in the temporary test directory from vectors.json. */",
             "static const edr_test_packet_t g_edr_test_packets[] =", "{"]
    for valid, collection in ((True, "valid_packets"), (False, "invalid_packets")):
        for vector in vectors[collection]:
            packet = bytes.fromhex(vector["hex"])
            byte_values = ", ".join(f"0x{value:02x}U" for value in packet) or "0U"
            motion, encoder = vector.get("motion", {}), vector.get("encoder", {})
            values = [motion.get(key, 0) for key in ("motor_word", "faults", "command_ms")]
            values += [encoder.get(key, 0) for key in ("left_counts", "right_counts", "sampled_ms", "flags")]
            literals = ", ".join(f"UINT32_C({value})" for value in values)
            lines.append("    {" + f'"{vector["name"]}", {str(valid).lower()}, '
                         + f'{vector.get("source", 0)}U, {vector.get("kind", 0)}U, '
                         + f"{len(packet)}U, {{{byte_values}}}, {literals}" + "},")
    lines += ["};", "static const edr_test_delta_t g_edr_test_deltas[] =", "{"]
    for vector in vectors["count_deltas"]:
        delta = vector["delta"]
        delta_literal = f"INT32_C({delta})" if delta is not None else "INT32_C(0)"
        lines.append("    {" + f'UINT32_C({vector["current"]}), UINT32_C({vector["previous"]}), '
                     + f"{str(delta is not None).lower()}, {delta_literal}" + "},")
    lines += ["};", "static const edr_test_age_t g_edr_test_ages[] =", "{"]
    for vector in vectors["freshness"]:
        literals = ", ".join(f"UINT32_C({vector[key]})" for key in ("now_ms", "sampled_ms", "maximum_ms"))
        lines.append("    {" + literals + f', {str(vector["fresh"]).lower()}' + "},")
    lines += ["};", ""]
    path.write_text("\n".join(lines))


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
    report = {"schema_version": 1, "passed": False, "stages": [],
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
                  lambda: subprocess.run(
                      [sys.executable, "-B", "-m", "unittest", "discover",
                       "-s", str(HERE), "-p", "test_*.py", "-v"],
                      cwd=ROOT, check=True), report["stages"])
        run_stage("C99 codec, golden packets and all-buddy framing",
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
