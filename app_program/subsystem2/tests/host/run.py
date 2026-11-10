#!/usr/bin/env python3
"""Build and run all Buddy 2 host profiles without the Pico SDK or hardware."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


HERE = Path(__file__).resolve().parent
SUBSYSTEM = HERE.parents[1]
SHARED = SUBSYSTEM.parent / "shared"

PROFILES = (
    ("standalone-no-encoder", ("-DAPP_S2_ONLY=1",)),
    ("standalone-dual-encoder", (
        "-DAPP_S2_ONLY=1", "-DS2_ENCODER_ENABLED=1")),
    ("standalone-dual-encoder-swapped", (
        "-DAPP_S2_ONLY=1", "-DS2_ENCODER_ENABLED=1",
        "-DBOARD_MOTOR1_ENCODER_A_GPIO=4U",
        "-DBOARD_MOTOR1_ENCODER_B_GPIO=5U",
        "-DBOARD_MOTOR2_ENCODER_A_GPIO=16U",
        "-DBOARD_MOTOR2_ENCODER_B_GPIO=17U")),
    ("standalone-dual-encoder-shared-bank", (
        "-DAPP_S2_ONLY=1", "-DS2_ENCODER_ENABLED=1",
        "-DBOARD_MOTOR1_ENCODER_A_GPIO=2U",
        "-DBOARD_MOTOR1_ENCODER_B_GPIO=3U")),
    ("standalone-dual-encoder-cross-bank", (
        "-DAPP_S2_ONLY=1", "-DS2_ENCODER_ENABLED=1",
        "-DBOARD_MOTOR1_ENCODER_A_GPIO=7U",
        "-DBOARD_MOTOR1_ENCODER_B_GPIO=12U",
        "-DBOARD_MOTOR2_ENCODER_A_GPIO=14U",
        "-DBOARD_MOTOR2_ENCODER_B_GPIO=16U")),
    ("integrated-no-encoder", ("-DAPP_S2_ONLY=0",)),
    ("integrated-dual-encoder", (
        "-DAPP_S2_ONLY=0", "-DS2_ENCODER_ENABLED=1")),
    ("integrated-custom-timeout", (
        "-DAPP_S2_ONLY=0", "-DS2_ENCODER_ENABLED=1",
        "-DS2_COMMAND_TIMEOUT_MS=750U")),
    # Robot speed PID and trims; spin/pivot ceiling, kick and stall defaults.
    ("integrated-pid-turn", (
        "-DAPP_S2_ONLY=0", "-DS2_ENCODER_ENABLED=1", "-DBOARD_PID_ENABLED=1",
        "-DBOARD_PID_MAX_CPS=1950", "-DBOARD_PID_KP_MILLI=30",
        "-DBOARD_PID_KI_MILLI=40", "-DBOARD_PID_KD_MILLI=0",
        "-DBOARD_PID_MAX_POWER=35", "-DBOARD_PID_TURN_MAX_POWER=55",
        "-DBOARD_TURN_KICK_MS=150U", "-DBOARD_TURN_KICK_POWER=45",
        "-DBOARD_TURN_STALL_MS=1200U", "-DBOARD_TURN_KICK_COUNTS=5", "-DBOARD_TURN_REKICK_MS=100U",
        "-DS2_MOTOR1_TRIM_PERCENT=86U", "-DS2_MOTOR2_TRIM_PERCENT=93U")),
)


def run_tests(compiler: str) -> None:
    """Run every profile; propagate a failed compiler/test exit to the caller."""
    compiler_path = shutil.which(compiler)
    if compiler_path is None:
        raise RuntimeError(
            f"C compiler not found: {compiler!r}. Install native GCC or Clang "
            "and put it on PATH, or supply --compiler with its executable path."
        )
    compiler_path = str(Path(compiler_path).resolve())

    print(f"Buddy 2 host compiler: {compiler_path}", flush=True)
    # All fixture and production copies belong to this temporary directory.
    # Tests never write to the firmware build or an existing user's build tree.
    with tempfile.TemporaryDirectory(prefix="s2-host-") as directory:
        temporary = Path(directory)
        subsystem = temporary / "subsystem2"
        shared = temporary / "shared"
        (temporary / "tk").mkdir()
        subsystem.mkdir()
        shared.mkdir()

        for filename in ("app.h", "board_pins.h", "subsystem2_host_test.c"):
            shutil.copy2(HERE / filename, temporary / filename)
        shutil.copy2(HERE / "tk/syslib.h", temporary / "tk/syslib.h")
        for filename in (
            "subsystem2.c", "subsystem2.h", "subsystem2_test.c",
            "s2_data.c", "s2_data.h", "speed_pid.h", "wheel_steering.h",
            "turn_drive.h",
        ):
            shutil.copy2(SUBSYSTEM / filename, subsystem / filename)
        for filename in ("edr.c", "edr.h", "encoder_snapshot.h"):
            shutil.copy2(SHARED / filename, shared / filename)
        # Exercise the integrated motor implementation with explicit test-only
        # drive permission; real firmware retains its default motor lock.
        (temporary / "robot_config.h").write_text(
            "#define BOARD_MOTORS_DISABLED 0\n", encoding="utf-8")

        for name, flags in PROFILES:
            executable = temporary / (name + (".exe" if os.name == "nt" else ""))
            command = [
                compiler_path, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-Wpedantic", "-DAPP_TEST=1", "-I", str(temporary),
                *flags,
                str(temporary / "subsystem2_host_test.c"),
                str(subsystem / "subsystem2_test.c"),
                str(subsystem / "s2_data.c"),
                str(shared / "edr.c"),
                "-o", str(executable),
            ]
            print(f"BUILD Buddy 2 {name}", flush=True)
            subprocess.run(command, cwd=temporary, check=True)
            print(f"RUN Buddy 2 {name}", flush=True)
            subprocess.run([str(executable)], cwd=temporary, check=True)
            print(f"PASS Buddy 2 {name}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--compiler", default=os.environ.get("CC", "gcc"),
        help="Native GCC/Clang executable (default: CC environment variable or gcc).",
    )
    arguments = parser.parse_args()
    try:
        run_tests(arguments.compiler)
    except subprocess.CalledProcessError as error:
        print(f"ERROR: Buddy 2 build/test failed: {error}", file=sys.stderr)
        return error.returncode if error.returncode > 0 else 1
    except (OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
