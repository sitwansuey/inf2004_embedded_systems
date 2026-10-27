"""Compile the real S5 codec, storage API and policy with hardware-only stubs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from edr_decode import decode_packet


class Subsystem5Tests(unittest.TestCase):
    def test_codec_and_status_api(self):
        compiler = shutil.which(os.environ.get("CC", "gcc"))
        self.assertIsNotNone(compiler, "A native C compiler is required")
        with tempfile.TemporaryDirectory(prefix="s5-data-") as temp:
            app = Path(temp) / "app_program"
            shutil.copytree(ROOT / "app_program/subsystem5", app / "subsystem5")
            shutil.copytree(ROOT / "app_program/shared", app / "shared")
            shutil.copyfile(ROOT / "testing/subsystem5/host_app.h", app / "app.h")
            shutil.copyfile(ROOT / "testing/subsystem5/test_data.c", app / "subsystem5/test_data.c")
            folder = app / "subsystem5"
            exe = Path(temp) / "test.exe"
            for test_mode in (0, 1):
                subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                "-Wno-unused-function", f"-DAPP_TEST={test_mode}",
                                str(folder / "test_data.c"), str(folder / "s5_data.c"),
                                str(folder / "s5_policy.c"), str(folder / "s5_scan.c"),
                                str(folder / "s5_width.c"), str(app / "shared/edr.c"),
                                "-o", str(exe)], check=True, capture_output=True, text=True)
                result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
                print(result.stdout.strip())
                record = decode_packet(bytes.fromhex(result.stdout.splitlines()[0]))
                self.assertEqual(record["record_type"], "obstacle")
                self.assertEqual(record["payload"]["scan_id"], 0x12345678)
                self.assertEqual(record["payload"]["angle_deg"], -90)
                self.assertEqual(record["payload"]["motor1_counts_start"], -2147483648)
                self.assertEqual(record["payload"]["time_ms"], 0xffffffff)
                packet = bytearray.fromhex(result.stdout.splitlines()[0])
                for offset, invalid in ((2, 15), (2, 0x50), (2, 128), (3, 128), (4, 181)):
                    bad = bytearray(packet)
                    bad[offset] = invalid
                    with self.assertRaises(ValueError):
                        decode_packet(bad)


if __name__ == "__main__":
    unittest.main()
