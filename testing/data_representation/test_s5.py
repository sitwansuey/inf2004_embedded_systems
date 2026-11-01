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
            shutil.copyfile(ROOT / "app_program/robot_config.h", app / "robot_config.h")
            shutil.copyfile(ROOT / "testing/subsystem5/test_data.c", app / "subsystem5/test_data.c")
            folder = app / "subsystem5"
            exe = Path(temp) / "test.exe"
            for test_mode in (0, 1):
                subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                "-Wno-unused-function", f"-DAPP_TEST={test_mode}",
                                str(folder / "test_data.c"), str(folder / "s5_data.c"),
                                str(folder / "s5_policy.c"), str(folder / "s5_scan.c"),
                                str(folder / "s5_width.c"), str(app / "shared/edr.c"),
                                "-o", str(exe)], check=True)
                result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
                print(result.stdout.strip())
                # This is a private full-resolution snapshot, NOT MQTT EDR16.
                raw=bytes.fromhex(result.stdout.splitlines()[0])
                self.assertEqual(len(raw),84)
                self.assertEqual(int.from_bytes(raw[8:12],"little"),0x12345678)
                self.assertEqual(raw[2]-90,-90)
                self.assertEqual(int.from_bytes(raw[40:44],"little",signed=True),-2147483648)
                self.assertEqual(int.from_bytes(raw[36:40],"little"),0xffffffff)



if __name__ == "__main__":
    unittest.main()
