"""Compile the real S5 codec, storage API and policy with hardware-only stubs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


def decode_internal_obstacle(packet):
    """Independent reader for the C fixture's internal snapshot envelope.

    This is not an MQTT decoder: the dashboard only accepts two-byte EDR16
    readings. The full-width S5 status codec remains a local firmware API.
    """
    if len(packet) != 86 or packet[:2] != bytes((0x45, 84)):
        raise ValueError("invalid internal obstacle envelope")
    data = packet[2:]
    if (data[0] & 128 or data[1] & 128 or (data[0] & 15) > 14
            or ((data[0] >> 4) & 7) > 4 or any(angle > 180 for angle in data[2:8])):
        raise ValueError("invalid internal obstacle fields")
    return {"record_type": "obstacle", "payload": {
        "scan_id": int.from_bytes(data[8:12], "little"),
        "angle_deg": data[2] - 90,
        "motor1_counts_start": int.from_bytes(data[40:44], "little", signed=True),
        "time_ms": int.from_bytes(data[36:40], "little"),
    }}


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
                record = decode_internal_obstacle(bytes.fromhex(result.stdout.splitlines()[0]))
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
                        decode_internal_obstacle(bad)


if __name__ == "__main__":
    unittest.main()
