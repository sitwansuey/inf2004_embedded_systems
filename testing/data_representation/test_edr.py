"""Independent, standard-library verification of the dashboard wire contract."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("edr_decode", ROOT / "tools/edr_decode.py")
EDR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EDR)
VECTORS = json.loads(Path(__file__).with_name("vectors.json").read_text())


class EdrTests(unittest.TestCase):
    def test_shared_known_packets(self):
        for vector in VECTORS["valid_packets"]:
            with self.subTest(vector=vector["name"]):
                packet = bytes.fromhex(vector["hex"])
                decoded = EDR.decode_packet(packet)
                self.assertEqual(decoded["source"], vector["source"])
                self.assertEqual(decoded["kind"], vector["kind"])
                if "motion" in vector:
                    fields = vector["motion"]
                    self.assertEqual(decoded["payload"], fields)
                    encoded = EDR.encode_motion(fields["command"], fields["power_percent"],
                                                fields["state"], fields["faults"],
                                                fields["command_ms"])
                elif "encoder" in vector:
                    fields = vector["encoder"]
                    self.assertEqual(decoded["payload"], dict(fields, valid=bool(fields["flags"] & 1)))
                    encoded = EDR.encode_encoder(**fields)
                else:
                    self.assertFalse(decoded["supported"])
                    self.assertEqual(decoded["payload"]["hex"], packet[2:].hex())
                    encoded = EDR.encode_packet(vector["source"], vector["kind"], packet[2:])
                self.assertEqual(encoded, packet)
                json.dumps(decoded)  # Public output must be JSON-compatible.

    def test_shared_malformed_packets(self):
        for vector in VECTORS["invalid_packets"]:
            with self.subTest(vector=vector["name"]):
                with self.assertRaises(ValueError):
                    EDR.decode_packet(bytes.fromhex(vector["hex"]))

    def test_every_motor_word_and_every_valid_combination(self):
        valid_count = 0
        for word in range(65536):
            command = word % 8
            power = (word // 8) % 128
            state = (word // 1024) % 4
            expected_valid = word < 4096 and command < 7 and power < 101 and state < 3
            if expected_valid:
                unpacked = EDR.unpack_motor(word)
                self.assertEqual(unpacked, {"motor_word": word, "command": command,
                                            "power_percent": power, "state": state})
                self.assertEqual(EDR.pack_motor(command, power, state), word)
                valid_count += 1
            else:
                with self.assertRaises(ValueError, msg=f"word {word}"):
                    EDR.unpack_motor(word)
        self.assertEqual(valid_count, 7 * 101 * 3)
        for command in range(7):
            for power in range(101):
                for state in range(3):
                    self.assertEqual(EDR.pack_motor(command, power, state),
                                     command + 8 * power + 1024 * state)

    def test_all_header_bytes_and_lengths(self):
        for header in range(256):
            source, kind, version = header & 7, (header >> 3) & 7, header >> 6
            payload = bytes(7 if (source, kind) == (2, 0) else
                            13 if (source, kind) == (2, 1) else 0)
            packet = bytes([header, len(payload)]) + payload
            if version == 1 and 1 <= source <= 5:
                self.assertEqual(EDR.decode_packet(packet)["source"], source)
            else:
                with self.assertRaises(ValueError):
                    EDR.decode_packet(packet)
        for length in range(256):
            payload = bytes(range(length))
            packet = EDR.encode_packet(1, 7, payload)
            self.assertEqual(len(packet), length + 2)
            self.assertEqual(EDR.decode_packet(packet)["payload"]["hex"], payload.hex())
            with self.assertRaises(ValueError):
                EDR.decode_packet(packet + b"\x00")
            with self.assertRaises(ValueError):
                EDR.decode_packet(packet[:-1])

    def test_exact_known_payload_sizes_and_reserved_fields(self):
        for kind, expected_size in ((0, 7), (1, 13)):
            for size in range(256):
                packet = EDR.encode_packet(2, kind, bytes(size))
                if size == expected_size:
                    EDR.decode_packet(packet)
                else:
                    with self.assertRaises(ValueError):
                        EDR.decode_packet(packet)
        for flags in range(256):
            packet = bytes([0x4A, 13]) + bytes(12) + bytes([flags])
            if flags == 0 or (flags < 32 and flags & 1):
                EDR.decode_packet(packet)
            else:
                with self.assertRaises(ValueError):
                    EDR.decode_packet(packet)
        for faults in range(256):
            packet = bytes([0x42, 7, 0, 0, faults, 0, 0, 0, 0])
            if faults < 128:
                EDR.decode_packet(packet)
            else:
                with self.assertRaises(ValueError):
                    EDR.decode_packet(packet)

    def test_all_buddy_envelope_matrix(self):
        for source in range(1, 6):
            for kind in range(8):
                for size in range(256):
                    with self.subTest(source=source, kind=kind, size=size):
                        known_size = {0: 7, 1: 13}.get(kind) if source == 2 else None
                        # Known schemas use their canonical zero payload;
                        # opaque schemas exercise embedded NULs and byte values.
                        payload = (bytes(size) if known_size is not None else
                                   bytes(0 if index % 3 == 0 else
                                         (index * 37 + size + source + kind) % 256
                                         for index in range(size)))
                        packet = EDR.encode_packet(source, kind, payload)
                        self.assertEqual(packet, bytes([64 + kind * 8 + source, size])
                                         + payload)
                        if known_size is not None and size != known_size:
                            with self.assertRaises(ValueError):
                                EDR.decode_packet(packet)
                            continue
                        decoded = EDR.decode_packet(packet)
                        self.assertEqual(decoded["version"], 1)
                        self.assertEqual(decoded["source"], source)
                        self.assertEqual(decoded["kind"], kind)
                        self.assertEqual(decoded["payload_size"], size)
                        if known_size == 7:
                            self.assertTrue(decoded["supported"])
                            self.assertEqual(decoded["payload"], {
                                "motor_word": 0, "command": 0, "power_percent": 0,
                                "state": 0, "faults": 0, "command_ms": 0})
                        elif known_size == 13:
                            self.assertTrue(decoded["supported"])
                            self.assertEqual(decoded["payload"], {
                                "left_counts": 0, "right_counts": 0,
                                "sampled_ms": 0, "flags": 0, "valid": False})
                        else:
                            self.assertFalse(decoded["supported"])
                            self.assertEqual(decoded["payload"]["hex"], payload.hex())

    def test_delta_and_clock_wrap(self):
        for vector in VECTORS["count_deltas"]:
            if vector["delta"] is None:
                with self.assertRaises(ValueError):
                    EDR.count_delta(vector["current"], vector["previous"])
            else:
                self.assertEqual(EDR.count_delta(vector["current"], vector["previous"]),
                                 vector["delta"])
        for vector in VECTORS["freshness"]:
            self.assertEqual(EDR.age_is_fresh(vector["now_ms"], vector["sampled_ms"],
                                              vector["maximum_ms"]), vector["fresh"])

    def test_encoder_canonical_unavailable_and_input_ranges(self):
        for args in ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 2),
                     (0, 0, 0, 32), (-1, 0, 0, 1), (2**32, 0, 0, 1)):
            with self.assertRaises(ValueError):
                EDR.encode_encoder(*args)
        for args in ((7, 0, 0), (0, 101, 0), (0, 0, 3), (-1, 0, 0), (True, 0, 0)):
            with self.assertRaises(ValueError):
                EDR.pack_motor(*args)
        for source, kind, payload in ((0, 0, b""), (6, 0, b""), (1, 8, b""),
                                      (1, 0, bytes(256))):
            with self.assertRaises(ValueError):
                EDR.encode_packet(source, kind, payload)
        with self.assertRaises(TypeError):
            EDR.decode_packet("420700000000000000")

    def test_cli_json_and_failure_status(self):
        command = [sys.executable, str(ROOT / "tools/edr_decode.py")]
        result = subprocess.run(command + [VECTORS["valid_packets"][1]["hex"]],
                                capture_output=True, text=True, check=True)
        self.assertEqual(json.loads(result.stdout)["payload"]["command_ms"], 0xFFFFFFFF)
        result = subprocess.run(command + ["4200"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("edr_decode:", result.stderr)
        with tempfile.TemporaryDirectory(prefix="edr-cli-") as directory:
            binary_packet = Path(directory) / "packet.bin"
            binary_packet.write_bytes(bytes.fromhex(VECTORS["valid_packets"][1]["hex"]))
            result = subprocess.run(command + ["--file", str(binary_packet)],
                                    capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(result.stdout)["payload"]["motor_word"], 2854)


if __name__ == "__main__":
    unittest.main()
