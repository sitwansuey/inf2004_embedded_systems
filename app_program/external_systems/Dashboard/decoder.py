"""Decode CSC02 CR1 telemetry. No MQTT dependency is needed for offline use."""
import argparse
import json
from pathlib import Path

SCHEMA = json.loads((Path(__file__).resolve().parents[3] / "tools" / "telemetry_schema.json").read_text())
BITS = sum(f["bits"] for f in SCHEMA)
SIZE = (BITS + 7) // 8

def decode(payload):
    if len(payload) != SIZE:
        raise ValueError(f"Expected {SIZE} bytes, got {len(payload)}")
    packed = int.from_bytes(payload, "little")
    if packed >> BITS:
        raise ValueError("Nonzero reserved padding")
    result = {}
    offset = 0
    for field in SCHEMA:
        value = ((packed >> offset) & ((1 << field["bits"]) - 1)) + field["min"]
        if value > field["max"]:
            raise ValueError(f"Invalid {field['name']}")
        result[field["name"]] = value
        offset += field["bits"]
    return result

def encode(values):
    """Independent integer implementation for cross-language fixtures."""
    packed = offset = 0
    for field in SCHEMA:
        value = values[field["name"]]
        if not isinstance(value, int) or not field["min"] <= value <= field["max"]:
            raise ValueError(field["name"])
        packed |= (value - field["min"]) << offset
        offset += field["bits"]
    return packed.to_bytes(SIZE, "little")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex")
    args = parser.parse_args()
    print(json.dumps(decode(bytes.fromhex(args.hex)), indent=2))
