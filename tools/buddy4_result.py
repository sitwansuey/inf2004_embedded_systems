"""Independent Buddy 4 result decoder. Raw payload 12 bytes; EDR frame 14."""
import argparse
import json

# name, bits, minimum, maximum. Little-endian stream, LSB-first per byte.
FIELDS = (
    ("calculated_ms", 32, 0, 0xffffffff),
    ("pitch_cdeg", 15, -9000, 9000),
    ("roll_cdeg", 16, -18000, 18000),
    ("height_mm", 10, 0, 1000),
    ("maximum_mm", 10, 0, 1000),
    ("terrain", 3, 0, 5),
    ("flags", 5, 0, 31),
    ("fault_code", 2, 0, 3),
)

def decode(payload):
    if len(payload) != 12:
        raise ValueError("Buddy 4 payload must be exactly 12 bytes")
    word = int.from_bytes(payload, "little")
    if word >> 93:
        raise ValueError("Reserved padding")
    result = {}
    offset = 0
    for name, bits, minimum, maximum in FIELDS:
        value = ((word >> offset) & ((1 << bits)-1)) + minimum
        if value > maximum:
            raise ValueError(name)
        result[name] = value
        offset += bits
    flags = result["flags"]
    if (result["fault_code"] & 2 and flags & 1) or (result["fault_code"] and flags & 2):
        raise ValueError("Fault contradicts measurement validity")
    if flags & 2 and (flags & 25 != 25 or result["height_mm"] == 0 or
                      result["maximum_mm"] < result["height_mm"]):
        raise ValueError("Invalid completed height")
    if flags & 1 and (not flags & 8 or flags & 4):
        raise ValueError("Invalid tilt")
    result["faults"] = result.pop("fault_code") << 1
    return result

def decode_frame(packet):
    if len(packet) != 14 or packet[:2] != bytes((0x54, 12)):
        raise ValueError("Expected EDR v1 source 4/kind 2, 12-byte payload")
    return decode(packet[2:])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex")
    parser.add_argument("--framed", action="store_true")
    args = parser.parse_args()
    print(json.dumps((decode_frame if args.framed else decode)(bytes.fromhex(args.hex)), indent=2))
