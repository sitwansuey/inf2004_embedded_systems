#!/usr/bin/env python3
"""Decode version 1 EDR MQTT bytes for dashboard consumers (no dependencies).

Example: python tools/edr_decode.py 4207260b7fffffffff
The envelope is shared by all five buddies. Buddy 2 kinds 0 and 1 and
Buddy 5 kind 0 have payload schemas here; other payloads remain lossless hex.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import sys


VERSION = 1
UINT32_MAX = 0xFFFFFFFF

S5_ANGLES = "angle_deg sample_angle_deg avoidance_deg planned_rotate_deg left_bound_deg right_bound_deg".split()
S5_FLAGS = "acquisition_clear approach_plan_valid encoder_valid valid left_bound_valid right_bound_valid object_width_valid".split()
S5_WORDS = ("scan_id planned_rotate_ms planned_approach_mm planned_return_mm "
            "obstacle_distance_mm distance_mm echo_us time_ms motor1_counts_start "
            "motor2_counts_start motor1_counts_end motor2_counts_end encoder_start_ms "
            "encoder_end_ms left_bound_distance_mm right_bound_distance_mm "
            "object_width_mm object_width_min_mm object_width_max_mm").split()


def decode_obstacle(data: bytes) -> dict:
    """Decode the exact S5 snapshot; validity and freshness remain consumer checks."""
    if len(data) != 84:
        raise ValueError("obstacle payload must contain exactly 84 bytes")
    state, echo = data[0] & 15, (data[0] >> 4) & 7
    if data[0] & 128 or data[1] & 128 or state > 14 or echo > 4:
        raise ValueError("invalid obstacle control bits")
    if any(value > 180 for value in data[2:8]):
        raise ValueError("invalid obstacle angle")
    result = {"state": state, "echo_result": echo}
    result.update((name, bool(data[1] & (1 << bit))) for bit, name in enumerate(S5_FLAGS))
    result.update((name, value - 90) for name, value in zip(S5_ANGLES, data[2:8]))
    for name, value in zip(S5_WORDS, struct.unpack("<19I", data[8:])):
        if name.startswith("motor") and value >= 0x80000000:
            value -= 0x100000000
        result[name] = value
    return result


def _integer(name: str, value: int, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in {minimum}..{maximum}")
    return value


def pack_motor(command: int, power_percent: int, state: int) -> int:
    """Build the 12-bit word; reject values that would truncate or overlap."""
    return (_integer("command", command, 0, 6)
            | (_integer("power_percent", power_percent, 0, 100) << 3)
            | (_integer("state", state, 0, 2) << 10))


def unpack_motor(word: int) -> dict:
    _integer("motor_word", word, 0, 0xFFFF)
    command = word & 7
    power = (word >> 3) & 127
    state = (word >> 10) & 3
    if word >> 12 or command > 6 or power > 100 or state > 2:
        raise ValueError("invalid motor word (reserved bits or field range)")
    return {"motor_word": word, "command": command,
            "power_percent": power, "state": state}


def encode_packet(source: int, kind: int, payload: bytes) -> bytes:
    """Frame a buddy-owned payload; semantic validation belongs to its codec."""
    _integer("source", source, 1, 5)
    _integer("kind", kind, 0, 7)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("payload must be bytes-like")
    payload = bytes(payload)
    _integer("payload length", len(payload), 0, 255)
    return bytes([(VERSION << 6) | (kind << 3) | source, len(payload)]) + payload


def encode_motion(command: int, power_percent: int, state: int,
                  faults: int, command_ms: int) -> bytes:
    word = pack_motor(command, power_percent, state)
    _integer("faults", faults, 0, 0x7F)
    _integer("command_ms", command_ms, 0, UINT32_MAX)
    return encode_packet(2, 0, struct.pack("<HBI", word, faults, command_ms))


def encode_encoder(left_counts: int, right_counts: int,
                   sampled_ms: int, flags: int) -> bytes:
    _integer("left_counts", left_counts, 0, UINT32_MAX)
    _integer("right_counts", right_counts, 0, UINT32_MAX)
    _integer("sampled_ms", sampled_ms, 0, UINT32_MAX)
    _integer("flags", flags, 0, 0x1F)
    if not flags & 1 and (flags or left_counts or right_counts or sampled_ms):
        raise ValueError("invalid encoder must use canonical all-zero payload")
    return encode_packet(2, 1, struct.pack("<IIIB", left_counts, right_counts,
                                         sampled_ms, flags))


def decode_packet(packet: bytes) -> dict:
    """Validate one complete MQTT application payload and return JSON data.

    Raises ValueError for an invalid envelope or a malformed known payload.
    Unknown buddy/kind payloads are returned with supported=False and raw hex.
    Counts and time are unsigned modulo-32 values, never signed coordinates.
    """
    if not isinstance(packet, (bytes, bytearray, memoryview)):
        raise TypeError("packet must be bytes-like")
    packet = bytes(packet)
    if len(packet) < 2:
        raise ValueError("packet must contain the two-byte header")
    header, payload_size = packet[:2]
    version, source, kind = header >> 6, header & 7, (header >> 3) & 7
    if version != VERSION:
        raise ValueError("unsupported EDR version")
    if not 1 <= source <= 5:
        raise ValueError("invalid buddy source")
    if len(packet) != payload_size + 2:
        raise ValueError("packet length does not equal header length")
    payload_bytes = packet[2:]
    result = {"version": version, "source": source, "kind": kind,
              "payload_size": payload_size, "supported": False,
              "record_type": "raw", "payload": {"hex": payload_bytes.hex()}}
    if source == 2 and kind == 0:
        if payload_size != 7:
            raise ValueError("motion payload must contain exactly seven bytes")
        word, faults, command_ms = struct.unpack("<HBI", payload_bytes)
        payload = unpack_motor(word)
        if faults & ~0x7F:
            raise ValueError("reserved motion fault bit is set")
        payload.update(faults=faults, command_ms=command_ms)
        result.update(supported=True, record_type="motion", payload=payload)
    elif source == 2 and kind == 1:
        if payload_size != 13:
            raise ValueError("encoder payload must contain exactly thirteen bytes")
        left, right, sampled_ms, flags = struct.unpack("<IIIB", payload_bytes)
        if flags & ~0x1F:
            raise ValueError("reserved encoder flag bit is set")
        if not flags & 1 and any(payload_bytes):
            raise ValueError("invalid encoder must use canonical all-zero payload")
        payload = {"left_counts": left, "right_counts": right,
                   "sampled_ms": sampled_ms, "flags": flags,
                   "valid": bool(flags & 1)}
        result.update(supported=True, record_type="encoder", payload=payload)
    elif source == 5 and kind == 0:
        result.update(supported=True, record_type="obstacle", payload=decode_obstacle(payload_bytes))
    return result


def count_delta(current: int, previous: int) -> int:
    """Signed modulo-32 tick delta; an ambiguous half-range raises ValueError."""
    _integer("current", current, 0, UINT32_MAX)
    _integer("previous", previous, 0, UINT32_MAX)
    difference = (current - previous) & UINT32_MAX
    if difference == 0x80000000:
        raise ValueError("ambiguous half-range count delta")
    return difference if difference < 0x80000000 else difference - 0x100000000


def age_is_fresh(now_ms: int, sampled_ms: int, maximum_ms: int) -> bool:
    _integer("now_ms", now_ms, 0, UINT32_MAX)
    _integer("sampled_ms", sampled_ms, 0, UINT32_MAX)
    _integer("maximum_ms", maximum_ms, 0, UINT32_MAX)
    return maximum_ms < 0x80000000 and ((now_ms - sampled_ms) & UINT32_MAX) <= maximum_ms


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("hex_packet", nargs="?", help="one packet as hexadecimal")
    inputs.add_argument("--file", type=Path, help="read one binary packet")
    arguments = parser.parse_args()
    try:
        packet = (arguments.file.read_bytes() if arguments.file is not None
                  else bytes.fromhex(arguments.hex_packet))
        print(json.dumps(decode_packet(packet), indent=2))
    except (ValueError, OSError, TypeError) as error:
        print(f"edr_decode: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
