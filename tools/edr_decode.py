<<<<<<< HEAD
"""Canonical EDR16 codec and CLI. Example: python tools/edr_decode.py '41 06'."""
=======
#!/usr/bin/env python3
"""Decode version 1 EDR MQTT bytes for dashboard consumers (no dependencies).

Example: python tools/edr_decode.py 4207260b7fffffffff
The envelope is shared by all five buddies. Buddy 2 kinds 0 and 1 and
Buddy 5 kind 0 have payload schemas here; other payloads remain lossless hex.
"""

from __future__ import annotations

>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
import argparse
import json

MQTT_TOPIC = "Robopico/sensors/edr16/v1"
NAMES = {1:"encoder_left", 2:"encoder_right", 3:"servo_angle_deg",
         4:"distance_cm", 5:"ir_left", 6:"ir_right", 7:"ir_barcode",
         8:"mag_x_ut", 9:"mag_y_ut", 10:"mag_z_ut", 11:"motor_status",
         12:"motor_faults", 13:"accel_x_mps2", 14:"accel_y_mps2", 15:"accel_z_mps2"}

<<<<<<< HEAD
def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"expected integer in {low}..{high}")
=======
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
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
    return value

def pack_motor(command, power_percent, state):
    return (integer(command,0,6) | (integer(power_percent,0,100) << 3)
            | (integer(state,0,2) << 10))

def unpack_motor(data):
    integer(data,0,4095)
    command, power, state = data & 7, (data >> 3) & 127, data >> 10
    integer(command,0,6); integer(power,0,100); integer(state,0,2)
    return {"command":command,"power_percent":power,"state":state}

def validate(sensor_id, data):
    integer(sensor_id,1,15); integer(data,0,4095)
    if sensor_id == 3: integer(data,0,180)
    if sensor_id == 11: unpack_motor(data)
    if sensor_id == 12: integer(data,0,127)

def encode_packet(sensor_id, data):
    validate(sensor_id, data)
    return ((data << 4) | sensor_id).to_bytes(2,"little")

<<<<<<< HEAD
def decode_packet(packet):
    if not isinstance(packet,(bytes,bytearray,memoryview)) or len(packet) != 2:
        raise ValueError("EDR16 requires exactly two bytes")
    word = int.from_bytes(packet,"little")
    sensor_id, data = word & 15, word >> 4
    validate(sensor_id,data)
    value = data
    if 8 <= sensor_id <= 10: value = (data-2048)/10
    if 13 <= sensor_id <= 15: value = (data-2048)*9.80665/1000
    result = {"id":sensor_id,"name":NAMES[sensor_id],"data":data,"value":value}
    if sensor_id == 11: result["motor"] = unpack_motor(data)
=======

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
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex",help="exactly two bytes, little-endian")
    args=parser.parse_args()
    try: print(json.dumps(decode_packet(bytes.fromhex(args.hex))))
    except (ValueError,TypeError) as error: parser.error(str(error))
if __name__ == "__main__": main()
