"""Canonical EDR16 codec and CLI. Example: python tools/edr_decode.py '41 06'."""
import argparse
import json

MQTT_TOPIC = "Robopico/sensors/edr16/v1"
NAMES = {0:"mission_status", 1:"encoder_left", 2:"encoder_right", 3:"servo_angle_deg",
         4:"distance_cm", 5:"ir_left", 6:"ir_right", 7:"ir_barcode",
         8:"mag_x_ut", 9:"mag_y_ut", 10:"mag_z_ut", 11:"motor_status",
         12:"motor_faults", 13:"accel_x_mps2", 14:"accel_y_mps2", 15:"accel_z_mps2"}
# ID 12 holds the shared fault bits (app_program/shared/edr.h): the motor's
# timeout 0x20 / error 0x40 and the IMU fault 0x04.
IMU_FAULT_BIT = 0x04

def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"expected integer in {low}..{high}")
    return value

def pack_motor(command, power_percent, state):
    return (integer(command,0,6) | (integer(power_percent,0,100) << 3)
            | (integer(state,0,2) << 10))

def unpack_motor(data):
    integer(data,0,4095)
    command, power, state = data & 7, (data >> 3) & 127, data >> 10
    integer(command,0,6); integer(power,0,100); integer(state,0,2)
    return {"command":command,"power_percent":power,"state":state}

def status_valid(data):
    subtype, value = data >> 8, data & 255
    if subtype == 0: return (value & 15) <= 13 and (value >> 4) <= 8  # phase | fault << 4 (0..8, 8 = lifted)
    if subtype == 1: return (value & 31) <= 26 and (value >> 5) <= 4
    # subtype 2: follower | black mask << 3. Follower this tick: 0 straight
    # (or not following), 1/2 steer right/left, 3/4 pivot right/left; mask:
    # straddle IRs on black, 1 right, 2 left.
    if subtype == 2: return value <= 63 and (value & 7) <= 4
    if subtype in (3, 4): return 38 <= value <= 218
    if subtype == 5: return (value & 7) <= 5  # terrain | barcode bars << 3
    if subtype in (6, 7): return value <= 200
    if subtype == 8: return value <= 100 or value >= 128  # total 5 mm | 128+left / 192+right 8 mm
    # subtype 9: quality bits 0..5 | fault cause << 6. Fault 1 (sensor):
    # 0 other, 1 sonar, 2 tilt, 3 odometry. Fault 2 (line): 0 other, 1 line
    # data stale (gap >= 250 ms or no TRACK handoff), 2 S3 fault event, 3 line
    # not found (REACQUIRE). Any other fault: 0.
    # Subtypes 11 (sequence) and 12 (internal drops) are counters modulo 256
    # that wrap: use counter_delta between records, never the absolute value.
    if subtype in (9, 10, 11, 12): return True
    if subtype in (13, 14): return 28 <= value <= 228
    # scan | barcode result << 4 | bypass side << 6 (0 none, 1 left, 2 right)
    return subtype == 15 and (value & 15) <= 14 and (value >> 6) <= 2

def counter_delta(previous, current):
    """Counts between two values of a wrapping status counter (subtype 11 or
    12, modulo 256). More than 255 between two records alias."""
    return (integer(current,0,255) - integer(previous,0,255)) & 255

def validate(sensor_id, data):
    integer(sensor_id,0,15); integer(data,0,4095)
    if sensor_id == 0 and not status_valid(data): raise ValueError("invalid mission status")
    if sensor_id == 3: integer(data,0,180)
    if sensor_id == 11: unpack_motor(data)
    if sensor_id == 12: integer(data,0,127)

def encode_packet(sensor_id, data):
    validate(sensor_id, data)
    return ((data << 4) | sensor_id).to_bytes(2,"little")

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
    if sensor_id == 0: result.update(subtype=data >> 8, status_value=data & 255)
    if sensor_id == 11: result["motor"] = unpack_motor(data)
    return result

# Robot events (app_program/shared/events.h): one EDR16-format record each on
# their own topic. Low nibble = event ID 1..5, high 12 bits = value.
EVENT_TOPIC = "Robopico/events/edr16/v1"
EVENT_NAMES = {1:"barcode", 2:"instruction", 3:"hump", 4:"obstacle", 5:"link"}

def decode_event(packet):
    """Split one robot event record into its fields; ValueError if invalid."""
    if not isinstance(packet,(bytes,bytearray,memoryview)) or len(packet) != 2:
        raise ValueError("an event record is exactly two bytes")
    word = int.from_bytes(packet,"little")
    event_id, value = word & 15, word >> 4
    if event_id == 1:
        fields = {"attempt":value & 3, "elements":(value >> 2) & 31, "letter":value >> 7}
        ok = 1 <= fields["attempt"] <= 3 and fields["letter"] <= 26
    elif event_id == 2:
        fields = {"letter":value & 31, "turn":(value >> 5) & 7}
        ok = 1 <= fields["letter"] <= 26 and fields["turn"] <= 4 and value >> 8 == 0
    elif event_id == 3:
        fields = {"kind":value & 7, "value":value >> 3}
        ok = 1 <= fields["kind"] <= 3
    elif event_id == 4:
        fields = {"kind":value & 3, "side":(value >> 2) & 3, "value":value >> 4}
        # kind 0 "how": 0 ended by a fault/STOP, 1..3 back on the line;
        # kind 2 side 0: no way round, scanning again (value = retry 1..3)
        ok = (fields["side"] <= 3 if fields["kind"] == 0 else
              fields["side"] == 0 if fields["kind"] == 1 else
              (1 <= fields["side"] <= 2 or (fields["side"] == 0 and 1 <= fields["value"] <= 3))
              if fields["kind"] == 2 else 1 <= fields["side"] <= 2)
    elif event_id == 5:
        fields = {"kind":value & 3, "seconds":value >> 2}
        ok = 1 <= fields["kind"] <= 2
    else:
        raise ValueError(f"unknown event id {event_id}")
    if not ok: raise ValueError(f"invalid {EVENT_NAMES[event_id]} event")
    return {"id":event_id, "name":EVENT_NAMES[event_id], **fields}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex",help="exactly two bytes, little-endian")
    args=parser.parse_args()
    try: print(json.dumps(decode_packet(bytes.fromhex(args.hex))))
    except (ValueError,TypeError) as error: parser.error(str(error))
if __name__ == "__main__": main()
