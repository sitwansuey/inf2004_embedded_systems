"""Canonical EDR16 codec and CLI. Example: python tools/edr_decode.py '41 06'."""
import argparse
import json

MQTT_TOPIC = "Robopico/sensors/edr16/v2"
NAMES = {0:"mission_status", 1:"encoder_left", 2:"encoder_right", 3:"servo_angle_deg",
         4:"distance_cm", 5:"ir_left", 6:"ir_right", 7:"ir_barcode",
         8:"mag_x_ut", 9:"mag_y_ut", 10:"mag_z_ut", 11:"motor_status",
         12:"motor_faults", 13:"accel_x_mps2", 14:"accel_y_mps2", 15:"accel_z_mps2"}

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
    if subtype == 0: return (value & 15) <= 12 and (value >> 4) <= 7
    if subtype == 1: return (value & 31) <= 26 and (value >> 5) <= 4
    if subtype == 2: return value <= 63 and (value & 7) <= 4
    if subtype in (3, 4): return 38 <= value <= 218
    if subtype == 5: return value <= 5
    if subtype in (6, 7): return value <= 200
    if subtype == 8: return value <= 100
    if subtype == 9: return value <= 63
    if subtype in (10, 11, 12): return True
    if subtype in (13, 14): return 28 <= value <= 228
    return subtype == 15 and value <= 14

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

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex",help="exactly two bytes, little-endian")
    args=parser.parse_args()
    try: print(json.dumps(decode_packet(bytes.fromhex(args.hex))))
    except (ValueError,TypeError) as error: parser.error(str(error))
if __name__ == "__main__": main()
