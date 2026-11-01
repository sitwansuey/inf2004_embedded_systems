"""Dashboard field adapter; the sole Python wire codec is tools/edr_decode.py."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from edr_decode import decode_packet as decode_record, MQTT_TOPIC
PACKED_TOPIC = MQTT_TOPIC
READINGS = {
    1: "Robopico/sensors/left_encoder/encoder_left",
    2: "Robopico/sensors/right_encoder/encoder_right",
    3: "Robopico/Actuators/servo_motor/servo_angle_deg",
    4: "Robopico/sensors/ultrasonic_ranger/distance_cm",
    5: "Robopico/sensors/left_ir/ir_left",
    6: "Robopico/sensors/right_ir/ir_right",
    7: "Robopico/sensors/barcode_ir/ir_barcode",
    8: "Robopico/sensors/magnetometer/mag_x_ut",
    9: "Robopico/sensors/magnetometer/mag_y_ut",
    10: "Robopico/sensors/magnetometer/mag_z_ut",
    12: "Robopico/Actuators/motors/fault_flags",
    13: "Robopico/sensors/accelerometer/accel_x_mps2",
    14: "Robopico/sensors/accelerometer/accel_y_mps2",
    15: "Robopico/sensors/accelerometer/accel_z_mps2",
}
PHASES = ("Disarmed", "Line tracking", "Acquisition", "Turn out", "Approach", "Align",
          "Pass obstacle", "Turn in", "Return", "Restore heading", "Line reacquisition", "Navigation turn", "Fault")
TERRAIN = ("Flat", "Ascent", "Crest", "Descent", "Shock", "Invalid")
def status_messages(subtype, value):
    sensor="Robopico/sensors/"
    actuator="Robopico/Actuators/"
    def one(section, field, v, motor=False):
        return [((actuator if motor else sensor)+section+"/"+field, v)]
    if subtype==0:
        phase=value&15
        return one("vehicle_controller","state",PHASES[phase] if phase<len(PHASES) else "Unknown",True)+one("vehicle_controller","fault_code",value>>4,True)
    if subtype==1:
        return one("barcode_ir","barcode",("—","A","B","C","D","Z")[value&7] if (value&7)<6 else "Unknown")+one("barcode_ir","navigation",("None","Left","Right","Straight","U-turn")[value>>3] if value>>3<5 else "Unknown")
    if subtype==2: return one("left_ir","line_state",value&7)+one("left_ir","black_mask",value>>3)
    if subtype==3: return one("magnetometer","pitch_deg",value-128)
    if subtype==4: return one("magnetometer","roll_deg",(value-128)*2)
    if subtype==5: return one("terrain","hump_state",TERRAIN[value] if value<len(TERRAIN) else "Unknown")
    if subtype in (6,7): return one("terrain","hump_height_cm" if subtype==6 else "run_max_height_cm",value/2)
    if subtype==8: return one("ultrasonic_ranger","width_cm",value/2)
    if subtype==9: return one("vehicle_controller","quality",value,True)+one("vehicle_controller","safety_state","Armed" if value&4 else "Stopped",True)+one("accelerometer","impact",bool(value&2))
    if subtype==10: return one("vehicle_controller","available",value,True)+one("terrain","valid",bool(value&64))
    if subtype==11: return one("vehicle_controller","sequence",value,True)
    if subtype==12: return one("vehicle_controller","dropped",value,True)
    if subtype==13: return one("motors","speed_mps",(value-128)/50,True)
    if subtype==14: return one("left_ir","line_error",(value-128)*10)
    return one("servo_motor","scan_state",value,True)

def decode_messages(payload):
    """Unmarshal one little-endian EDR16 record into local display fields.

    This does not publish MQTT messages. Motor ID 11 expands into three fields.
    Exact size, ID, and value ranges are validated before returning any fields.
    """
    record = decode_record(payload)
    if record["id"] == 0:
        return status_messages(record["subtype"], record["status_value"])
    if record["id"] == 11:
        motor=record["motor"]
        directions=("STOP","FORWARD","REVERSE","LEFT","RIGHT","PIVOT_RIGHT","PIVOT_LEFT")
        states=("STOPPED","MOVING","FAULT")
        return [("Robopico/Actuators/motors/direction",directions[motor["command"]]),
                ("Robopico/Actuators/motors/state",states[motor["state"]]),
                ("Robopico/Actuators/motors/power_percent",str(motor["power_percent"]))]
    return [(READINGS[record["id"]], str(record["value"]))]

def decode_packet(payload):
    """Compatibility for single-field consumers; motor records have 3 fields."""
    messages=decode_messages(payload)
    if len(messages)!=1: raise ValueError("motor status needs decode_messages")
    return messages[0]
