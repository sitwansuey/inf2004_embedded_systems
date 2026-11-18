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
          "Pass obstacle", "Turn in", "Return", "Restore heading", "Line reacquisition", "Navigation turn", "Fault", "Reverse")
TERRAIN = ("Flat", "Ascent", "Crest", "Descent", "Shock", "Invalid")
BARCODE_RESULTS = ("No barcode yet", "Reading...", "Successful", "Unsuccessful")
BYPASS_SIDES = ("None", "Left", "Right")
FAULT_CAUSES = ("None/other", "Sonar", "Tilt", "Odometry")  # why the last sensor fault latched; 0 is also an avoidance-step check
def status_messages(subtype, value):
    sensor="Robopico/sensors/"
    actuator="Robopico/Actuators/"
    def one(section, field, v, motor=False):
        return [((actuator if motor else sensor)+section+"/"+field, v)]
    if subtype==0:
        phase=value&15
        return one("vehicle_controller","state",PHASES[phase] if phase<len(PHASES) else "Unknown",True)+one("vehicle_controller","fault_code",value>>4,True)
    if subtype==1:
        letter=value&31
        return one("barcode_ir","barcode",chr(64+letter) if 1<=letter<=26 else "—")+one("barcode_ir","navigation",("None","Left","Right","Straight","U-turn")[value>>5] if value>>5<5 else "Unknown")
    if subtype==2: return one("left_ir","line_state",value&7)+one("left_ir","black_mask",value>>3)
    if subtype==3: return one("magnetometer","pitch_deg",value-128)
    if subtype==4: return one("magnetometer","roll_deg",(value-128)*2)
    if subtype==5:
        terrain=value&7
        return one("terrain","hump_state",TERRAIN[terrain] if terrain<len(TERRAIN) else "Unknown")+one("barcode_ir","elements_read",value>>3)
    if subtype in (6,7): return one("terrain","hump_height_cm" if subtype==6 else "run_max_height_cm",value/2)
    if subtype==8:
        # 0..100 total in 5 mm; 128+n / 192+n centre->left/right edge in 8 mm.
        if value>=128: return one("ultrasonic_ranger","width_right_cm" if value>=192 else "width_left_cm",round((value&63)*0.8,1))
        return one("ultrasonic_ranger","width_cm",value/2)
    if subtype==9:
        # Bits 0..5 quality flags; bits 6..7 sensor-fault cause.
        quality=value&63
        return (one("vehicle_controller","quality",quality,True)+one("vehicle_controller","safety_state","Armed" if quality&4 else "Stopped",True)+
                one("accelerometer","impact",bool(quality&2))+one("vehicle_controller","fault_cause",FAULT_CAUSES[value>>6],True))
    if subtype==10: return one("vehicle_controller","available",value,True)+one("terrain","valid",bool(value&64))
    if subtype==11: return one("vehicle_controller","sequence",value,True)
    if subtype==12: return one("vehicle_controller","dropped",value,True)
    if subtype==13: return one("motors","speed_mps",(value-128)/50,True)
    if subtype==14: return one("left_ir","line_error",(value-128)*10)
    return (one("servo_motor","scan_state",value&15,True)+one("barcode_ir","decode_result",BARCODE_RESULTS[(value>>4)&3])+
            one("servo_motor","bypass_side",BYPASS_SIDES[value>>6],True))

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
