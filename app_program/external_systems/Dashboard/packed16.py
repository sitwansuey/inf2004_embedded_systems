"""Dashboard field adapter; the sole Python wire codec is tools/edr_decode.py."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from edr_decode import decode_packet as decode_record, MQTT_TOPIC, EVENT_TOPIC, decode_event, counter_delta, IMU_FAULT_BIT
PACKED_TOPIC = MQTT_TOPIC
IMU_FAULT_TOPIC = "Robopico/sensors/accelerometer/imu_fault"
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
# Status subtype 9 bits 7-6, labelled as a sensor-fault (code 1) cause: why it
# latched; 0 is also an avoidance-step check. For a line fault (code 2) the
# same codes mean 0 other, 1 line data stale, 2 S3 fault event, 3 line not
# found (dashboard.js relabels them); any other fault sends 0.
FAULT_CAUSES = ("None/other", "Sonar", "Tilt", "Odometry")
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
        # Bits 0..5 quality flags; bits 6..7 fault cause (see FAULT_CAUSES).
        quality=value&63
        return (one("vehicle_controller","quality",quality,True)+one("vehicle_controller","safety_state","Armed" if quality&4 else "Stopped",True)+
                one("accelerometer","impact",bool(quality&2))+one("vehicle_controller","fault_cause",FAULT_CAUSES[value>>6],True))
    if subtype==10: return one("vehicle_controller","available",value,True)+one("terrain","valid",bool(value&64))
    if subtype==11: return one("vehicle_controller","sequence",value,True)
    # The robot's drop counter modulo 256 (wraps); dashboard.py shows the
    # drops of the last minute from the differences.
    if subtype==12: return one("vehicle_controller","dropped",value,True)
    if subtype==13: return one("motors","speed_mps",(value-128)/50,True)
    if subtype==14: return one("left_ir","line_error",(value-128)*10)
    return (one("servo_motor","scan_state",value&15,True)+one("barcode_ir","decode_result",BARCODE_RESULTS[(value>>4)&3])+
            one("servo_motor","bypass_side",BYPASS_SIDES[value>>6],True))

ATTEMPTS = {1: "initial scan", 2: "reverse scan", 3: "final attempt"}
EVENT_TURNS = ("no turn (not mapped)", "Left", "Right", "Straight", "U-turn")
SIDES = {1: "left", 2: "right"}

def describe_event(payload):
    """One robot event record -> (source, level, message) for the event log."""
    e = decode_event(payload)
    name = e["name"]
    if name == "barcode":
        attempt = f"Barcode attempt {e['attempt']} ({ATTEMPTS[e['attempt']]})"
        bars = f"{e['elements']}/29 bars & spaces read"
        if e["letter"]:
            return "Barcode", "INFO", f"{attempt}: decoded {chr(64 + e['letter'])}, {bars}"
        final = "; marker skipped" if e["attempt"] == 3 else ""
        return "Barcode", "WARN", f"{attempt}: failed, {bars}{final}"
    if name == "instruction":
        return "Barcode", "INFO", f"Instruction decoded: {chr(64 + e['letter'])} -> {EVENT_TURNS[e['turn']]}"
    if name == "hump":
        if e["kind"] == 1:
            return "Terrain", "INFO", f"Hump detected: climbing, tilt {e['value'] / 10:.1f} deg"
        # Heights in 2 mm; 0 when subsystem 4 lost the hump's height.
        height = f"{e['value'] * 2 / 10:.1f} cm" if e["value"] else "not measured"
        if e["kind"] == 2:
            return "Terrain", "INFO", f"Hump crest: going down slowly, height {height}"
        return "Terrain", "INFO", f"Hump cleared: level again, highest {height}"
    if name == "obstacle":
        kind, side, value = e["kind"], e["side"], e["value"]
        if kind == 1:
            return "Obstacle", "WARN", f"Obstacle detected {value} cm ahead"
        if kind == 2 and side == 0:
            return "Obstacle", "WARN", f"No way round found: scanning again (retry {value})"
        if kind == 2:
            return "Obstacle", "INFO", f"Obstacle scanned: {value} cm wide, bypass on the {SIDES[side]}"
        if kind == 3:
            return "Obstacle", "INFO", f"Bypass started: heading {value} deg to the {SIDES[side]}"
        if side == 0:
            return "Obstacle", "WARN", f"Obstacle avoidance stopped after {value} s (fault or STOP)"
        how = {1: "line found on the way back, heading restored",
               2: "line found by the on-the-spot search",
               3: "the obstacle went away"}[side]
        return "Obstacle", "INFO", f"Back on the line after {value} s ({how})"
    if e["kind"] == 1:
        return "Wi-Fi", "WARN", (f"Robot reconnected to the MQTT broker after {e['seconds']} s "
                                 "without it (Wi-Fi or broker link was down)")
    return "Wi-Fi", "INFO", f"Robot connected to the MQTT broker ({e['seconds']} s after boot)"

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
    if record["id"] == 12:
        # Shared fault bits: the IMU fault also shows on the Accelerometer card.
        return [(READINGS[12], str(record["value"])), (IMU_FAULT_TOPIC, bool(record["value"] & IMU_FAULT_BIT))]
    return [(READINGS[record["id"]], str(record["value"]))]
