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
def decode_messages(payload):
    """Unmarshal one little-endian EDR16 record into local display fields.

    This does not publish MQTT messages. Motor ID 11 expands into three fields.
    Exact size, ID, and value ranges are validated before returning any fields.
    """
    record = decode_record(payload)
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
