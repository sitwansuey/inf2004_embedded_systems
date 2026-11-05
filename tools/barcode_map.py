"""Send the instructor's four-letter route mapping and wait for the car's ACK.

Does not send START. BarcodeDemo firmware automatically starts 3 seconds after
a mapping and ready sensors. Position the robot before publishing. Requires paho-mqtt 2.x.
"""
import argparse
import json
import os
import threading

MAP_TOPIC = "csc02/barcode/map"
ACK_TOPIC = MAP_TOPIC + "/ack"
ACTIONS = ("forward", "uturn", "left", "right")
RESULTS = ("applied", "robot is armed; stop before changing the mapping",
           "invalid mapping", "command queue full", "mapping expired before application")


def mapping_payload(mapping):
    if set(mapping) != set(ACTIONS):
        raise ValueError("provide forward, uturn, left and right")
    letters = [mapping[key] for key in ACTIONS]
    if any(not isinstance(x, str) or len(x) != 1 or not "A" <= x <= "Z" for x in letters):
        raise ValueError("each movement needs one uppercase A-Z character")
    if len(set(letters)) != 4:
        raise ValueError("the four characters must be different")
    return json.dumps(mapping, separators=(",", ":")).encode("ascii"), "".join(letters).encode("ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for action in ACTIONS:
        parser.add_argument("--" + action, required=True)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--username", default=os.getenv("MQTT_USERNAME"))
    parser.add_argument("--timeout", type=float, default=10)
    parser.add_argument("--dry-run", action="store_true", help="print payload without connecting")
    parser.add_argument("--retain", action="store_true", help="broker resends configuration after reboot")
    args = parser.parse_args()
    try:
        payload, letters = mapping_payload({k: getattr(args, k) for k in ACTIONS})
    except ValueError as error:
        parser.error(str(error))
    if args.dry_run:
        print(MAP_TOPIC, payload.decode("ascii"))
        return
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.exit(1, "Install paho-mqtt: python -m pip install -r app_program/external_systems/Dashboard/requirements.txt\n")
    finished = threading.Event()
    outcome = []

    def fail(message):
        outcome.append(message)
        finished.set()

    def on_connect(client, userdata, flags, reason, properties):
        if reason.is_failure:
            fail(f"MQTT connection rejected: {reason}")
            return
        result, _ = client.subscribe(ACK_TOPIC, qos=1)
        if result != mqtt.MQTT_ERR_SUCCESS:
            fail("could not subscribe to the car's mapping acknowledgements")

    def on_subscribe(client, userdata, mid, reasons, properties):
        if not reasons or any(r.is_failure for r in reasons):
            fail("broker rejected mapping ACK subscription")
            return
        info = client.publish(MAP_TOPIC, payload, qos=1, retain=args.retain)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            fail("mapping publication failed")

    def on_message(client, userdata, message):
        ack = bytes(message.payload)
        if message.topic != ACK_TOPIC or len(ack) != 6 or ack[0] != 0xC4:
            return
        if ack[2:] != letters and not (ack[1] == 2 and ack[2:] == bytes(4)):
            return
        if ack[1] >= len(RESULTS):
            fail("unknown acknowledgement result")
        elif ack[1]:
            fail(RESULTS[ack[1]])
        else:
            outcome.append(None)
            finished.set()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if args.username:
        client.username_pw_set(args.username, os.getenv("MQTT_PASSWORD"))
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    try:
        client.connect(args.broker, args.port, keepalive=30)
        client.loop_start()
        if not finished.wait(args.timeout):
            parser.exit(1, "No application ACK from the car; mapping is not confirmed.\n")
        if outcome[0]:
            parser.exit(1, "Mapping rejected: " + outcome[0] + "\n")
        print("Car applied mapping: " + ", ".join(f"{k}={getattr(args, k)}" for k in ACTIONS))
        print("BarcodeDemo auto-start begins after 3 seconds of ready sensors; normal Mqtt uses explicit START.")
    except OSError as error:
        parser.exit(1, f"MQTT connection failed: {error}\n")
    finally:
        client.disconnect()
        client.loop_stop()


if __name__ == "__main__":
    main()
