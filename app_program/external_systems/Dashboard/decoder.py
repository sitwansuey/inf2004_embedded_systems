"""Decode one EDR16 MQTT record. The 50-byte mailbox format is internal only."""
from packed16 import decode_record as decode
from edr_decode import encode_packet as encode
if __name__ == "__main__":
    import argparse, json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex")
    print(json.dumps(decode(bytes.fromhex(parser.parse_args().hex)),indent=2))
