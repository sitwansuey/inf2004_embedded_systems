import json
from pathlib import Path
import subprocess
import sys
import unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools"))
import edr_decode as edr
class CodecTests(unittest.TestCase):
    def test_all_words(self):
        for word in range(65536):
            sid,data=word & 15,word >> 4
            valid=(sid!=0 and (sid!=3 or data<=180) and (sid!=12 or data<=127)
                   and (sid!=11 or ((data&7)<=6 and ((data>>3)&127)<=100 and data>>10<=2)))
            packet=word.to_bytes(2,"little")
            if not valid:
                with self.assertRaises(ValueError): edr.decode_packet(packet)
                with self.assertRaises(ValueError): edr.encode_packet(sid,data)
            else:
                decoded=edr.decode_packet(packet)
                self.assertEqual((decoded["id"],decoded["data"]),(sid,data))
                self.assertEqual(edr.encode_packet(sid,data),packet)
    def test_vectors(self):
        vectors=json.loads((ROOT/"testing/data_representation/vectors.json").read_text())
        for v in vectors["valid_packets"]:
            result=edr.decode_packet(bytes.fromhex(v["hex"]))
            self.assertEqual((result["id"],result["data"]),(v["id"],v["data"]))
        for v in vectors["invalid_packets"]:
            with self.assertRaises(ValueError): edr.decode_packet(bytes.fromhex(v["hex"]))
    def test_ranges_and_units(self):
        for sid,data in ((0,1),(16,1),(1,4096),(1,-1),(True,1),(1,1.5)):
            with self.assertRaises(ValueError): edr.encode_packet(sid,data)
        self.assertEqual(edr.decode_packet(edr.encode_packet(8,0))["value"],-204.8)
        self.assertAlmostEqual(edr.decode_packet(edr.encode_packet(13,3048))["value"],9.80665)
        self.assertEqual(edr.decode_packet(edr.encode_packet(11,edr.pack_motor(1,30,1)))["motor"],
                         {"command":1,"power_percent":30,"state":1})
    def test_cli(self):
        result=subprocess.run([sys.executable,"-B",str(ROOT/"tools/edr_decode.py"),"41 06"],capture_output=True,text=True)
        self.assertEqual(result.returncode,0)
        self.assertEqual(json.loads(result.stdout)["data"],100)
        result=subprocess.run([sys.executable,"-B",str(ROOT/"tools/edr_decode.py"),"42 07 f1 04 00 78 56 34 12"],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
if __name__=="__main__": unittest.main()
