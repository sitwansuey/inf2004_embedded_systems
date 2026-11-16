"""Width is produced by C, decoded from the single EDR16 topic and shown locally."""
from pathlib import Path
import subprocess,unittest
from types import SimpleNamespace
from unittest.mock import patch
from dashboard import BrokerBridge
from packed16 import PACKED_TOPIC,decode_messages
WIDTH='Robopico/sensors/ultrasonic_ranger/width_cm'
LEFT='Robopico/sensors/ultrasonic_ranger/width_left_cm'
RIGHT='Robopico/sensors/ultrasonic_ranger/width_right_cm'
def packet(subtype,value):return (((subtype<<8|value)<<4)).to_bytes(2,'little')
def send(bridge,payload,when):
 with patch('dashboard.time.monotonic',return_value=when):
  bridge.on_message(None,None,SimpleNamespace(topic=PACKED_TOPIC,payload=payload,retain=False))
class WidthPipeline(unittest.TestCase):
 def test_c_producer_to_dashboard(self):
  binary=Path(__file__).resolve().parents[3]/'testing/integration/.build/width_avoidance'
  if binary.with_suffix('.exe').exists():binary=binary.with_suffix('.exe')
  if not binary.exists():self.skipTest('Build testing/integration width_avoidance first')
  raw=bytes.fromhex(subprocess.check_output([str(binary),'--packet'],text=True).strip())
  self.assertEqual(len(raw),2);b=BrokerBridge();send(b,raw,10)
  self.assertEqual(b.readings[WIDTH]['payload'],17.0)  # isqrt halves 86+86=172 mm -> 34*5 mm
  self.assertEqual(b.received_packets,1);self.assertEqual(b.rejected_packets,0)
 def test_static_width_with_live_stream_and_invalid_quality(self):
  b=BrokerBridge();send(b,packet(8,35),10);send(b,packet(9,1),10.1)
  send(b,(1).to_bytes(2,'little'),11.2)
  with patch('dashboard.time.monotonic',return_value=11.25):
   width=next(x for x in b.snapshot()['messages'] if x['topic']==WIDTH)
   self.assertEqual(width['payload'],17.5)
  send(b,packet(9,0),11.3);self.assertIsNone(b.readings[WIDTH]['payload'])
 def test_half_widths_paged_retained_and_cleared(self):
  # Fixture d0=150, edges 173 mm at +/-30 deg: halves 86 mm -> (86+4)/8=11 -> 8.8 cm.
  b=BrokerBridge();send(b,packet(8,34),10);send(b,packet(8,128|11),10);send(b,packet(8,192|11),10)
  send(b,packet(9,1),10.1)
  self.assertEqual((b.readings[WIDTH]['payload'],b.readings[LEFT]['payload'],b.readings[RIGHT]['payload']),(17.0,8.8,8.8))
  send(b,packet(8,192|63),10.2);self.assertEqual(b.readings[RIGHT]['payload'],50.4)
  send(b,(1).to_bytes(2,'little'),11.2)
  with patch('dashboard.time.monotonic',return_value=11.25):
   kept={x['topic']:x['payload'] for x in b.snapshot()['messages']}
   self.assertEqual((kept[LEFT],kept[RIGHT]),(8.8,50.4))
  send(b,packet(9,0),11.3)
  for topic in (WIDTH,LEFT,RIGHT):self.assertIsNone(b.readings[topic]['payload'])
  for value in (101,127):
   with self.assertRaises(ValueError):decode_messages(packet(8,value))
 def test_offline_or_disconnect_clears_width(self):
  b=BrokerBridge();send(b,packet(8,35),10);send(b,packet(9,1),10.1)
  # No time expiry: the width keeps its latest value while the bridge waits.
  with patch('dashboard.time.monotonic',return_value=12):
   b.snapshot();self.assertIsNotNone(b.readings[WIDTH]['payload'])
  send(b,packet(8,35),13);b.reset_observations();self.assertFalse(b.width_valid)
  self.assertIsNone(b.readings[WIDTH]['payload'])
  send(b,packet(8,128|5),14);b.reset_observations();self.assertIsNone(b.readings[LEFT]['payload'])
if __name__=='__main__':unittest.main()
