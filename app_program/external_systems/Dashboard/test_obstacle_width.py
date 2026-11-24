"""Width is produced by C, decoded from the single EDR16 topic and shown locally."""
from pathlib import Path
import subprocess,unittest
from types import SimpleNamespace
from dashboard import BrokerBridge
from packed16 import PACKED_TOPIC,decode_messages
WIDTH='Robopico/sensors/ultrasonic_ranger/width_cm'
LEFT='Robopico/sensors/ultrasonic_ranger/width_left_cm'
RIGHT='Robopico/sensors/ultrasonic_ranger/width_right_cm'
def packet(subtype,value):return (((subtype<<8|value)<<4)).to_bytes(2,'little')
def send(bridge,payload):
 bridge.on_message(None,None,SimpleNamespace(topic=PACKED_TOPIC,payload=payload,retain=False))
class WidthPipeline(unittest.TestCase):
 def test_c_producer_to_dashboard(self):
  binary=Path(__file__).resolve().parents[3]/'testing/integration/.build/width_avoidance'
  if binary.with_suffix('.exe').exists():binary=binary.with_suffix('.exe')
  if not binary.exists():self.skipTest('Build testing/integration width_avoidance first')
  raw=bytes.fromhex(subprocess.check_output([str(binary),'--packet'],text=True).strip())
  self.assertEqual(len(raw),2);b=BrokerBridge();send(b,raw)
  self.assertEqual(b.readings[WIDTH]['payload'],17.0)  # isqrt halves 86+86=172 mm -> 34*5 mm
  self.assertEqual(b.received_packets,1);self.assertEqual(b.rejected_packets,0)
 def test_width_kept_after_quality_bit_0_clears(self):
  b=BrokerBridge();send(b,packet(8,35));send(b,packet(9,1))
  send(b,(1).to_bytes(2,'little'))  # an unrelated record leaves the width alone
  width=next(x for x in b.snapshot()['messages'] if x['topic']==WIDTH)
  self.assertEqual(width['payload'],17.5)
  # The scan is over (bit 0 clear): the last scan's width stays shown.
  send(b,packet(9,0));self.assertEqual(b.readings[WIDTH]['payload'],17.5)
  send(b,packet(8,40));self.assertEqual(b.readings[WIDTH]['payload'],20.0)  # the next scan replaces it
 def test_half_widths_paged_retained_and_cleared(self):
  # Fixture d0=150, edges 173 mm at +/-30 deg: halves 86 mm -> (86+4)/8=11 -> 8.8 cm.
  b=BrokerBridge();send(b,packet(8,34));send(b,packet(8,128|11));send(b,packet(8,192|11))
  send(b,packet(9,1))
  self.assertEqual((b.readings[WIDTH]['payload'],b.readings[LEFT]['payload'],b.readings[RIGHT]['payload']),(17.0,8.8,8.8))
  send(b,packet(8,192|63));self.assertEqual(b.readings[RIGHT]['payload'],50.4)
  send(b,(1).to_bytes(2,'little'))
  kept={x['topic']:x['payload'] for x in b.snapshot()['messages']}
  self.assertEqual((kept[LEFT],kept[RIGHT]),(8.8,50.4))
  send(b,packet(9,0))
  self.assertEqual((b.readings[WIDTH]['payload'],b.readings[LEFT]['payload'],b.readings[RIGHT]['payload']),(17.0,8.8,50.4))
  for value in (101,127):
   with self.assertRaises(ValueError):decode_messages(packet(8,value))
 def test_scan_results_kept_through_a_broker_reconnect(self):
  b=BrokerBridge();send(b,packet(8,35));send(b,packet(9,1))
  # No time expiry, and a broker reconnect clears live readings only.
  b.snapshot();self.assertIsNotNone(b.readings[WIDTH]['payload'])
  send(b,packet(8,128|5));send(b,packet(10,16))
  b.reset_observations()
  self.assertEqual((b.readings[WIDTH]['payload'],b.readings[LEFT]['payload']),(17.5,4.0))
  self.assertIsNone(b.readings['Robopico/Actuators/vehicle_controller/available']['payload'])
 def test_bypass_side_kept_until_the_next_scan(self):
  side='Robopico/Actuators/servo_motor/bypass_side'
  b=BrokerBridge();send(b,packet(15,0))
  self.assertEqual(b.readings[side]['payload'],'None')  # nothing scanned yet
  send(b,packet(15,1<<6));self.assertEqual(b.readings[side]['payload'],'Left')
  send(b,packet(15,0));self.assertEqual(b.readings[side]['payload'],'Left')  # no plan now: kept
  b.reset_observations();self.assertEqual(b.readings[side]['payload'],'Left')
  send(b,packet(15,2<<6));self.assertEqual(b.readings[side]['payload'],'Right')
if __name__=='__main__':unittest.main()
