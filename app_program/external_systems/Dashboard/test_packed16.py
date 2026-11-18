import http.client
import json
from pathlib import Path
import shutil
import subprocess
import threading
import unittest
from functools import partial
from types import SimpleNamespace
from packed16 import decode_messages, PACKED_TOPIC, READINGS
from dashboard import BrokerBridge, DashboardHandler, ThreadingHTTPServer, TOPICS
from edr_decode import encode_packet, pack_motor

# Values cover both wheels, every sensor axis, signed scaling, zero and limits.
SAMPLES={1:100,2:200,3:90,4:32,5:0,6:4095,7:2048,8:1948,9:2048,10:2148,
         11:pack_motor(1,30,1),12:32,13:3048,14:1048,15:2048}

def feed(bridge,topic,payload):
    bridge.on_message(None,None,SimpleNamespace(topic=topic,payload=payload))

def populated_bridge():
    bridge=BrokerBridge()
    for sid,value in SAMPLES.items(): feed(bridge,PACKED_TOPIC,encode_packet(sid,value))
    return bridge

class DashboardTests(unittest.TestCase):
    def test_exact_subscription_and_reconnect(self):
        calls=[]
        client=SimpleNamespace(subscribe=lambda topics: (calls.append(topics.copy()) or (0,1)))
        bridge=BrokerBridge()
        ok=SimpleNamespace(is_failure=False)
        for _ in range(2): bridge.on_connect(client,None,None,ok,None)
        self.assertEqual(calls,[TOPICS,TOPICS])
        self.assertEqual(TOPICS,[(PACKED_TOPIC,0),("csc02/command/ack",0)])
        bridge.on_subscribe(None,None,1,[ok],None)
        self.assertEqual(bridge.snapshot()['status'],'MQTT subscribed')

    def test_retained_disconnect_and_reconnect_clear(self):
        bridge=populated_bridge()
        count=bridge.received_packets
        bridge.on_message(None,None,SimpleNamespace(topic=PACKED_TOPIC,
            payload=encode_packet(4,10),retain=True))
        self.assertEqual(bridge.received_packets,count)
        self.assertEqual(bridge.readings[READINGS[4]]['payload'],'32')
        before=bridge.sequence
        bridge.on_disconnect(None,None,None,None,None)
        self.assertTrue(all(m['payload'] is None for m in bridge.snapshot()['messages']))
        self.assertGreater(bridge.sequence,before)
        self.assertIsNone(bridge.last_received_at)
        feed(bridge,PACKED_TOPIC,encode_packet(4,10))
        client=SimpleNamespace(subscribe=lambda topics:(0,1))
        bridge.on_connect(client,None,None,SimpleNamespace(is_failure=False),None)
        self.assertIsNone(bridge.readings[READINGS[4]]['payload'])

    def test_servo_validity_and_invalid_status(self):
        bridge=populated_bridge()
        feed(bridge,PACKED_TOPIC,encode_packet(0,0x93f))
        self.assertEqual(bridge.readings[READINGS[3]]['payload'],'90')
        feed(bridge,PACKED_TOPIC,encode_packet(0,0x91f))
        self.assertIsNone(bridge.readings[READINGS[3]]['payload'])
        feed(bridge,PACKED_TOPIC,(0x2070).to_bytes(2,'little'))
        self.assertEqual(bridge.rejected_packets,1)

    def test_fault_cause_and_quality_mask(self):
        bridge=populated_bridge();vc='Robopico/Actuators/vehicle_controller/'
        width='Robopico/sensors/ultrasonic_ranger/width_cm'
        for cause,label in ((1,'Sonar'),(2,'Tilt'),(3,'Odometry'),(0,'None/other')):
            feed(bridge,PACKED_TOPIC,encode_packet(0,(8<<8)|35))
            feed(bridge,PACKED_TOPIC,encode_packet(0,(9<<8)|63|(cause<<6)))
            self.assertEqual(bridge.readings[vc+'fault_cause']['payload'],label)
            self.assertEqual(bridge.readings[vc+'quality']['payload'],63)
            self.assertEqual(bridge.readings[vc+'safety_state']['payload'],'Armed')
            self.assertTrue(bridge.readings['Robopico/sensors/accelerometer/impact']['payload'])
            self.assertEqual(bridge.readings[READINGS[3]]['payload'],'90')
            self.assertEqual(bridge.readings[width]['payload'],17.5)
        # Cause bits alone never stand in for quality bits 0/1/2/4/5.
        feed(bridge,PACKED_TOPIC,encode_packet(0,(9<<8)|(3<<6)))
        self.assertEqual(bridge.readings[vc+'fault_cause']['payload'],'Odometry')
        self.assertEqual(bridge.readings[vc+'quality']['payload'],0)
        self.assertEqual(bridge.readings[vc+'safety_state']['payload'],'Stopped')
        self.assertFalse(bridge.readings['Robopico/sensors/accelerometer/impact']['payload'])
        self.assertIsNone(bridge.readings[READINGS[3]]['payload'])
        self.assertIsNone(bridge.readings[width]['payload'])
        self.assertEqual(decode_messages(encode_packet(0,(9<<8)|1|(2<<6)))[-1],(vc+'fault_cause','Tilt'))

    def test_all_ids_and_units(self):
        bridge=populated_bridge();snapshot=bridge.snapshot()
        self.assertEqual(snapshot['mqtt_topic'],PACKED_TOPIC)
        self.assertEqual(snapshot['received_packets'],15)
        self.assertEqual(len(snapshot['messages']),17)
        self.assertEqual(snapshot['rejected_packets'],0)
        self.assertIsNotNone(snapshot['last_received_at'])
        for sid in (1,2,3,4,5,6,7,12):
            self.assertEqual(int(bridge.readings[READINGS[sid]]['payload']),SAMPLES[sid])
        for sid,value in ((8,-10),(9,0),(10,10),(13,9.80665),(14,-9.80665),(15,0)):
            self.assertAlmostEqual(float(bridge.readings[READINGS[sid]]['payload']),value)
        for field,value in (('state','MOVING'),('direction','FORWARD'),('power_percent','30')):
            self.assertEqual(bridge.readings['Robopico/Actuators/motors/'+field]['payload'],value)
        self.assertEqual(decode_messages(b'\x01\x00'),[(READINGS[1],'0')])

    def test_invalid_packets_and_other_topics(self):
        bridge=populated_bridge();before=bridge.snapshot()
        invalid=(b'',b'\x01',b'\x01\x00\x02\x00',
                 bytes.fromhex('530b'),bytes.fromhex('7b00'),bytes.fromhex('0c08'),b'4106')
        for payload in invalid: feed(bridge,PACKED_TOPIC,payload)
        for topic in (READINGS[1],'Robopico/sensors/packed16/v1','other'):
            feed(bridge,topic,b'123')
        after=bridge.snapshot()
        self.assertEqual(after['messages'],before['messages'])
        self.assertEqual(after['received_packets'],15)
        self.assertEqual(after['rejected_packets'],len(invalid))
        self.assertEqual(after['last_received_at'],before['last_received_at'])
        feed(bridge,PACKED_TOPIC,encode_packet(1,101))
        self.assertEqual(len(bridge.readings),17)
        self.assertEqual(bridge.readings[READINGS[1]]['payload'],'101')
        self.assertNotEqual(bridge.receiver_id,BrokerBridge().receiver_id)

    def test_status_records_and_no_expiry(self):
        bridge=populated_bridge()
        feed(bridge,PACKED_TOPIC,encode_packet(0,(7<<8)|91))
        peak='Robopico/sensors/terrain/run_max_height_cm'
        self.assertEqual(bridge.readings[peak]['payload'],45.5)
        feed(bridge,PACKED_TOPIC,encode_packet(0,(1<<8)|3|(4<<5)))
        self.assertEqual(bridge.readings['Robopico/sensors/barcode_ir/navigation']['payload'],'U-turn')
        self.assertEqual(bridge.readings['Robopico/sensors/barcode_ir/barcode']['payload'],'C')
        feed(bridge,PACKED_TOPIC,encode_packet(0,(5<<8)|1|(15<<3)))
        self.assertEqual(bridge.readings['Robopico/sensors/barcode_ir/elements_read']['payload'],15)
        self.assertEqual(bridge.readings['Robopico/sensors/terrain/hump_state']['payload'],'Ascent')
        for result,label in ((2,'Successful'),(3,'Unsuccessful'),(1,'Reading...')):
            feed(bridge,PACKED_TOPIC,encode_packet(0,(15<<8)|4|(result<<4)))
            self.assertEqual(bridge.readings['Robopico/sensors/barcode_ir/decode_result']['payload'],label)
            self.assertEqual(bridge.readings['Robopico/Actuators/servo_motor/scan_state']['payload'],4)
        side='Robopico/Actuators/servo_motor/bypass_side'
        for bits,label in ((1,'Left'),(2,'Right'),(0,'None')):
            feed(bridge,PACKED_TOPIC,encode_packet(0,(15<<8)|9|(2<<4)|(bits<<6)))
            self.assertEqual(bridge.readings[side]['payload'],label)
            self.assertEqual(bridge.readings['Robopico/Actuators/servo_motor/scan_state']['payload'],9)
            self.assertEqual(bridge.readings['Robopico/sensors/barcode_ir/decode_result']['payload'],'Successful')
        with self.assertRaises(ValueError): encode_packet(0,(15<<8)|(3<<6))
        feed(bridge,PACKED_TOPIC,encode_packet(0,(8<<8)|128|11))
        feed(bridge,PACKED_TOPIC,encode_packet(0,(8<<8)|192|63))
        self.assertEqual(bridge.readings['Robopico/sensors/ultrasonic_ranger/width_left_cm']['payload'],8.8)
        self.assertEqual(bridge.readings['Robopico/sensors/ultrasonic_ranger/width_right_cm']['payload'],50.4)
        with self.assertRaises(ValueError): encode_packet(0,(8<<8)|101)
        feed(bridge,PACKED_TOPIC,encode_packet(0,13|(1<<4)))
        self.assertEqual(bridge.readings['Robopico/Actuators/vehicle_controller/state']['payload'],'Reverse')
        with self.assertRaises(ValueError): encode_packet(0,14)
        feed(bridge,PACKED_TOPIC,encode_packet(0,10<<8))
        self.assertIsNone(bridge.readings[READINGS[1]]['payload'])
        # Readings keep their latest value however old: no time expiry.
        self.assertEqual({m['topic']:m['payload'] for m in bridge.snapshot()['messages']}[peak],45.5)

    def test_lifted_fault_code(self):
        bridge=populated_bridge();vc='Robopico/Actuators/vehicle_controller/'
        feed(bridge,PACKED_TOPIC,encode_packet(0,(8<<4)|12))  # phase 12 Fault, fault 8 lifted
        self.assertEqual(bridge.readings[vc+'state']['payload'],'Fault')
        self.assertEqual(bridge.readings[vc+'fault_code']['payload'],8)
        with self.assertRaises(ValueError): encode_packet(0,(9<<4)|12)
        feed(bridge,PACKED_TOPIC,((((9<<4)|12)<<4)).to_bytes(2,'little'))  # fault 9 is rejected
        self.assertEqual(bridge.rejected_packets,1)
        self.assertEqual(bridge.readings[vc+'fault_code']['payload'],8)

    def test_barcode_read_history(self):
        bridge=BrokerBridge()
        status=lambda subtype,value: feed(bridge,PACKED_TOPIC,encode_packet(0,(subtype<<8)|value))
        motor=lambda command,power: feed(bridge,PACKED_TOPIC,encode_packet(11,pack_motor(command,power,1)))
        READING,OK,FAIL=1<<4,2<<4,3<<4
        # Normal-speed pass sees the whole marker but fails.
        motor(1,30);status(15,READING);status(5,12<<3);status(5,29<<3);status(15,READING);status(15,FAIL)
        # A line crossing (1 element) is not a barcode read.
        status(15,READING);status(5,1<<3);status(15,FAIL)
        # Reverse re-read decodes A; letter and turn arrive just after the result.
        motor(2,9);status(15,READING);status(5,29<<3);status(15,OK);status(1,1|(4<<5))
        # Slow final pass.
        motor(1,9);status(15,READING);status(5,27<<3);status(15,FAIL)
        rows=bridge.snapshot()['barcode_history']
        self.assertEqual([(r['pass'],r['elements'],r['result'],r['letter'],r['turn']) for r in rows],
            [('Normal',29,'Unsuccessful',None,None),('Reverse',29,'Successful','A','U-turn'),
             ('Slow',27,'Unsuccessful',None,None)])
        self.assertTrue(all(r['time'] and r['seconds']>=0 for r in rows))
        # Repeated result records do not add rows; history is bounded.
        status(15,FAIL);status(15,FAIL)
        self.assertEqual(len(bridge.snapshot()['barcode_history']),3)
        for _ in range(40): status(15,READING);status(5,20<<3);status(15,FAIL)
        self.assertEqual(len(bridge.snapshot()['barcode_history']),30)

    def test_http_snapshot_and_browser_display(self):
        bridge=populated_bridge()
        handler=partial(DashboardHandler,bridge=bridge,directory=str(Path(__file__).parent))
        server=ThreadingHTTPServer(('127.0.0.1',0),handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=5)
        try:
            connection.request('GET','/api/telemetry')
            response=connection.getresponse()
            self.assertEqual(response.status,200)
            self.assertEqual(response.getheader('Cache-Control'),'no-store')
            snapshot=json.loads(response.read())
            self.assertEqual(snapshot['messages'],bridge.snapshot()['messages'])
            node=shutil.which('node')
            if node:
                result=subprocess.run([node,str(Path(__file__).with_name('test_dashboard_display.cjs'))],
                    input=json.dumps(snapshot),capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                print(result.stdout.strip())
            else:
                print('Browser display test not run: Node.js is not on PATH')
        finally:
            connection.close();server.shutdown();server.server_close();thread.join(timeout=5)
if __name__=='__main__': unittest.main()
