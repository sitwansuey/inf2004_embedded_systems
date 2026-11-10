import json, threading, unittest, http.client
from functools import partial
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import Mock
from dashboard import BrokerBridge, DashboardHandler

class Commands(unittest.TestCase):
 def bridge(self):
  b=BrokerBridge();c=Mock();c.is_connected.return_value=True
  c.publish.return_value=SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
  def publish(topic, frame, **kwargs):
   b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=False,
                payload=bytes([194,frame[5],frame[1],0,0,0])))
   return SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
  c.publish.side_effect=publish
  b.command_client=c;return b,c
 def test_wire(self):
  b,c=self.bridge()
  for action,frame in [('reset',bytes([193,2,0,0,0,0])),('start',bytes([193,3,30,0,0,1])),('stop',bytes([193,0,0,0,0,2]))]:
   b.send_command(action);c.publish.assert_called_with('csc02/command',frame,qos=0,retain=False)
 def test_disconnect_and_invalid(self):
  b,c=self.bridge();c.is_connected.return_value=False
  with self.assertRaises(RuntimeError):b.send_command('start')
  with self.assertRaises(ValueError):b.send_command('drive')
  c.publish.assert_not_called()
 def test_wrong_ack_and_rejection(self):
  b,c=self.bridge()
  b.pending_commands[9]={"opcode":3,"reply":None}
  for seq,op,retained in [(8,3,False),(9,2,False),(9,3,True)]:
   b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=retained,
              payload=bytes([194,seq,op,0,0,0])))
  self.assertIsNone(b.pending_commands[9]["reply"])
  def reject(topic,frame,**kwargs):
   b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=False,
              payload=bytes([194,frame[5],frame[1],2,0,0])))
   return SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
  c.publish.side_effect=reject
  with self.assertRaisesRegex(RuntimeError,"Reset robot first"):b.send_command("start")
 def test_http_origin_and_validation(self):
  b,c=self.bridge();server=ThreadingHTTPServer(('127.0.0.1',0),partial(DashboardHandler,bridge=b))
  t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
  try:
   for origin,payload,expected in [('http://evil.example',{'action':'start'},403),
    (f'http://127.0.0.1:{server.server_port}',{'action':'drive'},400),
    (f'http://127.0.0.1:{server.server_port}',{'action':'reset'},200)]:
    h=http.client.HTTPConnection('127.0.0.1',server.server_port)
    h.request('POST','/api/command',json.dumps(payload),{'Content-Type':'application/json','Origin':origin})
    response=h.getresponse();self.assertEqual(response.status,expected);response.read();h.close()
   self.assertEqual(c.publish.call_count,1)
  finally:server.shutdown();server.server_close();t.join()
 def test_loopback_origins_and_json_errors(self):
  b,c=self.bridge();server=ThreadingHTTPServer(('127.0.0.1',0),partial(DashboardHandler,bridge=b))
  t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
  try:
   port=server.server_port
   cases=[(f'localhost:{port}',{'Origin':f'http://localhost:{port}'},200),
          (f'127.0.0.1:{port}',{'Referer':f'http://127.0.0.1:{port}/'},200),
          (f'127.0.0.1:{port}',{'Origin':'http://evil.example'},403),
          (f'127.0.0.1:{port}',{},403),
          (f'127.0.0.1:{port}',{'Origin':'null'},403),
          (f'evil.example:{port}',{'Origin':f'http://evil.example:{port}'},403)]
   for host,extra,status in cases:
    with self.subTest(host=host,headers=extra):
     h=http.client.HTTPConnection('127.0.0.1',port)
     h.request('POST','/api/command',json.dumps({'action':'reset'}),
               {'Host':host,'Content-Type':'application/json',**extra})
     response=h.getresponse();self.assertEqual(response.status,status)
     self.assertIn('application/json',response.getheader('Content-Type'))
     result=json.loads(response.read());self.assertIn('error' if status==403 else 'acknowledged',result);h.close()
   self.assertEqual(c.publish.call_count,2)
  finally:server.shutdown();server.server_close();t.join()
 def ack_with(self,b,c,result):
  def reply(topic,frame,**kwargs):
   b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=False,
                payload=bytes([194,frame[5],frame[1],result,0,0])))
   return SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
  c.publish.side_effect=reply
 def test_map_wire_and_table(self):
  b,c=self.bridge()
  self.assertEqual(b.snapshot()['barcode_map'],{'A':'U-turn','F':'Left','Z':'Right'})
  for args,frame in [(('map','A','left'),bytes([193,4,1,1,0,0])),(('map','z','uturn'),bytes([193,4,26,4,0,1])),
                     (('map','m','forward'),bytes([193,4,13,3,0,2])),(('map','F','ignore'),bytes([193,4,6,0,0,3])),
                     (('map','B','right'),bytes([193,4,2,2,0,4]))]:
   r=b.send_command(*args);c.publish.assert_called_with('csc02/command',frame,qos=0,retain=False)
   self.assertTrue(r['acknowledged'])
  self.assertEqual(b.snapshot()['barcode_map'],{'A':'Left','B':'Right','F':'Ignore','M':'Forward','Z':'U-turn'})
  r=b.send_command('map_reset');c.publish.assert_called_with('csc02/command',bytes([193,4,0,0,0,5]),qos=0,retain=False)
  self.assertEqual(r['barcode_map'],{'A':'U-turn','F':'Left','Z':'Right'})
  self.assertEqual(b.snapshot()['barcode_map'],{'A':'U-turn','F':'Left','Z':'Right'})
 def test_map_ack_validator(self):
  b,c=self.bridge()
  b.pending_commands[7]={"opcode":4,"reply":None}
  for op in (5,3):
   b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=False,payload=bytes([194,7,op,0,0,0])))
  self.assertIsNone(b.pending_commands[7]["reply"])
  b.on_message(None,None,SimpleNamespace(topic="csc02/command/ack",retain=False,payload=bytes([194,7,4,1,9,2])))
  self.assertEqual(b.pending_commands[7]["reply"],(1,9,2))
 def test_map_table_changes_only_on_accept(self):
  b,c=self.bridge()
  for result,pattern in [(1,"rejected A = Left"),(3,"expired")]:
   self.ack_with(b,c,result)
   with self.assertRaisesRegex(RuntimeError,pattern):b.send_command('map','A','left')
   with self.assertRaises(RuntimeError):b.send_command('map_reset')
  b.barcode_map['Q']='Left';self.ack_with(b,c,1)
  with self.assertRaises(RuntimeError):b.send_command('map_reset')
  self.assertEqual(b.snapshot()['barcode_map'],{'A':'U-turn','F':'Left','Q':'Left','Z':'Right'})
  c.publish.side_effect=lambda *a,**kw:SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
  b.ack_condition.wait_for=lambda *a,**kw:False  # no ACK arrives
  with self.assertRaisesRegex(RuntimeError,"did not acknowledge"):b.send_command('map','Q','right')
  self.assertEqual(b.barcode_map['Q'],'Left');self.assertEqual(b.pending_commands,{})
 def test_map_invalid_input(self):
  b,c=self.bridge()
  for args in [('map','','left'),('map','AB','left'),('map','1','left'),('map','ß','left'),('map','ﬆ','left'),
               ('map',None,'left'),('map',5,'left'),('map','A','straight'),('map','A','Left'),('map','A',None),
               ('map','A',['left']),('map_reset','A',None),('start',None,'left'),('map_set','A','left')]:
   with self.subTest(args=args),self.assertRaises(ValueError):b.send_command(*args)
  c.publish.assert_not_called()
 def test_send_all(self):
  b,c=self.bridge()
  b.send_command('map','A','ignore');b.send_command('map','C','left');b.send_command('map','Z','right')
  c.publish.reset_mock()
  r=b.send_command('send_all')
  # Every letter's final value, A..Z, no codebook reset first (no default window).
  frames=[call.args[1][1:5] for call in c.publish.call_args_list]
  want={1:0,3:1,6:1,26:2}  # A ignore, C left, F left (default), Z right; others no turn
  self.assertEqual(frames,[bytes([4,n,want.get(n,0),0]) for n in range(1,27)])
  self.assertEqual(r['sent'],26);self.assertEqual(r['barcode_map'],{'A':'Ignore','C':'Left','F':'Left','Z':'Right'})
  self.assertEqual(b.snapshot()['barcode_map'],r['barcode_map'])
  c.publish.reset_mock();self.ack_with(b,c,1)
  with self.assertRaisesRegex(RuntimeError,"Send all stopped"):b.send_command('send_all')
  self.assertEqual(c.publish.call_count,1);self.assertEqual(b.snapshot()['barcode_map'],r['barcode_map'])
 def test_http_map_validation(self):
  b,c=self.bridge();server=ThreadingHTTPServer(('127.0.0.1',0),partial(DashboardHandler,bridge=b))
  t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
  try:
   port=server.server_port
   def post(payload):
    h=http.client.HTTPConnection('127.0.0.1',port)
    h.request('POST','/api/command',json.dumps(payload),{'Content-Type':'application/json','Origin':f'http://127.0.0.1:{port}'})
    response=h.getresponse();result=json.loads(response.read());h.close();return response.status,result
   for payload in [{'action':'map','letter':'1','turn':'left'},{'action':'map','letter':'A'},
                   {'action':'map','letter':'A','turn':'up'},{'action':'map','letter':'A','turn':'left','x':1},
                   {'action':'map_reset','letter':'A'},{'action':'send_all','turn':'left'},
                   {'action':'map','letter':['A'],'turn':'left'},{'action':'map','letter':'A','turn':{}}]:
    with self.subTest(payload=payload):
     status,result=post(payload);self.assertEqual(status,400);self.assertTrue(result['error'])
   c.publish.assert_not_called()
   status,result=post({'action':'map','letter':'d','turn':'uturn'})
   self.assertEqual(status,200);self.assertIn('D to U-turn',result['message'])
   h=http.client.HTTPConnection('127.0.0.1',port);h.request('GET','/api/telemetry')
   snapshot=json.loads(h.getresponse().read());h.close()
   self.assertEqual(snapshot['barcode_map'],{'A':'U-turn','D':'U-turn','F':'Left','Z':'Right'})
   self.assertEqual(post({'action':'map_reset'})[0],200);self.assertEqual(post({'action':'send_all'})[0],200)
   self.assertEqual(c.publish.call_count,1+1+26)  # map, map_reset, send_all writes A..Z
  finally:server.shutdown();server.server_close();t.join()
if __name__=='__main__':unittest.main()
