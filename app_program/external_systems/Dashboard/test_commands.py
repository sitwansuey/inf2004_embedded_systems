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
if __name__=='__main__':unittest.main()
