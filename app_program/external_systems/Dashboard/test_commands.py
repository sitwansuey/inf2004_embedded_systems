import json, threading, unittest, http.client
from pathlib import Path
import tempfile
from functools import partial
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import Mock
from dashboard import BrokerBridge, DashboardHandler, MAP_TOPIC, MAP_ACK_TOPIC

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
  for action,frame in [('reset',bytes([193,2,0,0,0,0])),('start',bytes([193,3,20,0,0,1])),('stop',bytes([193,0,0,0,0,2]))]:
   b.send_command(action);c.publish.assert_called_with('csc02/command',frame,qos=0,retain=False)
 def test_disconnect_and_invalid(self):
  b,c=self.bridge();c.is_connected.return_value=False
  with self.assertRaises(RuntimeError):b.send_command('start')
  with self.assertRaises(ValueError):b.send_command('drive')
  c.publish.assert_not_called()
 def test_reset_restores_saved_map_without_start(self):
  with tempfile.TemporaryDirectory() as directory:
   path=Path(directory)/"map.json"
   mapping=dict(forward="E",uturn="M",left="Q",right="Z")
   path.write_text(json.dumps(mapping))
   b,c=self.bridge();b.mapping_file=path;calls=[];configured=False
   def publish(topic,data,**kwargs):
    nonlocal configured
    calls.append((topic,data,kwargs))
    if topic==MAP_TOPIC:
     self.assertEqual(json.loads(data),mapping);self.assertEqual(kwargs,dict(qos=1,retain=False))
     configured=True;reply=bytes.fromhex("c400454d515a");ack_topic=MAP_ACK_TOPIC
    else:
     self.assertIn(data[1],(0,2));self.assertEqual(kwargs,dict(qos=0,retain=False))
     if data[1]==2:self.assertTrue(configured)
     reply=bytes([194,data[5],data[1],0,0,0]);ack_topic="csc02/command/ack"
    b.on_message(None,None,SimpleNamespace(topic=ack_topic,payload=reply,retain=False))
    return SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
   c.publish.side_effect=publish
   self.assertIn("mapping restored",b.send_command("reset")["message"])
   self.assertEqual([t for t,_,_ in calls],["csc02/command",MAP_TOPIC,"csc02/command"])
   self.assertEqual([data[1] for t,data,k in calls if t=="csc02/command"],[0,2])
   self.assertEqual(BrokerBridge(path).saved_mapping,mapping)
   new=SimpleNamespace(topic=MAP_ACK_TOPIC,payload=b"\xc4\x00ZYXW",retain=True)
   b.on_message(None,None,new);self.assertEqual(json.loads(path.read_text()),mapping)
   new.retain=False;b.on_message(None,None,new)
   self.assertEqual(BrokerBridge(path).saved_mapping,dict(forward="Z",uturn="Y",left="X",right="W"))
 def test_missing_or_rejected_map_never_resets_or_starts(self):
  with tempfile.TemporaryDirectory() as directory:
   b,c=self.bridge();b.mapping_file=Path(directory)/"missing.json"
   with self.assertRaisesRegex(RuntimeError,"No saved movement map"):b.send_command("reset")
   self.assertEqual(c.publish.call_count,1);self.assertEqual(c.publish.call_args.args[1][1],0)
   b.mapping_file.write_text(json.dumps(dict(forward="E",uturn="M",left="Q",right="Z")))
   old=c.publish.side_effect
   def reject(topic,data,**kwargs):
    if topic!=MAP_TOPIC:return old(topic,data,**kwargs)
    b.on_message(None,None,SimpleNamespace(topic=MAP_ACK_TOPIC,payload=b"\xc4\x01EMQZ",retain=False))
    return SimpleNamespace(rc=0,wait_for_publish=lambda **kw:None,is_published=lambda:True)
   c.publish.reset_mock();c.publish.side_effect=reject
   with self.assertRaisesRegex(RuntimeError,"rejected the movement map"):b.send_command("reset")
   self.assertEqual(c.publish.call_count,2)
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
