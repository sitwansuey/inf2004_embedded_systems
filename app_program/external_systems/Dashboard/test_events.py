"""Robot event log: event records (app_program/shared/events.h) -> text, the
receiver's event log, robot offline/online and broker link events."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from dashboard import BrokerBridge, ROBOT_SILENT_S, TOPICS
from packed16 import EVENT_TOPIC, PACKED_TOPIC, describe_event
from edr_decode import decode_event, encode_packet

SHARED = Path(__file__).resolve().parents[2] / "shared"


def record(event_id, value):
    """Same packing as s1_event_post: low nibble id, high 12 bits value."""
    return ((value << 4) | event_id).to_bytes(2, "little")


def barcode(attempt, elements, letter):
    return record(1, attempt | elements << 2 | letter << 7)


def feed(bridge, topic, payload, retain=False):
    bridge.on_message(None, None, SimpleNamespace(topic=topic, payload=payload, retain=retain))


class EventTextTests(unittest.TestCase):
    def test_barcode_attempts(self):
        self.assertEqual(describe_event(barcode(1, 29, 6)),
                         ("Barcode", "INFO", "Barcode attempt 1 (initial scan): decoded F, 29/29 bars & spaces read"))
        self.assertEqual(describe_event(barcode(2, 27, 0)),
                         ("Barcode", "WARN", "Barcode attempt 2 (reverse scan): failed, 27/29 bars & spaces read"))
        self.assertEqual(describe_event(barcode(3, 12, 0))[2],
                         "Barcode attempt 3 (final attempt): failed, 12/29 bars & spaces read; marker skipped")
        self.assertIn("decoded Z", describe_event(barcode(3, 29, 26))[2])
        for bad in (barcode(0, 29, 1), barcode(1, 29, 27)):
            with self.assertRaises(ValueError):
                describe_event(bad)

    def test_instruction(self):
        self.assertEqual(describe_event(record(2, 6 | 1 << 5))[2], "Instruction decoded: F -> Left")
        self.assertEqual(describe_event(record(2, 1 | 4 << 5))[2], "Instruction decoded: A -> U-turn")
        self.assertIn("not mapped", describe_event(record(2, 2))[2])
        for bad in (record(2, 0), record(2, 6 | 5 << 5), record(2, 6 | 1 << 8)):
            with self.assertRaises(ValueError):
                describe_event(bad)

    def test_hump(self):
        self.assertEqual(describe_event(record(3, 1 | 52 << 3))[2], "Hump detected: climbing, tilt 5.2 deg")
        self.assertEqual(describe_event(record(3, 2 | 15 << 3))[2], "Hump crest: going down slowly, height 3.0 cm")
        self.assertEqual(describe_event(record(3, 3 | 20 << 3))[2], "Hump cleared: level again, highest 4.0 cm")
        # 0: subsystem 4 lost this hump's height.
        self.assertEqual(describe_event(record(3, 2))[2], "Hump crest: going down slowly, height not measured")
        with self.assertRaises(ValueError):
            describe_event(record(3, 4))

    def test_obstacle(self):
        self.assertEqual(describe_event(record(4, 1 | 18 << 4)), ("Obstacle", "WARN", "Obstacle detected 18 cm ahead"))
        self.assertEqual(describe_event(record(4, 2 | 1 << 2 | 26 << 4))[2],
                         "Obstacle scanned: 26 cm wide, bypass on the left")
        self.assertEqual(describe_event(record(4, 3 | 2 << 2 | 31 << 4))[2],
                         "Bypass started: heading 31 deg to the right")
        self.assertEqual(describe_event(record(4, 0 | 1 << 2 | 9 << 4))[2],
                         "Back on the line after 9 s (line found on the way back, heading restored)")
        self.assertIn("on-the-spot search", describe_event(record(4, 0 | 2 << 2 | 12 << 4))[2])
        self.assertIn("went away", describe_event(record(4, 0 | 3 << 2 | 2 << 4))[2])
        # how 0: the avoidance ended in a fault or a STOP.
        self.assertEqual(describe_event(record(4, 0 | 7 << 4)),
                         ("Obstacle", "WARN", "Obstacle avoidance stopped after 7 s (fault or STOP)"))
        # Side 0 on a scan: no way round, scanning again (retry 1..3).
        self.assertEqual(describe_event(record(4, 2 | 1 << 4)),
                         ("Obstacle", "WARN", "No way round found: scanning again (retry 1)"))
        self.assertIn("retry 2", describe_event(record(4, 2 | 2 << 4))[2])
        for bad in (record(4, 1 | 1 << 2), record(4, 2), record(4, 2 | 4 << 4), record(4, 3), record(4, 3 | 3 << 2)):
            with self.assertRaises(ValueError):
                describe_event(bad)

    def test_link(self):
        self.assertEqual(describe_event(record(5, 1 | 42 << 2)),
                         ("Wi-Fi", "WARN", "Robot reconnected to the MQTT broker after 42 s without it "
                                            "(Wi-Fi or broker link was down)"))
        self.assertIn("8 s after boot", describe_event(record(5, 2 | 8 << 2))[2])
        with self.assertRaises(ValueError):
            describe_event(record(5, 0))

    def test_framing(self):
        for bad in (b"", b"\x01", b"\x01\x02\x03", record(0, 1), record(6, 1), record(15, 1)):
            with self.assertRaises(ValueError):
                decode_event(bad)

    @unittest.skipUnless(shutil.which("gcc"), "needs gcc")
    def test_matches_the_firmware_packing(self):
        """Encode with the real events.h helpers, decode here."""
        source = ('#include <stdio.h>\n#include "events.h"\n'
                  'static void out(unsigned id,unsigned v){unsigned w=(v<<4)|id;printf("%02x%02x\\n",w&255,w>>8);}\n'
                  'void s1_event_post(unsigned id,unsigned value){out(id,value);}\n'
                  'int main(void){'
                  's1_event_post(EV_BARCODE,ev_barcode(2,29,6));'
                  's1_event_post(EV_BARCODE,ev_barcode(3,40,0));'
                  's1_event_post(EV_INSTRUCTION,ev_instruction(26,2));'
                  's1_event_post(EV_HUMP,ev_hump(EV_HUMP_CREST,700));'
                  's1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_SCANNED,2,26));'
                  's1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_BACK,1,300));'
                  's1_event_post(EV_OBSTACLE,ev_obstacle(EV_OBSTACLE_BACK,0,4));'
                  's1_event_post(EV_LINK,ev_link(EV_LINK_RECONNECTED,5000));return 0;}\n')
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / "events_check.c"
            exe = Path(tmp) / "events_check.exe"
            c.write_text(source)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(SHARED), str(c),
                            "-o", str(exe)], check=True, capture_output=True)
            lines = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
        events = [decode_event(bytes.fromhex(line)) for line in lines]
        self.assertEqual(events[0], {"id": 1, "name": "barcode", "attempt": 2, "elements": 29, "letter": 6})
        self.assertEqual(events[1]["elements"], 31)  # saturates
        self.assertEqual((events[2]["letter"], events[2]["turn"]), (26, 2))
        self.assertEqual((events[3]["kind"], events[3]["value"]), (2, 511))  # saturates
        self.assertEqual((events[4]["kind"], events[4]["side"], events[4]["value"]), (2, 2, 26))
        self.assertEqual((events[5]["kind"], events[5]["side"], events[5]["value"]), (0, 1, 255))
        self.assertEqual((events[6]["kind"], events[6]["side"], events[6]["value"]), (0, 0, 4))
        self.assertEqual((events[7]["kind"], events[7]["seconds"]), (1, 1023))


class EventLogTests(unittest.TestCase):
    def test_subscribes_to_events(self):
        self.assertIn((EVENT_TOPIC, 0), TOPICS)

    def test_robot_events_are_logged(self):
        bridge = BrokerBridge()
        feed(bridge, EVENT_TOPIC, barcode(1, 29, 6))
        feed(bridge, EVENT_TOPIC, barcode(1, 29, 6), retain=True)  # retained: ignored
        feed(bridge, EVENT_TOPIC, b"\x00\x00")                     # invalid: rejected
        events = bridge.snapshot()["events"]
        self.assertEqual(len(events), 1)
        self.assertEqual((events[0]["source"], events[0]["level"]), ("Barcode", "INFO"))
        self.assertIn("attempt 1", events[0]["message"])
        self.assertEqual(bridge.rejected_packets, 1)

    def test_log_is_bounded(self):
        bridge = BrokerBridge()
        for n in range(250):
            feed(bridge, EVENT_TOPIC, record(4, 1 | (n % 200) << 4))
        self.assertEqual(len(bridge.snapshot()["events"]), 200)

    def test_robot_offline_and_back_online(self):
        bridge = BrokerBridge()
        feed(bridge, PACKED_TOPIC, encode_packet(4, 30))
        start = bridge.last_received_at
        bridge.check_robot(start + ROBOT_SILENT_S - 0.5)
        self.assertEqual(bridge.snapshot()["events"], [])
        bridge.check_robot(start + ROBOT_SILENT_S + 0.1)
        bridge.check_robot(start + ROBOT_SILENT_S + 5)   # logged once
        events = bridge.snapshot()["events"]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["level"], "WARN")
        self.assertIn("Robot offline", events[0]["message"])
        feed(bridge, PACKED_TOPIC, encode_packet(4, 31))
        events = bridge.snapshot()["events"]
        self.assertEqual(len(events), 2)
        self.assertIn("Robot back online", events[1]["message"])

    def test_each_new_fault_is_logged_once(self):
        bridge = BrokerBridge()
        status = lambda phase, fault: encode_packet(0, phase | fault << 4)
        for phase, fault in ((1, 0), (12, 2), (12, 2), (0, 2), (0, 0), (12, 8), (12, 3)):
            feed(bridge, PACKED_TOPIC, status(phase, fault))
        events = bridge.snapshot()["events"]
        self.assertEqual([(e["source"], e["level"]) for e in events], [("Robot", "ERROR")] * 3)
        self.assertEqual([e["message"] for e in events],
                         ["Robot stopped: fault 2 (line)", "Robot stopped: fault 8 (lifted)",
                          "Robot stopped: fault 3 (shock)"])

    def test_no_robot_offline_while_the_dashboard_lost_the_broker(self):
        bridge = BrokerBridge()
        feed(bridge, PACKED_TOPIC, encode_packet(4, 30))
        bridge.on_disconnect(None, None, None, None, None)
        bridge.check_robot(bridge.snapshot()["last_received_at"] or 0 + 100)
        messages = [e["message"] for e in bridge.snapshot()["events"]]
        self.assertEqual(messages, ["Dashboard lost the MQTT broker; reconnecting"])
        bridge.on_disconnect(None, None, None, None, None)  # logged once
        client = SimpleNamespace(subscribe=lambda topics: (0, 1))
        bridge.on_connect(client, None, None, SimpleNamespace(is_failure=False), None)
        messages = [e["message"] for e in bridge.snapshot()["events"]]
        self.assertEqual(messages[-1], "Dashboard reconnected to the MQTT broker")
        self.assertEqual(len(messages), 2)


if __name__ == "__main__":
    unittest.main()
