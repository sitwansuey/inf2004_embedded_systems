import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from robot_audio import Monitor, Audio, MQTT_TOPIC, connect_client


def packet(sensor, value):
    return ((value << 4) | sensor).to_bytes(2, 'little')


class MonitorTests(unittest.TestCase):
    def send(self, monitor, now, distance=10, mask=0, state=3, phase=1,
             available=20, right=True):
        records = [(0, 0xA00 | available), (0, phase),
                   (0, 0x200 | (mask << 3) | state), (4, distance), (5, 4000)]
        if right:
            records.append((6, 4000))
        for sensor, value in records:
            monitor.receive(MQTT_TOPIC, packet(sensor, value), now)

    def activate(self, monitor, **kwargs):
        self.send(monitor, 0, **kwargs)
        self.assertEqual(monitor.ready(0), [])
        self.send(monitor, .4, **kwargs)
        return monitor.ready(.4)

    def test_both_warnings_and_cooldown_fairness(self):
        m = Monitor()
        self.assertEqual(self.activate(m), ['obstacle', 'off_course'])
        m.announced('obstacle', .4)
        self.assertEqual(m.ready(.5), ['off_course'])
        m.announced('off_course', .5)
        self.assertEqual(m.ready(.6), [])
        self.send(m, 5.6)
        self.assertEqual(m.ready(5.6), ['obstacle', 'off_course'])

    def test_single_sample_never_confirms_and_stale_suppresses(self):
        m = Monitor()
        self.send(m, 0)
        self.assertEqual(m.ready(0), [])
        self.assertEqual(m.ready(.5), [])
        self.assertEqual(m.ready(1.1), [])
        self.assertEqual(m.values['distance'][0], 10)

    def test_obstacle_hysteresis_zero_and_availability(self):
        m = Monitor()
        self.assertIn('obstacle', self.activate(m, distance=20))
        self.send(m, .5, distance=25)
        self.assertIn('obstacle', m.ready(.5))
        self.send(m, .6, distance=26)
        self.assertNotIn('obstacle', m.ready(.6))
        for distance, available in [(0, 20), (10, 4), (21, 20)]:
            with self.subTest(distance=distance, available=available):
                self.assertNotIn('obstacle', self.activate(Monitor(), distance=distance, available=available))

    def test_front_mask_ignores_barcode_and_raw_thresholds(self):
        self.assertIn('off_course', self.activate(Monitor(), mask=4))
        for mask in (1, 2, 3):
            self.assertNotIn('off_course', self.activate(Monitor(), mask=mask))
        for state in (0, 1, 4):
            self.assertNotIn('off_course', self.activate(Monitor(), state=state))

    def test_line_requires_both_sensors_and_availability(self):
        self.assertNotIn('off_course', self.activate(Monitor(), right=False))
        self.assertNotIn('off_course', self.activate(Monitor(), available=16))

    def test_phase_gating(self):
        for phase in (0, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12):
            self.assertNotIn('off_course', self.activate(Monitor(), phase=phase))
        for phase in (1, 10, 44):
            self.assertIn('off_course', self.activate(Monitor(), phase=phase))
        self.assertIn('off_course', self.activate(Monitor(all_phases=True), phase=0))

    def test_malformed_retained_and_reset(self):
        m = Monitor()
        self.assertFalse(m.receive('other', packet(4, 10), 0))
        self.assertFalse(m.receive(MQTT_TOPIC, packet(4, 10), 0, retained=True))
        for payload in (b'', b'123', packet(0, 0x207), packet(0, 13)):
            self.assertFalse(m.receive(MQTT_TOPIC, payload, 0))
        self.assertEqual(m.ready(0), [])
        self.activate(m)
        m.reset()
        self.assertEqual(m.ready(.5), [])


class AudioTests(unittest.TestCase):
    @patch('robot_audio.subprocess.Popen')
    @patch('robot_audio.shutil.which', side_effect=lambda name: name)
    def test_speech_nonblocking(self, which, popen):
        popen.return_value.poll.return_value = None
        a = Audio()
        a.play('obstacle')
        self.assertEqual(popen.call_args.args[0], ['espeak-ng', '-s', '155', 'Warning. Obstacle detected.'])
        self.assertNotIn('shell', popen.call_args.kwargs)
        self.assertTrue(a.busy())
        a.close()
        popen.return_value.terminate.assert_called_once()

    @patch('robot_audio.subprocess.Popen')
    def test_dry_run_never_starts_player(self, popen):
        a = Audio(dry_run=True)
        a.play('off_course')
        self.assertFalse(a.busy())
        popen.assert_not_called()

    @patch('robot_audio.subprocess.Popen')
    @patch('robot_audio.shutil.which', side_effect=lambda name: name)
    def test_wav_device(self, which, popen):
        wav = Mock()
        wav.is_file.return_value = True
        wav.resolve.return_value = '/tmp/warning.wav'
        a = Audio(wav, wav, 'plughw:1,0')
        a.play('off_course')
        self.assertEqual(popen.call_args.args[0], ['aplay', '-q', '-D', 'plughw:1,0', '/tmp/warning.wav'])


class ConnectionTests(unittest.TestCase):
    def test_subscription_reconnect_and_retained_handling(self):
        mqtt = Mock()
        mqtt.MQTT_ERR_SUCCESS = 0
        client = mqtt.Client.return_value
        client.subscribe.return_value = (0, 1)
        args = Mock(username=None, tls=False, broker='test-broker', port=1883)
        monitor = Monitor()
        package = Mock(client=mqtt)
        with patch.dict(sys.modules, {'paho': Mock(mqtt=package), 'paho.mqtt': package,
                                     'paho.mqtt.client': mqtt}):
            self.assertIs(connect_client(monitor, args), client)
        client.connect_async.assert_called_once_with('test-broker', 1883, keepalive=30)
        success = Mock(is_failure=False)
        client.on_connect(client, None, None, success, None)
        client.subscribe.assert_called_with(MQTT_TOPIC, qos=0)
        message = Mock(topic=MQTT_TOPIC, payload=packet(4, 10), retain=True)
        client.on_message(client, None, message)
        self.assertEqual(monitor.values, {})
        message.retain = False
        client.on_message(client, None, message)
        self.assertEqual(monitor.values['distance'][0], 10)
        client.on_disconnect(client, None, None, success, None)
        self.assertEqual(monitor.values, {})
        client.on_connect(client, None, None, success, None)
        self.assertEqual(client.subscribe.call_count, 2)


if __name__ == '__main__':
    unittest.main()
