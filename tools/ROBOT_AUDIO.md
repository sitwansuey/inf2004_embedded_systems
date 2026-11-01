# Raspberry Pi audio warnings

## Separate audio Raspberry Pi

The audio Raspberry Pi, dashboard computer and RoboPico are separate devices.
RoboPico publishes raw EDR16 sensor records to the MQTT broker. The dashboard
and audio Raspberry Pi independently subscribe to that same stream:

```text
RoboPico -> MQTT broker -> dashboard computer (display)
                       -> audio Raspberry Pi (decode, detect, play)
```

The audio Pi does not require the dashboard to run. It receives raw two-byte
telemetry, not audio commands or audio files over MQTT. This script decodes
the records locally and plays speech or local WAV recordings. Set `--broker`
to the shared broker's network address, not automatically the dashboard or Pi
address. If the broker runs on the dashboard computer, that computer's broker
service must remain running even when the dashboard application is closed.

Run `tools/robot_audio.py` on a Raspberry Pi running Raspberry Pi OS with a connected speaker and working default audio output. The Pico continues publishing telemetry; this script only subscribes and never sends driving commands. Keep `robot_audio.py` beside the repository's `edr_decode.py` if copying individual files.

From the repository directory on the Pi:

```bash
sudo apt update
sudo apt install -y python3-venv espeak-ng alsa-utils
python3 -m venv .venv-audio
source .venv-audio/bin/activate
python -m pip install -r tools/robot_audio_requirements.txt
python tools/robot_audio.py --test-audio
python tools/robot_audio.py --broker YOUR_BROKER_IP
```

Replace `YOUR_BROKER_IP` with the broker used by the dashboard and Pico. Use `localhost` only when the broker runs on this Pi. Stop with Ctrl+C. On later runs, activate the virtual environment before launching the script.

## Detection and wire format

The listener subscribes only to `Robopico/sensors/edr16/v1`. It uses the shared decoder for each exact two-byte, little-endian unsigned 16-bit record: the bottom four bits identify the sensor and the upper twelve bits carry its data. No separate sensor topics are needed.

- **Obstacle:** ID 4 reports a valid distance of 20 cm or less. The warning clears above 25 cm to avoid rapid toggling near the threshold. Zero/no-echo readings are ignored. It announces “Warning. Obstacle detected.” This applies to any ultrasonic scan direction; telemetry does not associate each distance with its actual sampling angle.
- **Off course:** the firmware's calibrated line status (ID 0, subtype 2) says searching/lost and neither front IR sensor's black-mask bit is set. Fresh raw readings for both IDs 5 and 6 are also required. The barcode sensor is ignored, and raw ADC thresholds are not guessed. It announces “Warning. Off course. No line detected by the front sensors.”
- Line warnings normally apply during TRACK, REACQUIRE, or a controller LINE fault. Intentional turns, bypass manoeuvres and the disarmed state suppress them. Use `--all-phases` for a bench test that also warns while parked.
- Availability status (ID 0, subtype 10) must say the relevant sensors are usable. These status records require the updated project firmware.

Each warning requires at least two distinct observations and a 0.3-second confirmation period. Warnings repeat at most once every five seconds per condition. Playback is sequential and does not block MQTT reception. Recordings are limited to 15 seconds; longer playback is stopped.

Telemetry older than one second, retained messages, malformed packets and other topics do not trigger warnings. Disconnect/reconnect clears the observations and requires fresh data again. Freshness is measured from arrival at the Pi: EDR16 does not contain source timestamps or atomic frame identifiers. This is an audio indicator; the firmware remains responsible for vehicle control.

## Options

Log detections without a speaker:

```bash
python tools/robot_audio.py --broker YOUR_BROKER_IP --dry-run --all-phases
```

Use your own WAV recordings:

```bash
python tools/robot_audio.py --broker YOUR_BROKER_IP \
  --obstacle-wav /home/pi/audio/obstacle.wav \
  --off-course-wav /home/pi/audio/off_course.wav
```

An omitted recording uses speech instead. `--audio-device plughw:1,0` selects an ALSA device and requires both WAV recordings; speech uses the OS default output. Select and test the Pi's speaker output before starting the listener.

Tune `--obstacle-cm`, `--clear-cm`, `--confirm-seconds`, `--repeat-seconds` and `--stale-seconds` if needed. Changing these affects audio only, not firmware avoidance thresholds. Run `--help` for all arguments.

For authenticated brokers, set `MQTT_USERNAME` and `MQTT_PASSWORD` in the environment. For TLS use `--tls --port 8883`, optionally with `--ca-cert /path/to/ca.pem`; certificate verification stays enabled.

## Verification

```bash
python -B -m unittest discover -s testing/audio -v
python -B tools/robot_audio.py --dry-run --test-audio
```

Tests exercise decoding, detection, stale/missing data, phase gating, cooldown and mocked audio playback. Actual Pi speaker playback and live broker delivery must be checked on your hardware.

References: [Paho Python MQTT documentation](https://eclipse.dev/paho/clients/python/docs/) and [Raspberry Pi OS documentation](https://www.raspberrypi.com/documentation/computers/os.html).
