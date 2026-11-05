# Barcode and MQTT demonstration

This implementation is based on `compiled` commit
`8bf9d12b7f50d218cff30600a6a1153b49e70bf8`. It decodes a single Code 39
letter framed by `*` (`*A*` through `*Z*`), including quiet white margins.
Reversing the sampled bars returns the same letter and sets the reverse flag.
The alphabet was checked against [ZXing's Code39Reader](https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java).
Existing narrow:wide 1:3 fitting, bounded optical edge compensation and
malformed/noisy/undersampled capture rejection are retained.

## Before the run

Use the motor-enabled **BarcodeDemo** build profile (`NET=lwip MQTT=1
BARCODE_DEMO=1 S2_COMMAND_TIMEOUT_MS=150`). Its reviewed configuration is
`calibration/barcode_demo30.h`:

- Motors enabled; equal 30% PWM, 100% trims, PID off; driver rejects requests
  above 30%. This carries over the working line-tracing motor setup.
- GP27 left / GP26 right; both white means assumed centred between sensors.
  GP28 is excluded from steering. Capture holds forward drive to prevent the
  earlier zigzag while crossing bars. All three input observations must remain fresh.
- Mixed input mode 3 keeps GP26/GP27 analog (white 187/189, black 3214/3177)
  and reads GP28 from the barcode module's DO/D0 output as a digital 0/1.
  Only ADC0/1 pads are initialized; GP28 is configured as GPIO input, never read
  by the ADC worker. GP29 remains radio-owned. GP28's old analog endpoints do
  not apply to this digital profile.
- Keep the barcode module at 3.3 V with common GND and move only its signal
  wire from AO/A0 to DO/D0, still ending at GP28. Verify white and black with
  motor power off. Default polarity is WHITE=0, BLACK=1; set
  `BOARD_BARCODE_DIGITAL_BLACK_LEVEL=0` if physical measurements show the inverse.
  Adjust the module's comparator screw until the actual white/black areas
  produce stable opposite logic levels at the mounted height.
- GP28 is sampled with the analog pair every 10 ms; this is GPIO polling,
  not interrupt edge capture. Telemetry's third raw value is now 0 or 1.
- Encoder turns use the previous trial's 631 ticks/rev, 220 mm wheel
  circumference and 113 mm track width. These numbers are not newly measured;
  validate 90/180-degree turn accuracy on the robot.
- Ultrasonic avoidance and terrain control are disabled for this line/barcode
  profile. Their missing readings remain unavailable in telemetry. The normal
  `Mqtt` integrated mission retains its calibration and sensor requirements.
- Motor watchdog is 150 ms. Stale ADC/encoders, controller timing gaps, STOP and
  turn timeouts still stop motion. No PID/terrain/bypass calibration flag is
  falsely marked complete.

**Automatic start:** boot stays stopped until it receives a valid MQTT mapping,
GP28 is over WHITE, and motor/ADC/encoder observations stay ready for 3 seconds.
It then starts at 30% without a START command, once per boot. It repeats barcode
and junction actions until STOP/fault. STOP cancels auto-start even during the
waiting period; neither RESET nor retained mapping delivery restarts it.
Power-cycle for another automatic run, or deliberately RESET then manually START
while stopped and over white. `BOARD_BARCODE_AUTO_START=0` selects manual start.

Configure ignored `config/wifi_secrets.h` with the actual Wi-Fi/broker settings.
The historical handoff contained only example credentials; they cannot connect
the robot. Build with `tools/build_windows.ps1 -Profile BarcodeDemo` using the
native toolchain options, or use `tools/prepare_barcode_demo.py` to run the PWM
checks and package a verified UF2. That helper refuses example Wi-Fi settings
unless `--allow-example-network` is explicitly supplied; that candidate is
labelled `NETWORK_NOT_CONFIGURED` and is not the final connected image.
Radio initialization finishes while stopped before ADC sampling starts,
avoiding the previously observed Wi-Fi firmware-upload interruption.
The older bench profiles do not run this barcode/junction mission.

On the PC, install `paho-mqtt` using the dashboard requirements. Enter the four
letters given by the instructor, with no firmware changes. For example ONLY:

```powershell
python tools/barcode_map.py --broker 192.168.1.10 --forward E --uturn M --left Q --right Z
```

The tool waits for the car's application ACK. It sends no START command, but
**applying a mapping arms the automatic-start sequence in BarcodeDemo**. Position
the car and clear its path before publishing. The normal `Mqtt` profile still
requires an explicit START. Use
`--dry-run` to inspect the payload without connecting, or `--retain` to have
the broker resend this configuration after a reboot. The map is held in RAM,
survives STOP/RESET/reconnect, and is lost when the Pico restarts. No map is
compiled into firmware. Supply broker credentials through `--username` and
the `MQTT_PASSWORD` environment variable if needed.

In manual-start mode, use the existing RESET/START controls after mapping.
START/ARM is refused until a mapping has been applied. Incoming mapping changes
are rejected while armed; stop, send a new mapping and wait for its ACK first.

## Run sequence

1. GP28 decodes A-Z in either direction. The character is copied into controller
   memory. GP26/GP27 continue tracing the line; reading a barcode does not turn
   or stop the car.
2. Both line sensors must remain black for at least 30 ms of fresh observations
   to confirm the next junction. A short black patch does not trigger a turn.
3. The received mapping selects forward, U-turn, left or right for the saved
   character. The existing encoder-yaw controller executes turns and reacquires
   the line. Forward acknowledges the marker and continues tracing.
4. The junction is latched until fresh non-junction observations establish
   clearance. Barcode capture is explicitly rearmed after the action, allowing
   another barcode/junction pair, including repeated identical characters.

BarcodeDemo sets `BOARD_BARCODE_RETRY=1`: an incomplete, noisy, overflowing,
timed-out, malformed or unknown candidate is discarded without a driving
fault. Line tracing continues, and capture rearms after a fresh white quiet
interval. A partial capture interrupted by a junction is likewise discarded
and may retry after junction clearance. A successfully decoded marker remains
latched until the controller consumes it at a junction; failed candidates do
not overwrite an already stored letter. Other profiles retain their existing
fatal barcode-error policy by default.

`[BARCODE REJECT]` logs retain the exact status/reason, captured run count,
maximum run duration and whether a junction caused the abort. They are copied
to telemetry outside the acquisition task; no formatting blocks sampling.

BarcodeDemo sets `BOARD_BARCODE_DEFAULT_FORWARD=1`: if the next confirmed
junction has no stored symbol, or its symbol is absent from the received map,
continue straight. Only a mapped letter overrides forward. Straight actions
keep forward drive active while acknowledging/rearming the scanner, without
a stop-and-restart handoff. Each junction is handled once until confirmed
clearance; subsequent missed scans likewise default forward.
`[BARCODE JUNCTION] ... default=forward reason=no_decode|unmapped` records this
choice. A no-decode fallback clears any old display symbol rather than implying
that it was read again. A new run still requires the existing MQTT mapping and
readiness gate. Other profiles retain the strict unmapped-character policy.

A second delivered barcode before consuming the first, sensor/clock faults,
stale safety data and transport errors retain their stop behavior. STOP clears
pending navigation. Valid mapped letters still use the received movements.

Place the barcode beside the tracing line, in GP28's forward path. Keep the
line under GP26/27 throughout the scan. Allow enough white travel after the
barcode for its trailing quiet-zone validation before the junction. The narrow
element is sampled by the 10 ms acquisition profile. BarcodeDemo temporarily
disables the 20 ms pulse-duration floor (`BOARD_BARCODE_MIN_RUN_MS=0`); this
accepts shorter observed pulses but cannot recover edges between samples.
The 1:3 width fitting, framing, quiet zones, timeout and freshness checks remain.
Real mounting, print dimensions, speed and turn geometry need track validation.
Two line sensors infer a cross from sustained both-black readings; they cannot
independently prove that every geometrical branch exists.

## MQTT contract

Publish UTF-8 JSON on `csc02/barcode/map`, preferably QoS 1:

```json
{"forward":"E","uturn":"M","left":"Q","right":"Z"}
```

Exactly these four keys, in any order, with four distinct uppercase A-Z values.
Payloads are length-delimited and limited to 128 bytes. Duplicate/unknown keys,
escapes, extra characters, truncated messages and duplicate letters are rejected.
The network owner copies the validated request into the bounded controller
queue. A successful publish is not confirmation that the car applied it.

Application ACKs on `csc02/barcode/map/ack` are six binary bytes:
`[0xC4, result, forward, uturn, left, right]`; letters are ASCII.
Results: 0 applied, 1 armed/busy, 2 invalid, 3 queue unavailable/full, 4 stale.
Invalid JSON uses four zero letter bytes. Subscribe before publishing and match
the returned letters. A lost ACK may be retried while stopped. Configuration does not grant the network RESET/START authorization. In the
automatic BarcodeDemo profile it starts the onboard 3-second readiness gate.

Telemetry uses `Robopico/sensors/edr16/v2`: subtype 1 has A=1 through Z=26 in
bits 4:0 and the movement in bits 7:5. Use this checkout's dashboard with this
firmware. The private snapshot remains 50 bytes (400 bits), and sensor event
records remain 14 bytes. See [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).

## Verification

```powershell
python -B testing/integration/run.py --compiler gcc
mingw32-make -C testing/subsystem3 test SHELL=C:/msys64/usr/bin/sh.exe
python -B testing/data_representation/run.py --compiler gcc
python -B -m unittest discover -s testing/barcode -v
```

The tests cover all 26 letters in both directions and multiple scales, full
sampled captures, memory retention, delayed junction execution, repeated markers,
all four runtime movements, malformed mappings, MQTT fragmentation, ACKs,
queue overflow, STOP invalidation and timer wrap. They do not establish physical
scan reliability or live broker interoperability. No tests flash or start a robot.
