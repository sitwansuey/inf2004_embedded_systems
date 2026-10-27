# Buddy 4 physical validation guide

The checkpoint and CR1 continuation have separate software/build evidence in
testing/subsystem4/verification-20261027.json and cr1-verification.json.
Physical tests below have NOT been run. Work through one stage at a time and
save the evidence before enabling the next capability. The smallest next test
is stage 2: the isolated sensor image, with motor power disconnected.

## Updated final-ramp scope

Professor's clarification supplied on 27 October 2026: the final ramp
will be approximately the height of the smaller practice ramp, slightly lower
or higher. The higher practice ramp may be disregarded for design and testing.
The objective remains estimating the height reached by the robot through IMU
sensing and processing, rather than solving the higher ramp's clearance issues.

Use the smaller practice ramp as the primary physical reference and, where
available, modest height variations around it. Measure its height, slopes and
profile; no numerical final-height bound or accuracy tolerance was supplied.
Do not hard-code the known ramp height as the estimate or narrow CR1 height
fields on the basis of "approximately" alone. Current IMU-derived pitch plus
encoder-travel processing remains an estimator to validate; the clarification
does not explicitly address whether encoder assistance is permitted.

## 1 Hardware free checks

Run from the repository root. Native GCC and MSYS2 are already installed here.

```powershell
& C:\msys64\ucrt64\bin\python.exe -B testing/data_representation/run.py --report edr-check-report.json
& C:\msys64\usr\bin\bash.exe -c 'export PATH=/ucrt64/bin:/usr/bin:$PATH; cd /c/Users/JT/Downloads/inf2004_embedded_systems; make -C app_program/subsystem4/tests/host test sizes && make -C testing/integration test'
```

Both suites must exit successfully. Tests use explicitly synthetic wheel and
IMU values; they do not supply your robot's calibration. For static analysis,
run the same bash command with `make -C app_program/subsystem4/tests/host analyze`.
The `sanitize` target requires a GCC/Clang installation with ASan/UBSan runtimes;
the installed Windows GCC cannot link those libraries.

## 2 Isolated IMU acquisition with motors disabled

Use the robot Pico W. Disconnect the motor supply and secure the chassis. Check
common ground, 3.3 V logic, GP2 SDA and GP3 SCL, and confirm the actual module
power requirements. I2C1 is owned only by S4. The source expects accelerometer
0x19 and magnetometer 0x1E. Do not use the old IRContinuous motion profile.

The extension already installed the following compiler and SDK. Build in
PowerShell, without WSL or any downloads:

```powershell
.\tools\build_windows.ps1 -Backend Native `
  -ArmToolchainBin C:\Users\JT\.pico-sdk\toolchain\15_2_Rel1\bin `
  -PicoSdkPath C:\Users\JT\.pico-sdk\sdk\2.3.0 -Profile S4Bench
```

Output: `build_make/mtk3pico_smp1_usb_cdc_wifi_enc_s4.uf2`. This startup starts
only the sensor/monitor application tasks, not the motor/controller tasks. It
still includes kernel USB/radio service code. Compilation alone is not flashing.
When ready for the physical session, hold BOOTSEL while connecting the robot,
then copy only this identified UF2 to its RPI-RP2 drive. Record its SHA256 first:

```powershell
Get-FileHash build_make/mtk3pico_smp1_usb_cdc_wifi_enc_s4.uf2 -Algorithm SHA256
```

Open the robot's actual COM port at 115200 using VS Code Serial Monitor. Capture
at least 30 seconds flat, then several steady orientations. `[S4]` repeats its
CSV heading. `init=0`, `a_ok=1`, `m_ok=1`, increasing sample counters and heartbeat,
bounded ages and no increasing I/O errors support successful acquisition.
Raw/scaled acceleration is sensor XYZ; flat with Z up should be near
`ax=0, ay=0, az=1000 mg`, allowing real offsets. Magnetometer mG changes with
orientation and nearby steel; this is not a calibrated compass test.

Pass: both channels produce fresh records, and gravity shifts between axes as
you hold known orientations. Fail: wrong identification, missing channel,
frozen counter, repeated I/O errors or persistently unusable records. Save raw
logs, wiring/axis photo, firmware hash and the held orientations. Do not invent
an accuracy tolerance; record measured deviations against your reference jig.

For missing-sensor behavior, power off before disconnecting the module. Boot
again with motor supply still disconnected. Pass: heartbeat/USB remains alive,
usable flags are false and no frozen numbers are reported as fresh. Reconnect
only with power off and verify recovery. Independent channel error recovery is
covered in software tests; do not damage the board trying to isolate one chip.

## 3 Establish mounting and tilt zero

Rigidly mount the board with sensor X forward, Y left, Z up. Confirm these are
the chip axes, not assumptions from connector position. Save a photo.

Keep `BOARD_MOTORS_DISABLED=1`. Copy
`app_program/robot_config.local.example.h` to ignored `robot_config.local.h`
only after inspecting it. Enter measured values; placeholders remain zero.
For the stationary flat mean acceleration `(ax,ay,az)` in mg, the implemented
raw angle formulas are:

```text
pitch_zero_cdeg = round(atan2(-ax, sqrt(ay*ay + az*az)) * 18000/pi)
roll_zero_cdeg  = round(atan2(ay, az) * 18000/pi)
```

After mounting/axis and offset checks, set `BOARD_TERRAIN_CALIBRATED=1` and the
two measured zero offsets, then rebuild S4Bench. `[S4TILT]` gives centidegrees:
100 = 1 degree. Before calibration it must show `valid=0`. Hold flat and known
positive/negative pitch angles using a jig. Positive pitch means the sensor X
acceleration component is negative. Hold roll within 15 degrees for usable
control tilt; larger roll is intentionally rejected by the present threshold.
Record reference angle, output, error and settle time. Dynamic movement can
invalidate gravity tilt; test steady holds first. There is no gyro output.

## 4 Encoder calibration with motor power still disconnected

Build `-Profile Default` using the same command. Keep the local motor lock at 1.
Default starts the integrated acquisition/controller/telemetry tasks but will
not authorize movement. `[S2ENC]` and `[S4TERRAIN]` are passive diagnostics.

Raise both wheels. Rotate each wheel manually in the physical forward direction
and confirm its forward count increases: motor 1 is left, motor 2 right. Confirm
reverse decreases it and the other wheel stays still. Then measure x1 ticks
over several marked complete wheel revolutions; divide the count difference
by revolutions. Do not substitute advertised quadrature x4 counts.

Measure loaded wheel circumference over several straight rolling revolutions,
and track width between wheel contact centre lines. Enter
`BOARD_TICKS_PER_REV`, `BOARD_WHEEL_CIRCUMFERENCE_MM` and
`BOARD_TRACK_WIDTH_MM`. If wheels need different scales, the present common
scale is insufficient; resolve that before claiming distance accuracy.
Rebuild Default. Pass: manually rolling a measured straight distance gives
consistent encoder progress and plausible calibrated distance/speed. Record
raw counts and the tape-measure distance; PWM is never a distance measurement.

## 5 Hump height and maximum using manual forward rolling

Keep motor power disconnected and the firmware motor lock enabled. This allows
the actual IMU/encoder/controller path to be checked without autonomous motion.
Use the smaller practice ramp with measured height and slopes. Check its
profile against the current 5-degree slope-entry and 3-degree flat thresholds;
if detection is unsuitable, review thresholds using measured noise and real
ramp data rather than selecting a steeper ramp to satisfy existing settings.
Roll slowly so motion remains gravity dominated.
First establish at least five fresh flat samples, then traverse the whole hump
forward, returning to the original base level. Avoid reversing either wheel.

Watch `[S4TERRAIN]`: state codes are 0 flat, 1 ascent, 2 crest, 3 descent,
4 shock, 5 invalid. A completed traversal should yield `height_valid=1`, a
positive `height_mm` and `max_mm` at least that height. Compare with the measured
height; record the error and repeatability. Software accepts a base-return
residual within 20 mm and minimum hump 5 mm; these are initial configurable
settings, not evidence of 20 mm accuracy. Serial diagnostics are about 1 Hz and
can miss short intermediate states; use timestamped instrumentation/video for
state timing claims.

For the smaller ramp, explicitly assess the 20 mm return tolerance and 5 mm
minimum hump against measured height, baseline noise and repeatability. They
are provisional algorithm settings, not the professor's acceptance criteria.
Define the measured reference consistently as rise above the initial flat
baseline; record robot/IMU geometry when comparing with the ramp's surface
height. Estimation should respond to changed heights, not recognize one
pre-entered answer.

Run several complete humps, including a smaller one after a larger one. Pass:
the maximum increases only for a larger valid completed hump. Then stop halfway,
reverse, introduce an ordinary sample loss, or reboot with the sensor missing
as separate controlled tests. Interrupted traversals must not create a new
completion. Fault/recovery or pause must retain the previous maximum within
the same powered run; resuming midway up a slope must not count a partial hump.
Return to flat before retrying. Reboot starts a new in-memory run and clears it.
Avoid deliberate impacts; shock logic has synthetic tests and can be assessed
using controlled fixtures if needed.

To test explicit RESET, use the integrated Default profile while stopped and
disarmed: send ASCII `s`, then `r` in Serial Monitor. Pass: maximum returns to
zero and interval/terrain history starts again. Sending ARM or pausing must not
clear it. RESET is not supported by the isolated S4-only application.

## 6 Controller STOP and binary dashboard integration

Only after stages 2–5 pass should a separately supervised powered-motor test be
planned with B2. Begin with raised wheels, explicit drive permission, calibrated
IR/sonar and all controller prerequisites. Default stays disarmed until ARM;
the sensor-only image cannot validate motor response. Do not use a short USB
ARM command as a generic Buddy 4 test: it requests integrated line tracking.

Verify on the actual robot that bad IMU/odometry input, stale data and explicit
STOP produce zero PWM and invalidate old queued movement. Measure source
confirmation → controller observation → PWM off → physical stopping separately.
The report's 50 ms confirmed-event target requires measured evidence under
concurrent load. A 10 ms controller tick or passing simulated test is not that
measurement. Retain logs and an instrument/video trace with an identified clock.

The dashboard uses the existing 50-byte aggregate format on `csc02/telemetry`.
For MQTT testing, use `-Profile Mqtt` after supplying your own ignored
`config/wifi_secrets.h` and the configured broker; keep motors locked for data
checks. The app publishes through the network-owning task. Install the dashboard
requirements in an isolated environment, then run:

```powershell
python app_program/external_systems/Dashboard/dashboard.py --broker YOUR_BROKER
```

Open `http://127.0.0.1:8001`. Pass: S4 values/units agree with console logs;
uncalibrated or stale tilt/height show unavailable; a retained maximum remains
visible; a malformed packet is rejected; disconnect marks data historical.
Save binary payloads including embedded zero bytes and screenshots. Software
agreement is already checked by 302 C/Python vectors; real MQTT delivery,
queue pressure, latency and physical fault timing remain physical evidence.

## Evidence to retain

For each stage save firmware SHA256, profile/configuration without credentials,
date/time, wiring/axis photo, raw timestamped console log, measured reference
values and pass/fail conclusion. Keep expected thresholds separate from actual
measurements. Buddy 4 owns sensor/terrain evidence; wheel calibration and motor
timing are joint work with Buddy 2, while MQTT/dashboard delivery is Buddy 1's
boundary. The original 30% IRContinuous test does not validate these stages.
