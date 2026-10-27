# Wiring from the Week 6 submission and pre-tribunal report

Sources read on 22 October 2026: submitted `TeamCSC02.pdf`, page 4, Hardware
Integration Pin Layout; Week 6 Word draft `ES (1).docx`, section 2; and
`CSC02_PreTri1.docx`, architecture defence/resource ownership/assumption tables.
The PDF page was rendered and visually inspected. The embedded Week 6 diagrams
are software/state diagrams and add no measured electrical wiring evidence.

| Module | Submitted Week 6 map | Current code |
|---|---|---|
| Three IR sensors | A0/SIG; analogue ADC; GP26, GP27, GP28 | Same pins; acquisition deliberately disabled pending calibration |
| Motor 1 / Motor 2 PWM | GP8/9 / GP10/11 | Same; drive lock enabled |
| IMU SDA / SCL | GP2 / GP3; accel + magnetometer, no gyro | Same, owned by Buddy 4 I2C1 task; basic hardware acquisition verified |
| Ultrasonic trigger / echo | GP0 / GP1 | GP1 / GP0, restored from Buddy 5's implementation after the user reported this wiring worked previously; see ULTRASONIC_PIN_FIX.md |
| Ultrasonic servo | GP12 | Same; PWM slice 6 |
| Encoder pairs | Motor 1 GP4/5, Motor 2 GP16/17, both labelled TBC | Buddy 2 driver assigns Motor 1 GP16/17, Motor 2 GP4/5 |
| Wi-Fi | Onboard Pico W | Existing CYW43 owner; tested firmware NET=none, MQTT=0 |

The submitted PDF explicitly describes analogue IR. The older Word draft
instead says DO/SIG in one cell despite describing ADC analogue inputs. Use
the submitted PDF as the intended pin/mode map. Measured black/white endpoints,
sensor spacing, barcode sensor placement and physical jumper/output checks
are not supplied by that table. The pre-tribunal document still lists AO/DO,
GP26 Grove conflicts and the GP28 VBAT jumper as unclosed validation items.
No calibration values were invented or IR availability falsely asserted.

Do not silently reverse the encoder driver based on a TBC table. The next test
records signed counts while the operator manually turns left then right wheels
with motor power off. This establishes which physical wheel drives each input
pair before odometry calibration or movement is enabled.

Added one-hertz passive console diagnostics while BOARD_MOTORS_DISABLED=1:
S2ENC reports the existing encoder snapshot; S5RAW reports snapshot-copy status,
time, scan state, freshness, raw validity/range, angles and echo result. Echo
codes are 0 OK, 1 stuck-high, 2 no-rise, 3 no-fall and 4 too-short. These read
the existing APIs; there is no new GPIO/PWM writer, bus owner or motor command.
Binary MQTT remains 50 bytes with the same schema and no new application state.

The reports do not establish actual ultrasonic power/echo levels or simultaneous
load stability; pre-tribunal assumption A5 remains open. The latest 30-second
test showed no usable range. The new readout distinguishes stale/failed copies
from no echo without guessing the cause. Servo operation remains possible.

Follow-up: the 60-second diagnostic capture was completed with the old reversed
pin configuration and reported no Echo rising edge in every obtained sample.
The user reports an HC-SR04 connected by a plain Grove cable to GP0/GP1/3V3/GND
and previously working. GitHub's subsystem5 branch defines TRIG=GP1, ECHO=GP0;
the integration had reversed these roles to match the report. The software
defaults now preserve the implemented wiring. Leave physical wiring/power
unchanged for this comparison. A new ARM build/flash and range capture are
required before claiming the correction restores operation. The earlier
undervoltage explanation was only a hypothesis; electrical limits still apply.

Native real-kernel header checks passed for SMP 0/1 and motor lock 0/1. The
diagnostic addition was ARM built/flashed at 04:05 and the pin correction at
04:33 on 22 October. The latter 30-second capture restored basic ranging:
12 fresh valid ranges, zero no-rise results, two no-fall results and 16 failed
status copies. Accuracy and physical left/right encoder mapping remain open.

The new bounded status-copy/freshness correction is only in local source;
see S5_STATUS_COPY_FIX.md. Its ARM build and physical validation are pending.
Use RETEST_SENSOR_STATUS_LOCKED.ps1 for the next fixed-target capture. Keep
the existing Grove wiring and 3V3 supply unchanged, drive lock on, and chassis,
wheels and target still. The helper sends no commands.
