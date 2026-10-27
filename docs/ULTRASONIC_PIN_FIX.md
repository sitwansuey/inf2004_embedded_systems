# Restore the existing Buddy 5 ultrasonic pin roles

On 22 October 2026 the user reported that the HC-SR04 previously worked using
the existing Grove cable on GP0/GP1/3V3/GND. The current GitHub branch heads were
checked read-only through the GitHub connector and match the local remote refs:

- `main`: `210a407e74fd1538b9494539480d6328b5434941`; S5 is an empty template.
- `subsystem5`: `5d8cf50a44803f680271f3c3c5ed28ad8072c878`; its
  `app_program/app.h` lines 36-38 define ECHO=GP0 and TRIG=GP1.

The integrated configuration had reversed these roles to TRIG=GP0/ECHO=GP1,
matching the submitted Week 6 table but conflicting with Buddy 5's code. This
is a confirmed integration mismatch and a plausible cause of the observed
S5_ECHO_NO_RISE. Restore the implemented mapping; do not change wiring or
supply voltage for the comparison.

The timer initialization, 10 microsecond trigger, 25 ms timeout, GPIO direction
setup and pulse conversion are unchanged from Buddy 5. Servo centring is still
1500 microseconds. The current build retains its motor lock, monotonic uptime,
safe narrow sample conversion, passive diagnostics and fail-closed Echo policy.
The S5 startup banner now prints the actual configured pin roles.

`testing/integration/s5_hardware_test.c` exercises the production hardware
driver through simulated timer/GPIO boundaries. The fixture treats physical
GP1 as Trigger and GP0 as Echo independently of the configuration: reversing
the production roles must fail. It also covers valid returns, missing/stuck/
short pulses, timer wrap, timer start failure and narrow status publication.
These are simulated checks, not measurements of the robot's electrical pins.

Native validation passed on 22 October: production driver, negative control
with the old reversed pin macros, obstacle scan regression, and real kernel
header checks for SMP 0/1 with motor lock 0/1. PowerShell parsing, sonar log
classification and old-image rejection checks also passed. The default source
still has BOARD_MOTORS_DISABLED=1. See the integration test log for native
commands/results; lock=0 was only a header syntax check, never flashed.

The corrected firmware was ARM built and flashed at 04:33:31 +08:00 on
22 October 2026. A 30-second capture returned 12 fresh valid ranges, two
no-fall results and no no-rise results among the 14 successful status copies.
Sixteen copies were unavailable. This restores basic echo acquisition with
the existing wiring, not calibrated distance accuracy. The image SHA256 is
15644919870cd0cc8b775f6e24aa6c4a74d371e7fef8a6deae318806401fd00b;
static RAM remains 26008 bytes and binary telemetry remains 50 bytes.

An additional local status-copy correction is now pending build/flash; see
S5_STATUS_COPY_FIX.md. Keep the drive lock on. Electrical supply and Echo
voltage have not been measured; earlier undervoltage comments were hypotheses
rather than a demonstrated cause. A standard 5 V HC-SR04 must not be supplied
from 5 V with a direct Echo connection to the Pico.

Only local files change; no GitHub branch, commit or pull request is modified.
