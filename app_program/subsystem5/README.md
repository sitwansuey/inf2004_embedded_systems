# Subsystem 5: ultrasonic ranger on a servo

S5 owns the HC-SR04 ultrasonic ranger and the servo that pans it. It
watches the distance ahead, and when an obstacle is confirmed it scans both
edges with the robot stopped, measures the obstacle's width and plans the
bypass (side, turn angle and leg lengths). The vehicle controller (VC)
drives the bypass; S5 only reports and points the sensor where the VC asks.
See [TEAM.md](../../TEAM.md) for the owner, pins, peripherals, task
priorities, interfaces IF-06, IF-11 and IF-12, and the kernel object budget.

## Hardware

| Pin | Use |
|---|---|
| GP0 | Ultrasonic ECHO input |
| GP1 | Ultrasonic TRIG output (10 µs pulse) |
| GP12 | Servo, PWM slice 6 A, 50 Hz, 750–2250 µs pulse; GP13 (slice 6 B) must not be used as PWM |
| GP22 | Buzzer, only in the acquisition diagnostic builds below |

Echo times use the kernel's 1 MHz timer; a missing echo times out after
25 ms. The servo centre pulse and slope are `BOARD_SERVO_CENTER_US` and
`BOARD_SERVO_US_PER_90` in `../robot_config.h`. Positive angles point left.

GP0 and GP1 are UART0. S5 is built only when UART0 is free
(`BOARD_UART0_FREE`, that is `CONSOLE=usb_cdc` and `CRASH_REPORT=0`). In the
`CONSOLE=uart` and `CRASH_REPORT=1` builds `s5_disabled.c` replaces it: every
call returns `E_NOSPT`, and an armed robot stops with fault 1 (sonar).

## Task and timing

| Task | Priority, core | Job |
|---|---|---|
| `obstacle scan` (`subsystem5.c`) | 20, core 1 | The only user of the ranger and the servo PWM; samples, scans and publishes the status |

S5 samples about every 65 ms and waits 150 ms for the servo to settle
before a reading at a new angle counts. Two forward echoes in a row within
200 mm confirm an obstacle (`S5_OBSTACLE_MM`, `S5_ACQUISITION_SAMPLES`).
S5 is not watched by the watchdog supervisor (TEAM.md section 7).

## Scan and plan

- **Acquisition** (`s5_scan.c`): the left side is checked at 20, 40 and
  60 degrees until one is clear, then the edge is bisected to 1 degree; the
  right side starts at the mirrored angle and is searched the same way. If
  the robot moves by more than 3 encoder counts during a sample
  (`s2_encoder_get_status()`, IF-06), the scan recentres and retries.
- **Width** (`s5_width.c`): the edge angles and distances give the
  obstacle's width and its extent left and right of the forward axis.
  `s5_width_reported()` subtracts the beam spread (`S5_BEAM_HALF_DEG`) for
  the width shown to the user; the plan keeps the measured edges.
- **Plan** (`s5_policy.c`): the side with the smaller half-width is chosen
  unless its corridor is blocked, and the turn, diagonal and return legs are
  computed before the status reports READY.
- **Status** (`s5_data.c`): the status is stored as an explicit 92-byte
  packed record, never as the native structure.

## Interfaces

| ID | Direction | API |
|---|---|---|
| IF-06 | S2 → S5 | `s2_encoder_get_status()`: wheel counts before and after each echo |
| IF-11 | VC → S5 | `s5_controller_bind()`, `s5_navigation_set()`: scan id, navigation state (MONITOR, APPROACH, OVERCOME, REORG) and look angle −90..90° |
| IF-12 | S5 → VC, S1 | `s5_get_status()` under the priority-inheritance mutex `status_mutex`; `s5_width_reported()` |

Only the bound controller task may set the navigation state. After a new
look angle, the old reading stays invalid until the servo has settled.

## Diagnostic builds

`TEST=1` builds with one of `APP_S5_MASTER_TEST`, `APP_ACQUISITION_TEST`,
`APP_ACQUISITION_LEFT_TEST` or `APP_S5_SERVO_TEST` set to 1 (for example in
the git-ignored `app_program/robot_config.local.h`) replace the scan task
with a diagnostic task (20, core 1) and do not start the VC. That task
sends S2 movement commands itself: the acquisition tests drive forward at
30 % to 15 cm and scan, and the master test then drives the planned turn,
diagonal and turn back open-loop. All but the servo test use the GP22
buzzer. Without `TEST=1` these settings
stop the build with an error.

## Verification

From the repository root with GCC, GNU Make and Python 3:

```sh
python -B -m unittest discover -s testing/data_representation -p test_s5.py -v
make -C testing/integration test
```

The first runs the status codec checks (S5-U01). The integration suite
includes the scan, edge-search, open-space, motion-retry, driver and
status-API recipes (S5-U02 to S5-U07) together with the VC tests that use
S5. Test cases, results and saved logs are in
[testing/subsystem5/](../../testing/subsystem5/TEST_LOG.md).
