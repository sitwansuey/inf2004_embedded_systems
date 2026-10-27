# Line-only 50% PWM v4 — installed baseline, 24–25 October 2026

The requested 50% PWM firmware is installed. Normal USB v4/cap=50 and final
STOP were verified. UF2 SHA256:
6131fa182021d34b65dc1c5fbdcf8bb2322f84d21908deacadababcb50058eb3

The 200 ms raised-wheel pulse completed at reported power 50; both encoders
advanced. The user confirmed both wheels forward and stopping afterward.
The first 2-second floor trial stopped after about 900 ms with LINE_AMBIGUOUS,
after both sensors reported BLACK. Both encoders advanced during left/right
corrections; no NO_PROGRESS fault occurred. Final STOP was acknowledged.
The user confirms it stayed on the sheet, swung left/right, and encountered
a turn; the course also contains a cross. The both-BLACK stop is an incomplete
course policy. Physical floor tracking remains unvalidated. Cross route: straight.

Source/tests: separate guarded 50% cap, six-bit minimum local power field,
exact 50 storage across the old 31/32 boundary; normal profiles retain 20%.
73,458 native policy assertions and ASan/UBSan pass; existing integration and
Buddy 3 suites pass; 26 in-memory host checks and 16 profile guards pass.
Full ARM build/UF2-to-ELF/source checks pass. No source change since that build.

| Measurement | Before v3 | v4 |
|---|---:|---:|
| text | 309920 | 309920 |
| data | 96 | 96 |
| bss | 25412 | 25412 |
| linked static RAM | 25508 | 25508 |
| UF2 bytes | 620032 | 620032 |
| native policy state bytes | 24 | 24 |
| binary telemetry bytes | 50 | 50 |
| USB bench frame bytes | 6 | 6 |

20-second maximum, USB lease 250 ms, motor watchdog 150 ms, encoder progress
750 ms, fresh IR 30 ms, boot STOPPED and urgent s STOP remain. Ultrasonic is
disabled. WHITE/WHITE alone cannot prove line presence. No gyro or calibrated
turn angle/distance is claimed. Calibration and motor GPIOs are unchanged.

Evidence: docs/evidence/2026-10-25/line_only_v4_50pct/INSTALLATION_RESULT.json
and original hardware/test logs. All changes are local; no push, PR or remote edit.
