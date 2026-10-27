# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-CR1-08 | Original Buddy 5 signal roles and production sonar driver | S5 GPIO/timer and sensor status | native | `s5_hardware_test.c` via integration Makefile | correct roles pass; reversed roles fail |
| INT-CR1-09 | Preempted status publisher and failure propagation | S5 snapshot/control API | native | `s5_status_test.c` | bounded wait yields, poll fails; no stale-copy success |
| INT-CR1-10 | Snapshot timestamps during bounded reads | telemetry freshness and codec | native | `snapshot_time_test.c` | age reference follows read; stale/error data unavailable |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-22 | INT-CR1-08 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_sonar_pin_fix.txt` | Production driver, old-role negative control, scan policy, real headers SMP 0/1 and lock 0/1; ARM/physical retest pending |
| 2026-10-22 | INT-CR1-09, INT-CR1-10 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_s5_status_fix.txt` | Production S5 API/telemetry task, old-poll negative control, real headers SMP 0/1 and lock 0/1; ARM/physical validation pending |
