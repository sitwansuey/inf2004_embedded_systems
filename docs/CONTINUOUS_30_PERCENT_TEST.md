# User-operated 20-second line test at 30% PWM

User requests a continuous 20-second run at 30% while moving the robot to
observe its response, without early stops for line patterns or encoders.
This separate IRContinuous v6 profile retains the existing IR steering:
GP27 BLACK -> left pivot, GP26 BLACK -> right pivot; both WHITE or both
BLACK -> forward. Every valid pattern can start. No course timeout or
encoder-progress timeout applies. Encoder values remain diagnostics.

Normal completion is the 20-second deadline. Explicit STOP, USB lease loss,
invalid/stale ADC, clock and actual motor faults still stop the motors.
Ultrasonic is disabled. This is a supervised sensor test, not autonomous
junction recognition or safe line-loss detection. No automatic repeating.

RUN_CONTINUOUS_20S.ps1 defaults to Inspect. -Run performs exactly one fixed
30%/20-second run, without track-layout confirmation switches. A file named
STOP_CONTINUOUS.flag in its output folder requests emergency STOP. It must
be deliberately removed before another run. Boot remains STOPPED.

v6 is installed. Normal USB identity and fresh STOP are verified. The user
completed the powered 30% test and confirms successful turning and going
straight through the cross. The saved run spans exactly 20,000 ms from first
active status to COMPLETE, with final commanded STOP acknowledged.

This is a PASS for the reported user-operated turn/cross response test.
The user planned to move/reposition the robot while testing. It does not
establish unassisted full-course tracking, repeated junction reliability,
speed/distance accuracy or braking distance. No further firmware changes.
Original logs and TURN_CROSS_VALIDATION.json are retained in the evidence folder.
Double-click tools/robot_bench/RUN_30PCT_20S.cmd after positioning and enabling
motor power. It starts one run, then returns stopped; rerun it to test again.
No track-layout prompts, no automatic repetition or restart on reconnection.

6,661 continuous-policy assertions and ASan/UBSan pass, including all valid
patterns through the full simulated 20 seconds and rejection above 30%.
Existing integration regressions pass (including separate 50%/course policies).
15 in-memory host checks and 9 profile guards pass. Full ARM build and
UF2/ELF/source comparison pass. User changes remain preserved.

Static RAM remains 25,508 bytes (data 96 + bss 25,412); text is 309,360 bytes
versus v4 309,920. UF2 is 619,008 bytes versus v4 620,032. Application binary
telemetry stays 50 bytes and USB control frames 6 bytes. The local power field
uses the minimum 5 bits for cap 30. Runtime stack/heap high-water is unmeasured.

Installed UF2 SHA256:
2deddabc20183951ddb8036240d8d1b50f5fdfe903ef4bf13a240956f018bf69
Evidence: docs/evidence/2026-10-25/continuous_v6/INSTALLATION_RESULT.json.
The separate v5 corner/cross candidate was built and tested, never installed,
and is on hold at the user's request. All work local, no push/PR/remote edits.
