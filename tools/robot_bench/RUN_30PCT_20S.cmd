@echo off
setlocal
echo One 20-second IR steering test at 30%% PWM.
echo Enable motor power only when positioned for your test. Ctrl+C is STOP.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0RUN_CONTINUOUS_20S.ps1" -Run
echo Test finished. Run this file again when you want another test.
pause
