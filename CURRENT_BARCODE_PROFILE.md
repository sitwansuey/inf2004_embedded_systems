# Restored barcode course checkpoint — 5 November 2026

This branch preserves the source snapshot used for the firmware restored to
the robot after the IRQ-capture experiment. It is based on `compiled` commit
`8bf9d12b7f50d218cff30600a6a1153b49e70bf8`.

The installed UF2 has SHA256
`908fd1c900b721779538f5decabaeff17b01f4d2f4efb92684ff365856fd1dc7`.
The UF2 and local network credentials are kept outside Git. Rebuilding with
different credentials or toolchains can change the binary hash.

## Active configuration

- Build the **course PI** variant, using `--course-pi` below. The plain
  `BarcodeDemo` profile is the earlier open-loop variant.
- GP26/GP27 remain analog line inputs. GP28 uses the module's DO output,
  white=0 and black=1, sampled every 10 ms. There is no IRQ barcode capture
  in this checkpoint.
- Dashboard Start requests **20%**. The motor driver retains its **30% PWM
  ceiling**, with fractional wheel PI output, gains 20/10/0 in milli units,
  and forward target factors left=1000/right=970.
- Gentle analog front-sensor correction remains enabled during active
  barcode capture. Ordinary line following resumes outside capture.
- A–Z Code 39 decoding accepts both scan directions. Framing, joint bounded
  width/edge/gap fitting and up to eight leading pulse-pair resynchronisations
  are retained. The pulse-duration minimum is zero.
- Barcode failures retry after white. Missing, failed or unmapped barcode
  instructions default to forward at a junction. The four letter/action
  assignments must arrive over MQTT; they are not fixed in the firmware.
- Wi-Fi telemetry includes retained scan results, pulse widths, rejection
  reasons and counters. Dashboard Reset sends STOP, restores its PC-side
  saved mapping, then resets the robot. Start remains an explicit button.

Firmware boot auto-start is still enabled: a received mapping plus white and
fresh inputs for three seconds can start a **30%** run once per boot. STOP
cancels that sequence. The dashboard's STOP/map/Reset workflow cancels it
before a manual 20% Start. Keep motor power off during flashing/setup.

## Build and use

Copy `config/wifi_secrets.example.h` to ignored `config/wifi_secrets.h` and
configure your Wi-Fi and raw TCP MQTT broker. This robot's PC broker used
port **8084**; set its current host address locally. Do not commit credentials.

```powershell
python -B tools/prepare_barcode_demo.py --course-pi --sdk C:/path/to/pico-sdk --arm-bin C:/path/to/arm-none-eabi/bin --output ../flash_ready/barcode_course
```

The helper checks configuration, production PWM/PI/STOP behavior and the
ELF/UF2 contents. It does not flash or start the robot.

Use the dashboard requirements and run `dashboard.py` with the broker,
`--mqtt-port 8084`, `--port 8001` and a local `--mapping-file`. Send the
instructor's four-letter mapping with `tools/barcode_map.py`. The dashboard
and MQTT diagnostic logger can run without USB serial.

## Validation and limits

The checkpoint passed the subsystem-3, course integration and dashboard host
suites. Width-fit tests include 11,212 checks, prefix resynchronisation covers
468 A–Z/direction/prefix cases, and course overlap tests cover 208 scenarios.
Physical wheel balancing was reported much straighter in a wireless test.
Full-size A and F have decoded, but repeatability and smaller printed cards
remain unresolved. Sensor height, alignment and front sensors encountering
barcode bars can affect results. This checkpoint is a fallback, not a claim
that the whole course or every barcode size has been physically validated.

The subsequent microsecond IRQ experiment and its 15%/20% selector are
preserved locally and are intentionally absent from this branch.
