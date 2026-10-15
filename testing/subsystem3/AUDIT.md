# Buddy 3 audit before implementation

Baseline main: 82114e223badc3d0eda2560663353eb9fbf045a1, tree 3df7e6bacce89d55160fe9534d94003d81121582. Direct clone is blocked by network policy. All 649 tracked files (642 unique blobs) were imported through the GitHub connector, SHA-verified and reconstructed with the exact original signed commit as a shallow local repository. Initial status is clean. No AGENTS.md, CODEOWNERS, existing Actions workflows, root Makefile, board_pins.h or controller implementation exist.

Architecture: Pico W RP2040, micro T-Kernel 3 SMP, build_make/makefile and pico_rp2040.mk, gnu11 Cortex-M0+ soft-float. app_main.c initializes all subsystems before starting them. Build wildcard covers app_program/*.c and app_program/*/*.c only. SDK 2.2.0 (README also says 2.3.0 tested), TinyUSB/cyw43 external for default build. UART/no-radio profile exists. Native GCC 15.2.0 and GNU Make 4.4.1 found in MSYS2; cross compiler and SDK absent. Firmware baseline make -C build_make -j2 fails at pico_rp2040.mk:130 (missing arm-none-eabi-gcc).

Ownership: Buddy 3 owns only subsystem3 and testing/subsystem3. TEAM.md is a stale example, priorities 16-17 are authoritative for implementation. Keep shared app.h/app_main.c, kernel, other buddies unchanged. S3 still consumes sample messages; S2 now motor implementation no longer declares SAMPLE_MSG/sem_slots/mpf_samples/S2_PERIOD_MS, but task_producer.c and S4 monitor still use them. Missing board_pins.h is another independent baseline build blocker. Preserve S3 legacy mailbox/mutex/stats symbols without inventing missing S2 definitions. Vehicle Controller must consume new S3 records; never call S2 PWM or encoder internals.

Hardware: write-up proposes GP26/27/28 but mixes DO/SIG and analogue labels. TEAM claims are empty. Kernel ADC enabled but pin init flags all FALSE, scan timeout 1000ms. Do not add SDK adc_init()/adc_read(): kernel already owns ADC hardware. Implement configurable GPIO acquisition and asynchronous kernel ADC requests, disabled by default, with no pin claims. ADC activation requires team pin configuration first.

Documents: task brief and starter fully read. ES (1).docx extracted; design priorities 0-4 conflict with actual T-Kernel/TEAM priorities. PDF spec labels 18mm line, 20mm gap, ~141mm barcode. Sample PDF pages 1/2 A and 3/4 Z share forms scaled horizontally ~2:1. Exact vector rectangles produce 29 alternating runs, start/stop patterns matching each other, narrow:wide 1:3. Support these observed single-symbol frames only. No confirmed symbol-command map, full codebook, checksum, sensor geometry or on-device waveform. No mapping by default. Pure capture uses time widths per character within bounded speed variation; clearance rearm supplied by controller, no pretend physical rescan.

Planned changed files: .github/workflows/buddy3-ci.yml; app_program/subsystem3/{buddy3_line,buddy3_barcode,buddy3_core,drv_ir}.{c,h}; subsystem3.{c,h}, task_consumer.c, subsystem3_test.c, README.md; testing/subsystem3/Makefile, native tests/support, TEST_LOG.md, logs and manual validation instructions. No other subsystem files. First code change is minimal CI plus real native normalization tests.

Commands: make -C testing/subsystem3 test; make -C testing/subsystem3 verify-failure; make -C build_make -j2; make -C build_make CONSOLE=uart WIFI=none -j2; git diff --check. Firmware CI deferred due independent broken interfaces and external unprovisioned toolchain/SDK. Connector permissions pull:true push:false; deliver git format-patch and local commits if no publication access.

## Detailed adapter audit correction before final implementation

The initial proposed asynchronous ADC approach above was rejected after tracing
kernel/tkernel/deviceio.c -> device/common/drvif/msdrvif.c:msdi_execfn ->
device/adc/adc.c:read_data -> adc_rp2040.c:adc_convert. The generic device API
invokes msdi_execfn in the requesting task; msdi_execfn ignores tmout and calls
the ADC read synchronously under its lock. TMO_POL cannot guarantee a bounded
ADC read. Final code explicitly rejects S3_IR_ADC with E_NOSPT and uses no ADC
registers, requests or pin setup. Analogue algorithms remain independent/tested.
This blocker needs the ADC/kernel owner's bounded acquisition contract.

GitHub connector metadata reports push:false. Remote feature branch creation was
attempted and returned HTTP 403 Resource not accessible by integration. No branch
or PR was created remotely. The local branch and format-patch series are the
reviewable delivery; Actions cannot run until a maintainer publishes the branch.
