CC = gcc
.DEFAULT_GOAL := test
CPPFLAGS += -I../../app_program/subsystem3
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -O2
ifeq ($(OS),Windows_NT)
EXEEXT = .exe
endif
CORE_SOURCES = ../../app_program/subsystem3/buddy3_line.c \
               ../../app_program/subsystem3/buddy3_barcode.c \
               ../../app_program/subsystem3/buddy3_core.c
CORE_HEADERS = $(wildcard ../../app_program/subsystem3/buddy3_*.h)
ADAPTER_SOURCES = test_adapter.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES)
ADAPTER_DEPS = host_support.h adapter_fixture.h \
               ../../app_program/subsystem3/drv_ir_adc.c \
               ../../app_program/subsystem3/drv_ir_adc.h ../../app_program/subsystem3/subsystem3.c \
               ../../app_program/subsystem3/subsystem3.h \
               ../../app_program/subsystem3/task_consumer.c \
               ../../app_program/subsystem3/drv_ir.h ../../app_program/subsystem3/buddy3_board.h $(CORE_HEADERS)
BINARY = .build/test_buddy3$(EXEEXT)
ADAPTER = .build/test_adapter$(EXEEXT)
ADC_TEST = .build/test_adc$(EXEEXT)
ADC_TICK = .build/test_adc_tick$(EXEEXT)
ADC_ACQUIRE = .build/test_adc_acquire$(EXEEXT)
RAW_TEST = .build/test_raw$(EXEEXT)
RAW_UNLOCKED = .build/test_raw_unlocked$(EXEEXT)
ROBOT_LAYOUT = .build/test_robot_ir_layout$(EXEEXT)
ROBOT_CALIBRATION = .build/test_robot_ir_calibration$(EXEEXT)
ROBOT_CAL_DEFAULT = .build/test_robot_ir_cal_default$(EXEEXT)
PAUSED_TIME = .build/test_paused_time$(EXEEXT)
BARCODE_CAPTURE = .build/test_barcode_capture$(EXEEXT)
MOTOR_SCAN_SAMPLER = .build/test_motor_scan_sampler$(EXEEXT)
SAMPLE_SCHEDULE = .build/test_sample_schedule$(EXEEXT)
EDGE_BIAS = .build/test_barcode_edge_bias$(EXEEXT)

.PHONY: test verify-failure check-headers clean barcode-replay
barcode-replay: .build/replay_barcode_capture$(EXEEXT)

.build/replay_barcode_capture$(EXEEXT): replay_barcode_capture.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_calibration.h ../../app_program/subsystem3/buddy3_board.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 replay_barcode_capture.c $(CORE_SOURCES) $(LDFLAGS) -o $@
test: $(EDGE_BIAS) $(BINARY) $(ADAPTER) $(ADC_TEST) $(ADC_TICK) $(ADC_ACQUIRE) $(RAW_TEST) $(RAW_UNLOCKED) $(ROBOT_LAYOUT) $(ROBOT_CALIBRATION) $(ROBOT_CAL_DEFAULT) $(PAUSED_TIME) $(BARCODE_CAPTURE) $(SAMPLE_SCHEDULE) $(MOTOR_SCAN_SAMPLER)
	./$(BINARY) $(TEST_ARGS)
	./$(ADAPTER)
	./$(ADC_TEST)
	./$(ADC_TICK)
	./$(ADC_ACQUIRE)
	./$(RAW_TEST)
	./$(RAW_UNLOCKED)
	./$(ROBOT_LAYOUT)
	./$(ROBOT_CALIBRATION)
	./$(ROBOT_CAL_DEFAULT)
	./$(PAUSED_TIME)
	./$(BARCODE_CAPTURE)
	./$(SAMPLE_SCHEDULE)
	./$(MOTOR_SCAN_SAMPLER)
	./$(EDGE_BIAS)

$(EDGE_BIAS): test_barcode_edge_bias.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_calibration.h ../../app_program/subsystem3/buddy3_board.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 test_barcode_edge_bias.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(MOTOR_SCAN_SAMPLER): test_motor_scan_sampler.c $(ADAPTER_DEPS) $(CORE_SOURCES) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MOTOR_SCAN_BENCH=1 -DBOARD_MOTORS_DISABLED=0 -DBOARD_IR_MODE=2 -DBOARD_IR_BARCODE_CAPTURE=1 -DBOARD_UART0_FREE=1 -DS2_COMMAND_TIMEOUT_MS=150 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_motor_scan_sampler.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(SAMPLE_SCHEDULE): test_sample_schedule.c $(ADAPTER_DEPS) $(CORE_SOURCES)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_sample_schedule.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(BARCODE_CAPTURE): test_barcode_capture.c $(ADAPTER_DEPS) $(CORE_SOURCES) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_BARCODE_CAPTURE=1 -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_barcode_capture.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(PAUSED_TIME): test_paused_time.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_board.h ../../app_program/subsystem3/buddy3_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 test_paused_time.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_CALIBRATION): test_robot_ir_calibration.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_board.h ../../app_program/subsystem3/buddy3_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 test_robot_ir_calibration.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_CAL_DEFAULT): test_robot_ir_calibration.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_board.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_robot_ir_calibration.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_LAYOUT): test_robot_ir_layout.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_robot_ir.h ../../app_program/subsystem3/buddy3_board.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_robot_ir_layout.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(RAW_TEST): test_raw.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_raw.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(RAW_UNLOCKED): test_raw.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=0 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_raw.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(BINARY): test_buddy3.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_buddy3.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADAPTER): $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -include host_support.h $(ADAPTER_SOURCES) $(LDFLAGS) -o $@

$(ADC_TEST): test_adc.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADC_TICK): test_adc_tick.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc_tick.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADC_ACQUIRE): test_adc_acquire.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc_acquire.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

verify-failure: $(BINARY) $(ADAPTER) $(ADC_TEST)
	@if $(MAKE) --no-print-directory test TEST_ARGS=--force-failure; then \
	    echo "ERROR: failing assertion was swallowed"; exit 1; \
	else echo "PASS: test target propagated a real failed assertion"; fi

clean:
	rm -f $(BINARY) $(ADAPTER) $(ADC_TEST) $(ADC_TICK) $(ADC_ACQUIRE) $(RAW_TEST) $(RAW_UNLOCKED) $(ROBOT_LAYOUT) $(ROBOT_CALIBRATION) $(ROBOT_CAL_DEFAULT) $(PAUSED_TIME) $(BARCODE_CAPTURE) $(SAMPLE_SCHEDULE) $(MOTOR_SCAN_SAMPLER) $(EDGE_BIAS) .build/replay_barcode_capture$(EXEEXT)

# Host parsing only: kernel MMIO casts are 32-bit, host pointers may be 64-bit.
# -ffreestanding also selects GCC's standard integer definitions, as firmware does.
check-headers:
	@for smp in 0 1; do \
	    echo "Checking real kernel headers, SMP=$$smp"; \
	    $(CC) -std=gnu11 -ffreestanding -Wall -Wextra -Werror \
	      -Wno-int-to-pointer-cast -fsyntax-only -D_PICO_RP2040_ \
	      -DCNF_SMP=$$smp -DAPP_TEST=1 -DTM_CONSOLE_UART=0 \
	      -I../../include -I../../config -I../../kernel/knlinc \
	      ../../app_program/subsystem3/drv_ir.c \
	      ../../app_program/subsystem3/drv_ir_adc.c \
	      ../../app_program/subsystem3/subsystem3.c \
	      ../../app_program/subsystem3/task_consumer.c \
	      ../../app_program/subsystem3/subsystem3_test.c || exit $$?; \
	done


.PHONY: analyze arm-check
analyze:
	$(MAKE) --no-print-directory -B test CFLAGS="$(CFLAGS) -fanalyzer -O1"

# Real Cortex-M0+ object compilation, separately labelled from a firmware link.
# No host stubs, SDK substitutions, or replacement subsystem implementations.
ARM_CC ?= arm-none-eabi-gcc
ARM_SOURCES = $(CORE_SOURCES) ../../app_program/subsystem3/drv_ir.c \
              ../../app_program/subsystem3/drv_ir_adc.c \
              ../../app_program/subsystem3/subsystem3.c \
              ../../app_program/subsystem3/task_consumer.c \
              ../../app_program/subsystem3/subsystem3_test.c
arm-check:
	$(ARM_CC) --version
	@for smp in 0 1; do \
	    mkdir -p .build/arm-smp$$smp; \
	    for source in $(ARM_SOURCES); do \
	        name=$$(basename "$$source" .c); \
	        $(ARM_CC) -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft \
	          -std=gnu11 -ffreestanding -O2 -Wall -Wextra -Werror \
	          -D_PICO_RP2040_ -DCNF_SMP=$$smp -DAPP_TEST=1 -DTM_CONSOLE_UART=0 \
	          -I../../include -I../../config -I../../kernel/knlinc \
	          -c "$$source" -o ".build/arm-smp$$smp/$$name.o" || exit $$?; \
	    done; \
	    echo "PASS: Buddy 3 ARM object compilation SMP=$$smp"; \
	done
