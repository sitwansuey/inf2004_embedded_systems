CC = gcc
.DEFAULT_GOAL := test
# ROBOT_CONFIG_NO_LOCAL: a git-ignored robot_config.local.h never leaks into host tests.
CPPFLAGS += -I../../app_program/subsystem3 -DROBOT_CONFIG_NO_LOCAL
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
               ../../app_program/subsystem3/drv_ir.h ../../app_program/robot_config.h $(CORE_HEADERS)
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
ROBOT_CAL_OFF = .build/test_robot_ir_cal_off$(EXEEXT)
PAUSED_TIME = .build/test_paused_time$(EXEEXT)
SAMPLE_SCHEDULE = .build/test_sample_schedule$(EXEEXT)
EDGE_BIAS = .build/test_barcode_edge_bias$(EXEEXT)
RESYNC = .build/test_barcode_resync$(EXEEXT)
COMPLETION = .build/test_barcode_completion$(EXEEXT)
BARCODE_IMPORT = .build/test_barcode_import$(EXEEXT)
MIXED_BARCODE = .build/test_mixed_barcode$(EXEEXT)
END_QUIET = .build/test_barcode_end_quiet$(EXEEXT)
FAULT_FAIL = .build/test_barcode_fault_fail$(EXEEXT)

# BOARD_IR_MODE 2 builds use the measured ADC endpoints (robot_config.h
# includes robot_ir_calibration.h for modes 2 and 3; 3 is the mission default).
# The ADC worker tests sample all three channels, so they pin mode 2 (in the
# mission's mode 3 GP28 is a digital input; see test_mixed_barcode).
.PHONY: test verify-failure check-headers clean
test: $(RESYNC) $(COMPLETION) $(END_QUIET) $(FAULT_FAIL) $(BARCODE_IMPORT) $(MIXED_BARCODE) $(EDGE_BIAS) $(BINARY) $(ADAPTER) $(ADC_TEST) $(ADC_TICK) $(ADC_ACQUIRE) $(RAW_TEST) $(RAW_UNLOCKED) $(ROBOT_LAYOUT) $(ROBOT_CALIBRATION) $(ROBOT_CAL_DEFAULT) $(ROBOT_CAL_OFF) $(PAUSED_TIME) $(SAMPLE_SCHEDULE)
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
	./$(ROBOT_CAL_OFF)
	./$(PAUSED_TIME)
	./$(SAMPLE_SCHEDULE)
	./$(EDGE_BIAS)
	./$(RESYNC)
	./$(COMPLETION)
	./$(END_QUIET)
	./$(FAULT_FAIL)
	./$(BARCODE_IMPORT)
	./$(MIXED_BARCODE)

$(RESYNC): test_barcode_resync.c $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_barcode.c
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_barcode_resync.c ../../app_program/subsystem3/buddy3_barcode.c $(LDFLAGS) -o $@

$(COMPLETION): test_barcode_completion.c test_barcode_resync.c $(CORE_HEADERS) ../../app_program/subsystem3/buddy3_barcode.c
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_barcode_completion.c ../../app_program/subsystem3/buddy3_barcode.c $(LDFLAGS) -o $@

$(END_QUIET): test_barcode_end_quiet.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_barcode_end_quiet.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(FAULT_FAIL): test_barcode_fault_fail.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_barcode_fault_fail.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(BARCODE_IMPORT): test_barcode_import.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_barcode_import.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(MIXED_BARCODE): test_mixed_barcode.c $(ADAPTER_DEPS) $(CORE_SOURCES) ../../app_program/robot_ir_config.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=3 -DS3_TEST_ADC_ENABLED=1 -DDEVCONF_ADC_PIN_INIT_2=0 -include host_support.h test_mixed_barcode.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(EDGE_BIAS): test_barcode_edge_bias.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_ir_calibration.h ../../app_program/robot_config.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 test_barcode_edge_bias.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(SAMPLE_SCHEDULE): test_sample_schedule.c $(ADAPTER_DEPS) $(CORE_SOURCES)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_sample_schedule.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(PAUSED_TIME): test_paused_time.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_config.h ../../app_program/robot_ir_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 test_paused_time.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_CALIBRATION): test_robot_ir_calibration.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_config.h ../../app_program/robot_ir_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 test_robot_ir_calibration.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_CAL_DEFAULT): test_robot_ir_calibration.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_config.h ../../app_program/robot_ir_calibration.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_robot_ir_calibration.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_CAL_OFF): test_robot_ir_calibration.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_config.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=0 test_robot_ir_calibration.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ROBOT_LAYOUT): test_robot_ir_layout.c $(CORE_SOURCES) $(CORE_HEADERS) ../../app_program/robot_ir_config.h ../../app_program/robot_config.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_robot_ir_layout.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(RAW_TEST): test_raw.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_raw.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(RAW_UNLOCKED): test_raw.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=0 -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_raw.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(BINARY): test_buddy3.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_buddy3.c $(CORE_SOURCES) $(LDFLAGS) -o $@

# BOARD_IR_MODE 0: a board without the IR ADC pins (S3-U19 expects ADC mode
# to be refused); the mission default (mode 3) is covered by test_mixed_barcode.
$(ADAPTER): $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=0 -include host_support.h $(ADAPTER_SOURCES) $(LDFLAGS) -o $@

$(ADC_TEST): test_adc.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADC_TICK): test_adc_tick.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc_tick.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADC_ACQUIRE): test_adc_acquire.c $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -DBOARD_MOTORS_DISABLED=1 -DBOARD_IR_MODE=2 -DS3_TEST_ADC_ENABLED=1 -include host_support.h test_adc_acquire.c ../../app_program/subsystem3/drv_ir.c $(CORE_SOURCES) $(LDFLAGS) -o $@

verify-failure: $(BINARY) $(ADAPTER) $(ADC_TEST)
	@if $(MAKE) --no-print-directory test TEST_ARGS=--force-failure; then \
	    echo "ERROR: failing assertion was swallowed"; exit 1; \
	else echo "PASS: test target propagated a real failed assertion"; fi

clean:
	rm -rf .build

# Host parsing only: kernel MMIO casts are 32-bit, host pointers may be 64-bit.
# -ffreestanding also selects GCC's standard integer definitions, as firmware does.
check-headers:
	@for smp in 0 1; do \
	    echo "Checking real kernel headers, SMP=$$smp"; \
	    $(CC) -std=gnu11 -ffreestanding -Wall -Wextra -Werror \
	      -Wno-int-to-pointer-cast -fsyntax-only -D_PICO_RP2040_ \
	      -DCNF_SMP=$$smp -DAPP_TEST=1 -DTM_CONSOLE_UART=0 -DROBOT_CONFIG_NO_LOCAL \
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
