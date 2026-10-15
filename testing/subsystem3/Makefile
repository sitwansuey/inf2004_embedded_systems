CC = gcc
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
ADAPTER_DEPS = host_support.h ../../app_program/subsystem3/subsystem3.c \
               ../../app_program/subsystem3/subsystem3.h \
               ../../app_program/subsystem3/task_consumer.c \
               ../../app_program/subsystem3/drv_ir.h $(CORE_HEADERS)
BINARY = .build/test_buddy3$(EXEEXT)
ADAPTER = .build/test_adapter$(EXEEXT)

.PHONY: test verify-failure check-headers clean
test: $(BINARY) $(ADAPTER)
	./$(BINARY) $(TEST_ARGS)
	./$(ADAPTER)

$(BINARY): test_buddy3.c $(CORE_SOURCES) $(CORE_HEADERS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) test_buddy3.c $(CORE_SOURCES) $(LDFLAGS) -o $@

$(ADAPTER): $(ADAPTER_SOURCES) $(ADAPTER_DEPS)
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) -include host_support.h $(ADAPTER_SOURCES) $(LDFLAGS) -o $@

verify-failure: $(BINARY) $(ADAPTER)
	@if $(MAKE) --no-print-directory test TEST_ARGS=--force-failure; then \
	    echo "ERROR: failing assertion was swallowed"; exit 1; \
	else echo "PASS: test target propagated a real failed assertion"; fi

clean:
	rm -f $(BINARY) $(ADAPTER)

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
	      ../../app_program/subsystem3/subsystem3.c \
	      ../../app_program/subsystem3/task_consumer.c \
	      ../../app_program/subsystem3/subsystem3_test.c || exit $$?; \
	done
