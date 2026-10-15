CC = gcc
CPPFLAGS += -I../../app_program/subsystem3
CFLAGS = -std=c11 -Wall -Wextra -Werror -pedantic -O2
SOURCES = test_buddy3.c ../../app_program/subsystem3/buddy3_line.c
BINARY = .build/test_buddy3

.PHONY: test verify-failure clean
test: $(BINARY)
	./$(BINARY) $(TEST_ARGS)

$(BINARY): $(SOURCES) ../../app_program/subsystem3/buddy3_line.h
	mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SOURCES) $(LDFLAGS) -o $@

verify-failure: $(BINARY)
	@if $(MAKE) --no-print-directory test TEST_ARGS=--force-failure; then \
	    echo "ERROR: failing assertion was swallowed"; exit 1; \
	else echo "PASS: test target propagated a real failed assertion"; fi

clean:
	rm -f $(BINARY) $(BINARY).exe
