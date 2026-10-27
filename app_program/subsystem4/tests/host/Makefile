CC ?= cc
CPPFLAGS += -I../.. -I.build
CFLAGS ?= -O1 -g
CFLAGS += -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic
BINARIES := .build/driver_test .build/data_test .build/api_test .build/terrain_input_test
SHIM_SOURCE := .build/shim/subsystem4/subsystem4.c
SHIM_INPUTS := ../../subsystem4.c ../../subsystem4.h ../../sensor_data.h ../../drv_lsm303dlhc.h app.h

.PHONY: test sanitize clean sizes analyze
analyze:
	$(MAKE) -B test CFLAGS='$(CFLAGS) -fanalyzer'
sizes:
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) size_probe.c -o .build/size_probe
	./.build/size_probe
test: $(BINARIES)
	./.build/driver_test
	./.build/data_test
	./.build/api_test
	./.build/terrain_input_test

.build/terrain_input_test: terrain_input_test.c ../../encoder_input.c ../../encoder_input.h ../../terrain.c ../../terrain.h ../../../subsystem2/odometry.c ../../../shared/edr.c
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) terrain_input_test.c ../../encoder_input.c ../../terrain.c ../../../subsystem2/odometry.c ../../../shared/edr.c $(LDFLAGS) -lm -o "$@"

.build/driver_test: driver_test.c ../../drv_lsm303dlhc.c ../../drv_lsm303dlhc.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) driver_test.c ../../drv_lsm303dlhc.c $(LDFLAGS) -o "$@"

.build/data_test: data_test.c ../../sensor_data.c ../../sensor_data.h ../../drv_lsm303dlhc.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) data_test.c ../../sensor_data.c $(LDFLAGS) -o "$@"

$(SHIM_SOURCE): $(SHIM_INPUTS)
	@mkdir -p .build/shim/subsystem4
	cp ../../subsystem4.c ../../subsystem4.h ../../sensor_data.h ../../drv_lsm303dlhc.h .build/shim/subsystem4/
	cp app.h .build/shim/app.h

.build/api_test: api_test.c $(SHIM_SOURCE) ../../sensor_data.c
	$(CC) $(CPPFLAGS) $(CFLAGS) api_test.c $(SHIM_SOURCE) ../../sensor_data.c $(LDFLAGS) -o "$@"

sanitize:
	$(MAKE) clean
	$(MAKE) test CFLAGS='-O1 -g -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer' LDFLAGS='-fsanitize=address,undefined'

clean:
	rm -rf .build
