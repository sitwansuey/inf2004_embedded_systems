CC ?= cc
CPPFLAGS += -I../.. -I.build
CFLAGS ?= -O1 -g
CFLAGS += -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic
BINARIES := .build/driver_test .build/data_test .build/api_test .build/sensor_task_test .build/mounting_test .build/terrain_input_test .build/result_test .build/odometry_storage_test .build/monitoring_test
SHIM_SOURCE := .build/shim/subsystem4/subsystem4.c
SHIM_INPUTS := ../../packed_bits.h ../../subsystem4.c ../../task_sensor.c ../../subsystem4.h ../../sensor_data.h ../../drv_lsm303dlhc.h ../../drv_pico_i2c.h app.h

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
	./.build/api_test cycle-start-fail
	./.build/api_test sensor-start-fail
	./.build/api_test monitor-start-fail
	./.build/sensor_task_test
	./.build/mounting_test
	./.build/monitoring_test
	./.build/terrain_input_test
	./.build/result_test
	./.build/odometry_storage_test
	python3 result_vectors.py
	python3 motion_vectors.py

.build/odometry_storage_test: odometry_storage_test.c ../../../subsystem2/odometry.c ../../../subsystem2/odometry.h ../../../shared/edr.c
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) odometry_storage_test.c ../../../subsystem2/odometry.c ../../../shared/edr.c $(LDFLAGS) -o "$@"

.build/result_test: result_test.c ../../result_data.c ../../result_data.h ../../packed_bits.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) result_test.c ../../result_data.c $(LDFLAGS) -o "$@"

.build/terrain_input_test: terrain_input_test.c ../../../subsystem2/odometry.h ../../encoder_input.c ../../encoder_input.h ../../terrain.c ../../geometry.c ../../terrain.h ../../mounting.h ../../../subsystem2/odometry.c ../../../shared/edr.c
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) terrain_input_test.c ../../encoder_input.c ../../terrain.c ../../geometry.c ../../../subsystem2/odometry.c ../../../shared/edr.c $(LDFLAGS) -lm -o "$@"

.build/monitoring_test: monitoring_test.c ../../motion.c ../../motion.h ../../geometry.c ../../terrain.c ../../sensor_data.c
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) monitoring_test.c ../../motion.c ../../geometry.c ../../terrain.c ../../sensor_data.c $(LDFLAGS) -lm -o "$@"

.build/driver_test: driver_test.c ../../drv_lsm303dlhc.c ../../drv_lsm303dlhc.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) driver_test.c ../../drv_lsm303dlhc.c $(LDFLAGS) -o "$@"

.build/data_test: data_test.c ../../sensor_data.c ../../sensor_data.h ../../drv_lsm303dlhc.h ../../packed_bits.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) data_test.c ../../sensor_data.c $(LDFLAGS) -o "$@"

$(SHIM_SOURCE): $(SHIM_INPUTS)
	@mkdir -p .build/shim/subsystem4
	cp ../../packed_bits.h ../../subsystem4.c ../../task_sensor.c ../../subsystem4.h ../../sensor_data.h ../../drv_lsm303dlhc.h ../../drv_pico_i2c.h .build/shim/subsystem4/
	cp app.h .build/shim/app.h

.build/api_test: api_test.c $(SHIM_SOURCE) ../../sensor_data.c
	$(CC) $(CPPFLAGS) $(CFLAGS) api_test.c $(SHIM_SOURCE) ../../sensor_data.c $(LDFLAGS) -o "$@"

.build/sensor_task_test: sensor_task_test.c $(SHIM_SOURCE) ../../sensor_data.c
	$(CC) $(CPPFLAGS) $(CFLAGS) sensor_task_test.c .build/shim/subsystem4/task_sensor.c ../../sensor_data.c $(LDFLAGS) -o "$@"

.build/mounting_test: mounting_test.c ../../mounting.h ../../terrain.c ../../terrain.h ../../result_data.c ../../sensor_data.c ../../../subsystem1/telemetry_codec.c ../../../subsystem1/telemetry_edr.h
	@mkdir -p .build
	$(CC) $(CPPFLAGS) $(CFLAGS) mounting_test.c ../../terrain.c ../../result_data.c ../../sensor_data.c ../../../subsystem1/telemetry_codec.c $(LDFLAGS) -lm -o "$@"

sanitize:
	$(MAKE) clean
	$(MAKE) test CFLAGS='-O1 -g -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer' LDFLAGS='-fsanitize=address,undefined'

clean:
	rm -rf .build
