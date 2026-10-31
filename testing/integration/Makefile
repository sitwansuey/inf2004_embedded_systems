CC ?= gcc
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Werror -pedantic
S4_RESULT = ../../app_program/subsystem4/result_data.c ../../app_program/subsystem4/sensor_data.c
CORE = $(S4_RESULT) ../../app_program/vehicle_policy.c ../../app_program/subsystem2/odometry.c ../../app_program/subsystem4/terrain.c ../../app_program/shared/edr.c
SCAN = ../../app_program/subsystem5/s5_scan.c ../../app_program/subsystem5/s5_policy.c ../../app_program/subsystem5/s5_width.c
.PHONY: test sanitize
test:
	mkdir -p .build
	$(CC) $(CFLAGS) buddy4_vehicle_test.c $(CORE) ../../app_program/subsystem4/encoder_input.c $(LDFLAGS) -lm -o .build/buddy4_vehicle
	./.build/buddy4_vehicle
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=30 continuous_line_test.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c $(LDFLAGS) -o .build/continuous_line
	./.build/continuous_line
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=50 line_course_test.c ../../app_program/ir_line_course.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c $(LDFLAGS) -o .build/line_course
	./.build/line_course
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=50 line_only_test.c ../../app_program/ir_line_progress.c ../../app_program/ir_motor_scan.c $(LDFLAGS) -o .build/line_only
	./.build/line_only
	$(CC) $(CFLAGS) -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 line_trace_test.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c ../../app_program/subsystem3/buddy3_core.c ../../app_program/subsystem3/buddy3_barcode.c $(LDFLAGS) -o .build/line_trace
	./.build/line_trace
	$(CC) $(CFLAGS) policy_test.c $(CORE) $(LDFLAGS) -lm -o .build/policy
	./.build/policy
	$(CC) $(CFLAGS) -Wshadow -Wconversion ir_motor_scan_test.c ../../app_program/ir_motor_scan.c $(LDFLAGS) -o .build/ir_motor_scan
	./.build/ir_motor_scan
	$(CC) $(CFLAGS) ir_scan_usb_test.c ../../app_program/ir_motor_scan.c $(LDFLAGS) -o .build/ir_scan_usb
	./.build/ir_scan_usb
	$(CC) $(CFLAGS) -DBOARD_IR_MOTOR_SCAN_BENCH=1 ir_scan_queue_test.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/ir_scan_queue
	./.build/ir_scan_queue
	$(CC) $(CFLAGS) -include host.h scan_test.c $(SCAN) $(LDFLAGS) -o .build/scan
	./.build/scan
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -Imocks -include s5_hardware_host.h s5_hardware_test.c ../../app_program/subsystem5/s5_hardware.c $(LDFLAGS) -o .build/sonar
	./.build/sonar
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -Imocks -include s5_hardware_host.h s5_status_test.c ../../app_program/subsystem5/s5_data.c $(LDFLAGS) -o .build/sonar_status
	./.build/sonar_status
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DTM_MQTT=1 -include s5_hardware_host.h snapshot_time_test.c ../../app_program/app_logging.c $(S4_RESULT) ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/snapshot_time
	./.build/snapshot_time
	$(CC) $(CFLAGS) protocol_cli.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/protocol
	python3 protocol_test.py
	$(CC) $(CFLAGS) telemetry_queue_test.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/queue
	./.build/queue
	$(CC) $(CFLAGS) -Imocks mqtt_test.c $(LDFLAGS) -o .build/mqtt
	./.build/mqtt
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -fsanitize=address,undefined -fno-omit-frame-pointer' LDFLAGS='-fsanitize=address,undefined'
