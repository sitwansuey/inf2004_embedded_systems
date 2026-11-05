CC ?= gcc
ROUTE = ../../app_program/barcode_route.c ../../app_program/barcode_boot.c
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Werror -pedantic
S4_RESULT = ../../app_program/subsystem4/result_data.c ../../app_program/subsystem4/sensor_data.c
CORE = $(S4_RESULT) ../../app_program/vehicle_policy.c ../../app_program/subsystem2/odometry.c ../../app_program/subsystem4/terrain.c ../../app_program/shared/edr.c
SCAN = ../../app_program/subsystem5/s5_scan.c ../../app_program/subsystem5/s5_policy.c ../../app_program/subsystem5/s5_width.c
.PHONY: test sanitize
test:
	mkdir -p .build
	$(CC) $(CFLAGS) wheel_calibration_test.c $(LDFLAGS) -o .build/wheel_calibration
	.build/wheel_calibration
	$(CC) $(CFLAGS) barcode_line_follow_test.c -o .build/barcode_line_follow
	.build/barcode_line_follow
	$(CC) $(CFLAGS) diagnostic_cli.c -o .build/diagnostic
	python3 diagnostic_test.py
	$(CC) $(CFLAGS) -Imocks -DTM_NET_LWIP=1 -DBOARD_BARCODE_DEMO=0 wifi_priority_test.c $(LDFLAGS) -o .build/wifi_priority_default
	.build/wifi_priority_default
	$(CC) $(CFLAGS) -Imocks -DTM_NET_LWIP=1 -DBOARD_BARCODE_DEMO=1 wifi_priority_test.c $(LDFLAGS) -o .build/wifi_priority_demo
	.build/wifi_priority_demo
	$(CC) $(CFLAGS) barcode_boot_test.c ../../app_program/vehicle_policy.c $(ROUTE) $(LDFLAGS) -o .build/barcode_boot
	.build/barcode_boot
	$(CC) $(CFLAGS) barcode_demo_test.c ../../app_program/vehicle_policy.c ../../app_program/subsystem3/buddy3_barcode.c ../../app_program/subsystem3/buddy3_core.c ../../app_program/subsystem3/buddy3_line.c $(ROUTE) $(LDFLAGS) -o .build/barcode_demo
	.build/barcode_demo
	$(CC) $(CFLAGS) speed30_test.c $(ROUTE) $(LDFLAGS) -o .build/speed30
	.build/speed30
	$(CC) $(CFLAGS) obstacle_bend_test.c ../../app_program/vehicle_policy.c $(ROUTE) $(LDFLAGS) -o .build/obstacle_bend
	.build/obstacle_bend
	$(CC) $(CFLAGS) edr_stream_test.c ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/edr_stream
	python3 edr_stream_test.py
	$(CC) $(CFLAGS) mission_test.c ../../app_program/vehicle_policy.c ../../app_program/subsystem3/buddy3_barcode.c $(ROUTE) $(LDFLAGS) -o .build/mission
	./.build/mission
	$(CC) $(CFLAGS) barcode_motor_vehicle_test.c $(CORE) ../../app_program/subsystem4/encoder_input.c $(ROUTE) $(LDFLAGS) -lm -o .build/barcode_motor_vehicle
	.build/barcode_motor_vehicle
	$(CC) $(CFLAGS) -DBOARD_BARCODE_WHEEL_PI=1 barcode_motor_vehicle_test.c $(CORE) ../../app_program/subsystem4/encoder_input.c $(ROUTE) $(LDFLAGS) -lm -o .build/barcode_wheel_pi_vehicle
	.build/barcode_wheel_pi_vehicle
	$(CC) $(CFLAGS) buddy4_vehicle_test.c $(CORE) ../../app_program/subsystem4/encoder_input.c $(ROUTE) $(LDFLAGS) -lm -o .build/buddy4_vehicle
	./.build/buddy4_vehicle
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=30 continuous_line_test.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c $(ROUTE) $(LDFLAGS) -o .build/continuous_line
	./.build/continuous_line
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=50 line_course_test.c ../../app_program/ir_line_course.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c $(ROUTE) $(LDFLAGS) -o .build/line_course
	./.build/line_course
	$(CC) $(CFLAGS) -DIR_SCAN_MAX_MS=20000 -DIR_SCAN_MAX_POWER=50 line_only_test.c ../../app_program/ir_line_progress.c ../../app_program/ir_motor_scan.c $(ROUTE) $(LDFLAGS) -o .build/line_only
	./.build/line_only
	$(CC) $(CFLAGS) -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 line_trace_test.c ../../app_program/ir_line_trace.c ../../app_program/vehicle_line.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem3/buddy3_line.c ../../app_program/subsystem3/buddy3_core.c ../../app_program/subsystem3/buddy3_barcode.c $(ROUTE) $(LDFLAGS) -o .build/line_trace
	./.build/line_trace
	$(CC) $(CFLAGS) policy_test.c $(CORE) $(ROUTE) $(LDFLAGS) -lm -o .build/policy
	./.build/policy
	$(CC) $(CFLAGS) -Wshadow -Wconversion ir_motor_scan_test.c ../../app_program/ir_motor_scan.c $(ROUTE) $(LDFLAGS) -o .build/ir_motor_scan
	./.build/ir_motor_scan
	$(CC) $(CFLAGS) ir_scan_usb_test.c ../../app_program/ir_motor_scan.c $(ROUTE) $(LDFLAGS) -o .build/ir_scan_usb
	./.build/ir_scan_usb
	$(CC) $(CFLAGS) -DBOARD_IR_MOTOR_SCAN_BENCH=1 ir_scan_queue_test.c ../../app_program/ir_motor_scan.c ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/ir_scan_queue
	./.build/ir_scan_queue
	$(CC) $(CFLAGS) -include host.h obstacle_motion_test.c $(SCAN) $(ROUTE) $(LDFLAGS) -o .build/obstacle_motion
	.build/obstacle_motion
	$(CC) $(CFLAGS) -include host.h obstacle_open_space_test.c $(SCAN) $(ROUTE) $(LDFLAGS) -o .build/obstacle_open_space
	.build/obstacle_open_space
	$(CC) $(CFLAGS) -include host.h obstacle_scan_test.c $(SCAN) $(ROUTE) $(LDFLAGS) -o .build/obstacle_scan
	.build/obstacle_scan
	$(CC) $(CFLAGS) -include host.h scan_test.c $(SCAN) $(ROUTE) $(LDFLAGS) -o .build/scan
	./.build/scan
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -Imocks -include s5_hardware_host.h s5_hardware_test.c ../../app_program/subsystem5/s5_hardware.c $(ROUTE) $(LDFLAGS) -o .build/sonar
	./.build/sonar
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -Imocks -include s5_hardware_host.h s5_status_test.c ../../app_program/subsystem5/s5_data.c $(ROUTE) $(LDFLAGS) -o .build/sonar_status
	./.build/sonar_status
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DTM_MQTT=1 -include s5_hardware_host.h snapshot_time_test.c ../../app_program/app_logging.c $(S4_RESULT) ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/snapshot_time
	./.build/snapshot_time
	$(CC) $(CFLAGS) -DBOARD_BARCODE_DEMO=1 -DS2_ENCODER_ENABLED=1 -DS2_COMMAND_TIMEOUT_MS=150 -DBOARD_UART0_FREE=1 -DTM_MQTT=1 -include ../../app_program/robot_config.h -include s5_hardware_host.h snapshot_time_test.c ../../app_program/app_logging.c $(S4_RESULT) ../../app_program/subsystem1/telemetry_codec.c ../../app_program/subsystem3/buddy3_line.c $(ROUTE) $(LDFLAGS) -o .build/snapshot_diagnostics
	./.build/snapshot_diagnostics
	$(CC) $(CFLAGS) protocol_cli.c ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/protocol
	python3 protocol_test.py
	$(CC) $(CFLAGS) telemetry_queue_test.c ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/queue
	./.build/queue
	$(CC) $(CFLAGS) -Imocks mqtt_test.c ../../app_program/subsystem1/telemetry_codec.c $(ROUTE) $(LDFLAGS) -o .build/mqtt
	./.build/mqtt
	python3 -m unittest discover -s ../barcode -v
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -fsanitize=address,undefined -fno-omit-frame-pointer' LDFLAGS='-fsanitize=address,undefined'
