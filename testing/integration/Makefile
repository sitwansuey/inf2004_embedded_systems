CC ?= gcc
# ROBOT_CONFIG_NO_LOCAL: a git-ignored app_program/robot_config.local.h never
# leaks into host tests. The robot_config.h defaults are the mission profile
# (mask-pivot follower, straddle IR mode 3, PID, ramp speed, lift fault);
# other scenarios are explicit -D flags on their recipe.
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Werror -pedantic -DROBOT_CONFIG_NO_LOCAL
S4_RESULT = ../../app_program/subsystem4/result_data.c ../../app_program/subsystem4/sensor_data.c
CORE = $(S4_RESULT) ../../app_program/vehicle_policy.c ../../app_program/subsystem2/odometry.c ../../app_program/subsystem4/terrain.c ../../app_program/shared/edr.c
SCAN = ../../app_program/subsystem5/s5_scan.c ../../app_program/subsystem5/s5_policy.c ../../app_program/subsystem5/s5_width.c
# vehicle.c's straddle line observer and the S3 configuration it validates (S3 owns ADC acquisition).
STRADDLE_SRC = ../../app_program/ir_line_trace.c ../../app_program/subsystem3/buddy3_line.c ../../app_program/subsystem3/buddy3_core.c ../../app_program/subsystem3/buddy3_barcode.c
# $(SCAN): the vehicle test feeds samples through S5's real acquisition step.
VEHICLE_SRC = $(CORE) ../../app_program/subsystem4/encoder_input.c $(SCAN) $(STRADDLE_SRC)
.PHONY: test sanitize
test:
	mkdir -p .build
	$(CC) $(CFLAGS) -include host.h width_avoidance_test.c ../../app_program/vehicle_policy.c ../../app_program/subsystem5/s5_width.c ../../app_program/subsystem5/s5_policy.c ../../app_program/subsystem5/s5_data.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -lm -o .build/width_avoidance
	./.build/width_avoidance
	$(CC) $(CFLAGS) smooth_motor_test.c $(LDFLAGS) -o .build/smooth_motor
	./.build/smooth_motor
	$(CC) $(CFLAGS) speed30_test.c $(LDFLAGS) -o .build/speed30
	.build/speed30
	$(CC) $(CFLAGS) obstacle_bend_test.c ../../app_program/vehicle_policy.c $(LDFLAGS) -o .build/obstacle_bend
	.build/obstacle_bend
	$(CC) $(CFLAGS) bypass_return_test.c ../../app_program/vehicle_policy.c $(LDFLAGS) -o .build/bypass_return
	./.build/bypass_return
	$(CC) $(CFLAGS) -D_USE_MATH_DEFINES reverse_track_test.c $(LDFLAGS) -lm -o .build/reverse_track
	.build/reverse_track
	$(CC) $(CFLAGS) line_steer_test.c $(LDFLAGS) -o .build/line_steer
	.build/line_steer
	$(CC) $(CFLAGS) obstacle_confirm_test.c $(LDFLAGS) -o .build/obstacle_confirm
	.build/obstacle_confirm
	$(CC) $(CFLAGS) barcode_retry_test.c $(LDFLAGS) -o .build/barcode_retry
	.build/barcode_retry
	$(CC) $(CFLAGS) edr_stream_test.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/edr_stream
	python3 edr_stream_test.py
	$(CC) $(CFLAGS) mission_test.c ../../app_program/vehicle_policy.c ../../app_program/subsystem3/buddy3_barcode.c $(LDFLAGS) -o .build/mission
	./.build/mission
	$(CC) $(CFLAGS) nav_junction_test.c ../../app_program/vehicle_policy.c $(LDFLAGS) -o .build/nav_junction
	./.build/nav_junction
	$(CC) $(CFLAGS) line_trace_test.c $(STRADDLE_SRC) $(LDFLAGS) -o .build/line_trace
	./.build/line_trace
	$(CC) $(CFLAGS) -DBOARD_IR_MODE=2 line_trace_test.c $(STRADDLE_SRC) $(LDFLAGS) -o .build/line_trace_mode2
	./.build/line_trace_mode2
	$(CC) $(CFLAGS) policy_test.c $(CORE) $(LDFLAGS) -lm -o .build/policy
	./.build/policy
	$(CC) $(CFLAGS) watchdog_test.c ../../app_program/shared/watchdog_policy.c $(LDFLAGS) -o .build/watchdog
	./.build/watchdog
	$(CC) $(CFLAGS) -include host.h obstacle_motion_test.c $(SCAN) $(LDFLAGS) -o .build/obstacle_motion
	.build/obstacle_motion
	$(CC) $(CFLAGS) -include host.h obstacle_open_space_test.c $(SCAN) $(LDFLAGS) -o .build/obstacle_open_space
	.build/obstacle_open_space
	$(CC) $(CFLAGS) -include host.h obstacle_scan_test.c $(SCAN) $(LDFLAGS) -o .build/obstacle_scan
	.build/obstacle_scan
	$(CC) $(CFLAGS) -include host.h scan_test.c $(SCAN) $(LDFLAGS) -o .build/scan
	./.build/scan
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DBOARD_MOTORS_DISABLED=1 -Imocks -include s5_hardware_host.h s5_hardware_test.c ../../app_program/subsystem5/s5_hardware.c $(LDFLAGS) -o .build/sonar
	./.build/sonar
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DBOARD_MOTORS_DISABLED=1 -DBOARD_SERVO_CENTER_US=1300 -Imocks -include s5_hardware_host.h s5_hardware_test.c ../../app_program/subsystem5/s5_hardware.c $(LDFLAGS) -o .build/sonar_low_centre
	./.build/sonar_low_centre
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DBOARD_MOTORS_DISABLED=1 -DBOARD_SERVO_US_PER_90=1000 -Imocks -include s5_hardware_host.h s5_hardware_test.c ../../app_program/subsystem5/s5_hardware.c $(LDFLAGS) -o .build/sonar_slope
	./.build/sonar_slope
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -Imocks -include s5_hardware_host.h s5_status_test.c ../../app_program/subsystem5/s5_data.c $(LDFLAGS) -o .build/sonar_status
	./.build/sonar_status
	$(CC) $(CFLAGS) -DBOARD_UART0_FREE=1 -DTM_MQTT=1 -include s5_hardware_host.h snapshot_time_test.c ../../app_program/app_logging.c $(S4_RESULT) ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/snapshot_time
	./.build/snapshot_time
	$(CC) $(CFLAGS) protocol_cli.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/protocol
	python3 protocol_test.py
	$(CC) $(CFLAGS) telemetry_queue_test.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/queue
	./.build/queue
	$(CC) $(CFLAGS) -Imocks mqtt_test.c ../../app_program/subsystem1/telemetry_codec.c $(LDFLAGS) -o .build/mqtt
	./.build/mqtt
	$(CC) $(CFLAGS) -include host.h -DBOARD_RAMP_SPEED=0 -DBOARD_LIFT_FAULT_MS=0U buddy4_vehicle_test.c $(VEHICLE_SRC) $(LDFLAGS) -lm -o .build/buddy4_vehicle_no_ramp_lift
	./.build/buddy4_vehicle_no_ramp_lift
	$(CC) $(CFLAGS) -include host.h buddy4_vehicle_test.c $(VEHICLE_SRC) $(LDFLAGS) -lm -o .build/buddy4_vehicle
	./.build/buddy4_vehicle
	$(CC) $(CFLAGS) -include host.h -DBOARD_SHOCK_STOP=1 buddy4_vehicle_test.c $(VEHICLE_SRC) $(LDFLAGS) -lm -o .build/buddy4_vehicle_shock_stop
	./.build/buddy4_vehicle_shock_stop
	$(CC) $(CFLAGS) -include host.h -DBOARD_LINE_MASK_PIVOT=0 -DBOARD_LINE_EDGE_TRACE=1 buddy4_vehicle_test.c $(VEHICLE_SRC) $(LDFLAGS) -lm -o .build/buddy4_vehicle_edge_trace
	./.build/buddy4_vehicle_edge_trace
	$(CC) $(CFLAGS) -include host.h -DBOARD_BARCODE_READ_LINE_STEER=150 buddy4_vehicle_test.c $(VEHICLE_SRC) $(LDFLAGS) -lm -o .build/buddy4_vehicle_read150
	./.build/buddy4_vehicle_read150
sanitize:
	$(MAKE) test CFLAGS='$(CFLAGS) -fsanitize=address,undefined -fno-omit-frame-pointer' LDFLAGS='-fsanitize=address,undefined'
