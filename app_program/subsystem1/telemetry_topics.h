/* Shared with the dashboard MQTT contract. Values are stable enum IDs. */
#ifndef S1_TELEMETRY_TOPICS_H
#define S1_TELEMETRY_TOPICS_H

#define S1_TOPIC_MAX 96
#define S1_PAYLOAD_MAX 128
#define S1_TELEMETRY_CAPACITY 16

/* X-macro keeps IDs and topic strings in one place. */
#define S1_TOPIC_LIST(X) \
    X(S1_LEFT_IR_IR_LEFT, "Robopico/sensors/left_ir/ir_left") \
    X(S1_RIGHT_IR_IR_RIGHT, "Robopico/sensors/right_ir/ir_right") \
    X(S1_BARCODE_IR_IR_BARCODE, "Robopico/sensors/barcode_ir/ir_barcode") \
    X(S1_BARCODE_IR_BARCODE, "Robopico/sensors/barcode_ir/barcode") \
    X(S1_LEFT_ENCODER_ENCODER_LEFT, "Robopico/sensors/left_encoder/encoder_left") \
    X(S1_LEFT_ENCODER_LEFT_SPEED_MPS, "Robopico/sensors/left_encoder/left_speed_mps") \
    X(S1_RIGHT_ENCODER_ENCODER_RIGHT, "Robopico/sensors/right_encoder/encoder_right") \
    X(S1_RIGHT_ENCODER_RIGHT_SPEED_MPS, "Robopico/sensors/right_encoder/right_speed_mps") \
    X(S1_ACCELEROMETER_ACCEL_X_MPS2, "Robopico/sensors/accelerometer/accel_x_mps2") \
    X(S1_ACCELEROMETER_ACCEL_Y_MPS2, "Robopico/sensors/accelerometer/accel_y_mps2") \
    X(S1_ACCELEROMETER_ACCEL_Z_MPS2, "Robopico/sensors/accelerometer/accel_z_mps2") \
    X(S1_ACCELEROMETER_IMPACT, "Robopico/sensors/accelerometer/impact") \
    X(S1_ACCELEROMETER_IMU_STATUS, "Robopico/sensors/accelerometer/imu_status") \
    X(S1_MAGNETOMETER_MAG_X_UT, "Robopico/sensors/magnetometer/mag_x_ut") \
    X(S1_MAGNETOMETER_MAG_Y_UT, "Robopico/sensors/magnetometer/mag_y_ut") \
    X(S1_MAGNETOMETER_MAG_Z_UT, "Robopico/sensors/magnetometer/mag_z_ut") \
    X(S1_MAGNETOMETER_HEADING_DEG, "Robopico/sensors/magnetometer/heading_deg") \
    X(S1_MAGNETOMETER_PITCH_DEG, "Robopico/sensors/magnetometer/pitch_deg") \
    X(S1_MAGNETOMETER_ROLL_DEG, "Robopico/sensors/magnetometer/roll_deg") \
    X(S1_MAGNETOMETER_TURN_RATE_DPS, "Robopico/sensors/magnetometer/turn_rate_dps") \
    X(S1_MAGNETOMETER_VALID, "Robopico/sensors/magnetometer/valid") \
    X(S1_ULTRASONIC_RANGER_DISTANCE_CM, "Robopico/sensors/ultrasonic_ranger/distance_cm") \
    X(S1_ULTRASONIC_RANGER_WIDTH_CM, "Robopico/sensors/ultrasonic_ranger/width_cm") \
    X(S1_ULTRASONIC_RANGER_DEPTH_CM, "Robopico/sensors/ultrasonic_ranger/depth_cm") \
    X(S1_ULTRASONIC_RANGER_LEFT_CLEARANCE_CM, "Robopico/sensors/ultrasonic_ranger/left_clearance_cm") \
    X(S1_ULTRASONIC_RANGER_RIGHT_CLEARANCE_CM, "Robopico/sensors/ultrasonic_ranger/right_clearance_cm") \
    X(S1_TERRAIN_MOTION_CLASS, "Robopico/sensors/terrain/motion_class") \
    X(S1_TERRAIN_HUMP_STATE, "Robopico/sensors/terrain/hump_state") \
    X(S1_TERRAIN_HUMP_HEIGHT_CM, "Robopico/sensors/terrain/hump_height_cm") \
    X(S1_TERRAIN_RUN_MAX_HEIGHT_CM, "Robopico/sensors/terrain/run_max_height_cm") \
    X(S1_TERRAIN_VALID, "Robopico/sensors/terrain/valid") \
    X(S1_SERVO_MOTOR_STATE, "Robopico/Actuators/servo_motor/state") \
    X(S1_SERVO_MOTOR_SERVO_ANGLE_DEG, "Robopico/Actuators/servo_motor/servo_angle_deg") \
    X(S1_SERVO_MOTOR_BYPASS_SIDE, "Robopico/Actuators/servo_motor/bypass_side") \
    X(S1_SERVO_MOTOR_ROUTE, "Robopico/Actuators/servo_motor/route") \
    X(S1_MOTORS_STATE, "Robopico/Actuators/motors/state") \
    X(S1_MOTORS_DIRECTION, "Robopico/Actuators/motors/direction") \
    X(S1_MOTORS_POWER_PERCENT, "Robopico/Actuators/motors/power_percent") \
    X(S1_MOTORS_DISTANCE_M, "Robopico/Actuators/motors/distance_m") \
    X(S1_MOTORS_LAST_ERROR, "Robopico/Actuators/motors/last_error") \
    X(S1_MOTORS_LAST_COMMAND_MS, "Robopico/Actuators/motors/last_command_ms") \
    X(S1_VEHICLE_CONTROLLER_STATE, "Robopico/Actuators/vehicle_controller/state") \
    X(S1_VEHICLE_CONTROLLER_FAULT_CODE, "Robopico/Actuators/vehicle_controller/fault_code") \
    X(S1_VEHICLE_CONTROLLER_FAULT_MESSAGE, "Robopico/Actuators/vehicle_controller/fault_message") \
    X(S1_VEHICLE_CONTROLLER_SAFETY_STATE, "Robopico/Actuators/vehicle_controller/safety_state") \
    X(S1_VEHICLE_CONTROLLER_UPTIME_MS, "Robopico/Actuators/vehicle_controller/uptime_ms") \
    X(S1_VEHICLE_CONTROLLER_TELEMETRY_PERIOD_MS, "Robopico/Actuators/vehicle_controller/telemetry_period_ms") \
    X(S1_VEHICLE_CONTROLLER_LATENCY_MS, "Robopico/Actuators/vehicle_controller/latency_ms") \
    X(S1_VEHICLE_CONTROLLER_SEQUENCE, "Robopico/Actuators/vehicle_controller/sequence") \
    X(S1_VEHICLE_CONTROLLER_EVENTS, "Robopico/Actuators/vehicle_controller/events")

#define S1_TOPIC_ENUM(name, topic) name,
typedef enum {
    S1_TOPIC_LIST(S1_TOPIC_ENUM)
    S1_TOPIC_COUNT
} S1_READING_ID;
#undef S1_TOPIC_ENUM

/* Service bridge: no T-Kernel/lwIP includes, avoiding namespace conflicts.
 * Called ONLY by the network service. 1 = message, 0 = empty/not ready,
 * negative = error. Output arrays need the capacities above.
 */
int s1_telemetry_take(char *topic, char *payload);
#endif
