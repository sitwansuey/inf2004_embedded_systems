// The 15 EDR16 records in test_packed16.py expand into these 17 display fields.
// This is receiver output, not a second implementation of the binary decoder.
export const WIRE_TOPIC = 'Robopico/sensors/edr16/v1';
export const EDR16_FIELDS = [
  ['motion', 'encoder_left', 'Robopico/sensors/left_encoder/encoder_left', '100', 100],
  ['motion', 'encoder_right', 'Robopico/sensors/right_encoder/encoder_right', '200', 200],
  ['obstacle', 'servo_angle_deg', 'Robopico/Actuators/servo_motor/servo_angle_deg', '90', 90],
  ['obstacle', 'distance_cm', 'Robopico/sensors/ultrasonic_ranger/distance_cm', '32', 32],
  ['navigation', 'ir_left', 'Robopico/sensors/left_ir/ir_left', '0', 0],
  ['navigation', 'ir_right', 'Robopico/sensors/right_ir/ir_right', '4095', 4095],
  ['navigation', 'ir_barcode', 'Robopico/sensors/barcode_ir/ir_barcode', '2048', 2048],
  ['terrain', 'mag_x_ut', 'Robopico/sensors/magnetometer/mag_x_ut', '-10.0', -10],
  ['terrain', 'mag_y_ut', 'Robopico/sensors/magnetometer/mag_y_ut', '0.0', 0],
  ['terrain', 'mag_z_ut', 'Robopico/sensors/magnetometer/mag_z_ut', '10.0', 10],
  ['motion', 'direction', 'Robopico/Actuators/motors/direction', 'FORWARD', 'FORWARD'],
  ['motion', 'state', 'Robopico/Actuators/motors/state', 'MOVING', 'MOVING'],
  ['motion', 'power_percent', 'Robopico/Actuators/motors/power_percent', '30', 30],
  ['motion', 'fault_flags', 'Robopico/Actuators/motors/fault_flags', '32', 32],
  ['terrain', 'accel_x_mps2', 'Robopico/sensors/accelerometer/accel_x_mps2', '9.80665', 9.80665],
  ['terrain', 'accel_y_mps2', 'Robopico/sensors/accelerometer/accel_y_mps2', '-9.80665', -9.80665],
  ['terrain', 'accel_z_mps2', 'Robopico/sensors/accelerometer/accel_z_mps2', '0.0', 0],
];

export function receiverSnapshot(overrides = {}) {
  const receivedAt = Object.hasOwn(overrides, 'last_received_at')
    ? overrides.last_received_at : Date.now() / 1000;
  return {
    status: 'MQTT subscribed',
    mqtt_topic: WIRE_TOPIC,
    receiver_id: 'test-receiver-a',
    received_packets: 15,
    rejected_packets: 0,
    last_received_at: receivedAt,
    messages: EDR16_FIELDS.map(([, , topic, payload], index) => ({
      topic, payload, sequence: index + 1, received_at: receivedAt,
    })),
    ...overrides,
  };
}

export function replaceReading(snapshot, topic, payload, sequence = 18, receivedAt = Date.now() / 1000) {
  return {
    ...snapshot,
    received_packets: snapshot.received_packets + 1,
    last_received_at: receivedAt,
    messages: [
      ...snapshot.messages.filter(message => message.topic !== topic),
      { topic, payload, sequence, received_at: receivedAt },
    ].sort((left, right) => left.sequence - right.sequence),
  };
}
