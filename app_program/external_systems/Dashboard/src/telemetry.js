// EDR16 is decoded by Python. These names route its local display fields;
// they are never additional broker subscriptions.
export const WIRE_TOPIC = 'Robopico/sensors/edr16/v1';
const definitions = [
  ['navigation', 'Left IR', '03', 'Raw left sensor reading.', ['ir_left']],
  ['navigation', 'Right IR', '03', 'Raw right sensor reading.', ['ir_right']],
  ['navigation', 'Barcode IR', '03', 'Barcode sensor reading and decoded data.', ['ir_barcode', 'barcode']],
  ['motion', 'Left wheel encoder', '02', 'Packed counts wrap at 4096; speed requires calibrated wheel feedback.', ['encoder_left', 'left_speed_mps']],
  ['motion', 'Right wheel encoder', '02', 'Packed counts wrap at 4096; speed requires calibrated wheel feedback.', ['encoder_right', 'right_speed_mps']],
  ['terrain', 'Accelerometer', '04', 'Acceleration and impact information from the GY-511.', ['accel_x_mps2', 'accel_y_mps2', 'accel_z_mps2', 'impact', 'imu_status']],
  ['terrain', 'Magnetometer', '04', 'Heading uses the magnetometer. Pitch, roll and turn rate are derived estimates; the GY-511 has no gyroscope.', ['mag_x_ut', 'mag_y_ut', 'mag_z_ut', 'heading_deg', 'pitch_deg', 'roll_deg', 'turn_rate_dps', 'valid']],
  ['obstacle', 'Ultrasonic Ranger', '05', 'Range and obstacle geometry from the ultrasonic scan.', ['distance_cm', 'width_cm', 'depth_cm', 'left_clearance_cm', 'right_clearance_cm']],
  ['terrain', 'Terrain & hump analysis', '04', 'Derived motion and hump estimates.', ['motion_class', 'hump_state', 'hump_height_cm', 'run_max_height_cm', 'valid']],
  ['obstacle', 'Servo Motor', '05', 'Scan actuator position and avoidance decisions.', ['state', 'servo_angle_deg', 'bypass_side', 'route']],
  ['motion', 'Motor control', '02', 'Commanded PWM output is separate from measured wheel motion.', ['state', 'direction', 'power_percent', 'fault_flags', 'distance_m', 'last_error', 'last_command_ms']],
  ['system', 'Vehicle controller', '05', 'Mission state, safety and telemetry timing.', ['state', 'fault_code', 'fault_message', 'safety_state', 'uptime_ms', 'telemetry_period_ms', 'latency_ms', 'sequence']],
];
const labels = {
  ir_left: ['Raw reading'], ir_right: ['Raw reading'], ir_barcode: ['Raw reading'],
  encoder_left: ['Pulse count (mod 4096)'], encoder_right: ['Pulse count (mod 4096)'],
  left_speed_mps: ['Wheel speed', 'm/s'], right_speed_mps: ['Wheel speed', 'm/s'],
  accel_x_mps2: ['Acceleration X', 'm/s²'], accel_y_mps2: ['Acceleration Y', 'm/s²'], accel_z_mps2: ['Acceleration Z', 'm/s²'],
  mag_x_ut: ['Magnetic field X', 'µT'], mag_y_ut: ['Magnetic field Y', 'µT'], mag_z_ut: ['Magnetic field Z', 'µT'],
  heading_deg: ['Heading / yaw alias', '°'], pitch_deg: ['Pitch', '°'], roll_deg: ['Roll', '°'], turn_rate_dps: ['Estimated turn rate', '°/s'],
  distance_cm: ['Obstacle distance', 'cm'], width_cm: ['Width', 'cm'], depth_cm: ['Depth', 'cm'], left_clearance_cm: ['Left clearance', 'cm'], right_clearance_cm: ['Right clearance', 'cm'],
  hump_height_cm: ['Current hump peak', 'cm'], run_max_height_cm: ['Run maximum height', 'cm'], servo_angle_deg: ['Servo angle', '°'],
  power_percent: ['Commanded power', '%'], fault_flags: ['Motor fault bits'], distance_m: ['Travelled distance', 'm'], last_command_ms: ['Command timestamp', 'ms'],
  uptime_ms: ['Robot uptime', 'ms'], telemetry_period_ms: ['Telemetry period', 'ms'], latency_ms: ['Reported delivery latency', 'ms'], valid: ['Estimate validity'],
};
const numericFields = new Set([
  'ir_left', 'ir_right', 'ir_barcode', 'encoder_left', 'encoder_right', 'left_speed_mps', 'right_speed_mps',
  'accel_x_mps2', 'accel_y_mps2', 'accel_z_mps2', 'mag_x_ut', 'mag_y_ut', 'mag_z_ut', 'heading_deg', 'pitch_deg', 'roll_deg', 'turn_rate_dps',
  'distance_cm', 'width_cm', 'depth_cm', 'left_clearance_cm', 'right_clearance_cm', 'hump_height_cm', 'run_max_height_cm',
  'servo_angle_deg', 'power_percent', 'fault_flags', 'distance_m', 'last_command_ms', 'uptime_ms', 'telemetry_period_ms', 'latency_ms', 'sequence',
]);

export const FIELD_GROUPS = Object.freeze(definitions.map(([id, title, owner, note, fields]) => Object.freeze({
  id, title, owner, note,
  fields: Object.freeze(fields.map(key => Object.freeze({
    key, label: labels[key]?.[0] || key.replaceAll('_', ' ').replace(/^./, character => character.toUpperCase()),
    ...(labels[key]?.[1] ? { unit: labels[key][1] } : {}), numeric: numericFields.has(key),
  }))),
})));
const sections = ['left_ir', 'right_ir', 'barcode_ir', 'left_encoder', 'right_encoder', 'accelerometer', 'magnetometer', 'ultrasonic_ranger', 'terrain', 'servo_motor', 'motors', 'vehicle_controller'];
export const topicBindings = Object.freeze(FIELD_GROUPS.flatMap((group, index) => group.fields.map(field => Object.freeze({
  topic: `Robopico/${index >= 9 ? 'Actuators' : 'sensors'}/${sections[index]}/${field.key}`,
  key: group.id, field: field.key,
}))));
export const eventTopic = 'Robopico/Actuators/vehicle_controller/events';
const bindings = new Map(topicBindings.map(binding => [binding.topic, binding]));
const knownFields = new Map();
for (const group of FIELD_GROUPS) {
  if (!knownFields.has(group.id)) knownFields.set(group.id, new Set());
  for (const field of group.fields) knownFields.get(group.id).add(field.key);
}
const forbidden = new Set(['__proto__', 'prototype', 'constructor']);

function object(value, label) {
  if (!value || typeof value !== 'object' || Array.isArray(value)
      || ![Object.prototype, null].includes(Object.getPrototypeOf(value))) throw new Error(`${label} must be an object.`);
  if (Object.keys(value).some(key => forbidden.has(key))) throw new Error(`${label} contains an unsafe key.`);
}
function scalar(value, label) {
  if (value !== null && !['string', 'number', 'boolean'].includes(typeof value)) throw new Error(`${label} must be a scalar value.`);
  if (typeof value === 'number' && !Number.isFinite(value)) throw new Error(`${label} must be finite.`);
  return value;
}
function fieldValue(group, field, value) {
  if (!knownFields.get(group)?.has(field)) throw new Error(`Unknown telemetry field: ${group}.${field}.`);
  scalar(value, `${group}.${field}`);
  if (value === null || !numericFields.has(field)) return value;
  if (typeof value !== 'number') throw new Error(`${group}.${field} must be a finite number.`);
  const packedCount = /^(ir_left|ir_right|ir_barcode|encoder_left|encoder_right)$/.test(field);
  const max = packedCount || field === 'distance_cm' ? 4095 : field === 'servo_angle_deg' ? 180 : field === 'power_percent' ? 100 : field === 'fault_flags' ? 127 : null;
  if (max !== null && (value < 0 || value > max)) throw new Error(`${group}.${field} must be in 0..${max}.`);
  if ((packedCount || field === 'fault_flags') && !Number.isInteger(value)) throw new Error(`${group}.${field} must be an integer.`);
  if (/^(last_command_ms|uptime_ms|telemetry_period_ms|latency_ms|sequence)$/.test(field) && value < 0) throw new Error(`${group}.${field} must be nonnegative.`);
  return value;
}
function safeEvent(value) {
  object(value, 'Event');
  const event = {};
  for (const key of Object.keys(value)) {
    if (!['time', 'source', 'level', 'message'].includes(key)) throw new Error(`Unknown event field: ${key}.`);
    event[key] = scalar(value[key], `Event ${key}`);
  }
  return event;
}

export function validateTelemetry(packet) {
  object(packet, 'Telemetry');
  const clean = {};
  for (const group of Object.keys(packet)) {
    if (group === 'events') {
      if (!Array.isArray(packet.events) || packet.events.length > 100) throw new Error('events must be an array of at most 100 objects.');
      clean.events = packet.events.map(safeEvent);
      continue;
    }
    if (!knownFields.has(group)) throw new Error(`Unknown telemetry group: ${group}.`);
    object(packet[group], group);
    clean[group] = {};
    for (const field of Object.keys(packet[group])) clean[group][field] = fieldValue(group, field, packet[group][field]);
  }
  return clean;
}

export const DEMO_PACKET = Object.freeze(validateTelemetry({
  motion: { state: 'MOVING', direction: 'FORWARD', power_percent: 35, left_speed_mps: 0.24, right_speed_mps: 0.25, distance_m: 3.8, encoder_left: 420, encoder_right: 424, last_error: 0, last_command_ms: 12000 },
  navigation: { ir_left: 650, ir_barcode: 2100, ir_right: 720, barcode: 'RIGHT' },
  terrain: { valid: true, heading_deg: 92, pitch_deg: 4.2, roll_deg: 0.8, turn_rate_dps: 1.4, accel_x_mps2: 0.1, accel_y_mps2: 0.02, accel_z_mps2: 9.8, motion_class: 'CLIMBING', impact: false, hump_state: 'ASCENDING', hump_height_cm: 2.1, run_max_height_cm: 3.6, imu_status: 'OK' },
  obstacle: { state: 'SCANNING', distance_cm: 32, width_cm: 14, depth_cm: 20, left_clearance_cm: 40, right_clearance_cm: 18, servo_angle_deg: 60, bypass_side: 'LEFT', route: 'Pending controller' },
  system: { state: 'LINE_TRACK', fault_code: 0, fault_message: 'None', safety_state: 'NORMAL', uptime_ms: 12500, telemetry_period_ms: 10, latency_ms: 24, sequence: 1250 },
  events: [{ time: '00:12.4', source: 'Buddy 4', level: 'INFO', message: 'Suspected hump; reduce speed requested.' }, { time: '00:12.5', source: 'Buddy 5', level: 'INFO', message: 'Obstacle scan started.' }],
}));

function freezeState(state) {
  for (const [group, fields] of Object.entries(state.packet)) {
    if (group === 'events') fields.forEach(Object.freeze);
    Object.freeze(fields);
  }
  Object.freeze(state.packet);
  Object.freeze(state.receivedAt);
  return Object.freeze(state);
}
function initialState(epoch = 0) {
  return { packet: {}, receivedAt: {}, mode: 'Live', status: 'Connecting to MQTT broker', error: null,
    receiverId: null, lastSequence: 0, receivedPackets: 0, rejectedPackets: 0, lastReceivedAt: null, epoch };
}
function sourceStatus(state) {
  if (state.mode !== 'Live') return 'disconnected';
  if (state.error || /reject|fail|error|unavailable/i.test(state.status)) return 'error';
  if (/disconnect/i.test(state.status)) return 'disconnected';
  return state.status === 'MQTT subscribed' ? 'connected' : 'connecting';
}
export function readingState(state, group, field, nowMs = Date.now(), staleAfterMs = 5000) {
  if (state.packet[group]?.[field] == null) return 'missing';
  if (state.mode !== 'Live') return 'snapshot';
  const timestamp = state.receivedAt[`${group}.${field}`];
  if (state.error || /disconnect|reject|fail|error|unavailable|stale|connecting|subscribing/i.test(state.status)
      || timestamp == null || nowMs - timestamp > staleAfterMs) return 'stale';
  return 'current';
}

export function createTelemetryStore() {
  let state = freezeState(initialState());
  const listeners = new Set();
  const sources = new Map();
  function getSource(group, field) {
    if (!knownFields.get(group)?.has(field)) throw new Error(`Unknown telemetry field: ${group}.${field}.`);
    const channel = `${group}.${field}`;
    if (!sources.has(channel)) {
      const history = [], subscribers = new Set();
      let destroyed = false;
      const source = {
        subscribe(callback) { if (!destroyed) subscribers.add(callback); return () => subscribers.delete(callback); },
        getHistory: () => history.slice(),
        get status() { return destroyed ? 'disconnected' : sourceStatus(state); },
        destroy() { destroyed = true; history.length = 0; subscribers.clear(); },
      };
      sources.set(channel, { source, push(sample) {
        if (destroyed) return;
        const point = Object.freeze(sample);
        history.push(point);
        if (history.length > 240) history.shift();
        for (const subscriber of [...subscribers]) subscriber(point);
      } });
    }
    return sources.get(channel).source;
  }
  function commit(next, samples = [], clearSources = false) {
    if (clearSources) { for (const { source } of sources.values()) source.destroy(); sources.clear(); }
    state = freezeState(next);
    for (const sample of samples) {
      const [group, field] = sample.channel.split('.');
      getSource(group, field);
      sources.get(sample.channel).push(sample);
    }
    for (const listener of [...listeners]) listener();
  }
  function stageReading(next, topic, payload, timestamp, samples) {
    const binding = bindings.get(topic);
    if (!binding && topic !== eventTopic) return false;
    let value = payload;
    if (typeof payload === 'string') { try { value = JSON.parse(payload); } catch { value = payload; } }
    if (topic === eventTopic) {
      next.packet.events = [...(next.packet.events || []), safeEvent(value)].slice(-100);
    } else {
      const { key: group, field } = binding;
      value = fieldValue(group, field, value);
      next.packet[group] = { ...(next.packet[group] || {}), [field]: value };
      const channel = `${group}.${field}`;
      next.receivedAt[channel] = timestamp;
      if (typeof value === 'number') samples.push({ timestamp, value, channel });
    }
    return true;
  }
  function resumeLive() { commit(initialState(state.epoch + 1), [], true); }
  return {
    subscribe(callback) { listeners.add(callback); return () => listeners.delete(callback); },
    getSnapshot: () => state,
    getSource,
    ingestSnapshot(snapshot, nowMs = Date.now()) {
      if (state.mode !== 'Live') return false;
      object(snapshot, 'Receiver snapshot');
      if (snapshot.mqtt_topic !== WIRE_TOPIC) throw new Error('Receiver snapshot uses an unexpected MQTT topic.');
      if (typeof snapshot.receiver_id !== 'string' || !snapshot.receiver_id) throw new Error('Receiver identity is missing.');
      if (!Array.isArray(snapshot.messages)) throw new Error('Receiver messages must be an array.');
      if (typeof snapshot.status !== 'string') throw new Error('Receiver status must be text.');
      for (const name of ['received_packets', 'rejected_packets']) {
        if (!Number.isSafeInteger(snapshot[name]) || snapshot[name] < 0) throw new Error(`Invalid receiver ${name}.`);
      }
      if (snapshot.last_received_at != null && (!Number.isFinite(snapshot.last_received_at) || snapshot.last_received_at < 0)) throw new Error('Invalid receiver reception time.');
      const changedReceiver = state.receiverId !== snapshot.receiver_id;
      const next = changedReceiver ? initialState(state.epoch + 1) : { ...state, packet: { ...state.packet }, receivedAt: { ...state.receivedAt } };
      const samples = [];
      next.receiverId = snapshot.receiver_id;
      next.status = snapshot.status;
      next.error = null;
      next.receivedPackets = snapshot.received_packets;
      next.rejectedPackets = snapshot.rejected_packets;
      next.lastReceivedAt = snapshot.last_received_at == null ? null : snapshot.last_received_at * 1000;
      const messages = [...snapshot.messages].sort((a, b) => a?.sequence - b?.sequence);
      for (const message of messages) {
        object(message, 'Receiver message');
        if (!Number.isSafeInteger(message.sequence) || message.sequence < 1 || typeof message.topic !== 'string') throw new Error('Invalid receiver message.');
        if (message.sequence <= next.lastSequence) continue;
        if (message.received_at != null && (!Number.isFinite(message.received_at) || message.received_at < 0)) throw new Error('Invalid field reception time.');
        const timestamp = message.received_at != null ? message.received_at * 1000 : next.lastReceivedAt ?? nowMs;
        stageReading(next, message.topic, message.payload, timestamp, samples);
        next.lastSequence = message.sequence;
      }
      commit(next, samples, changedReceiver);
      return true;
    },
    setTelemetry(packet, mode = 'Imported', nowMs = Date.now()) {
      if (!['Live', 'Demo', 'Imported'].includes(mode)) throw new Error('Unknown telemetry mode.');
      const clean = validateTelemetry(packet);
      const next = { ...initialState(state.epoch + 1), packet: clean, mode,
        status: mode === 'Live' ? 'MQTT data supplied' : `${mode} snapshot`, lastReceivedAt: nowMs };
      const samples = [];
      for (const [group, fields] of Object.entries(clean)) {
        if (group === 'events') continue;
        for (const [field, value] of Object.entries(fields)) {
          if (value === null) continue;
          const channel = `${group}.${field}`;
          next.receivedAt[channel] = nowMs;
          if (typeof value === 'number') samples.push({ timestamp: nowMs, value, channel });
        }
      }
      commit(next, samples, true);
    },
    receiveMqtt(topic, payload, nowMs = Date.now()) {
      if (!bindings.has(topic) && topic !== eventTopic) return false;
      const changedMode = state.mode !== 'Live';
      const next = changedMode ? initialState(state.epoch + 1) : { ...state, packet: { ...state.packet }, receivedAt: { ...state.receivedAt } };
      const samples = [];
      stageReading(next, topic, payload, nowMs, samples);
      next.lastReceivedAt = nowMs;
      next.error = null;
      commit(next, samples, changedMode);
      return true;
    },
    reset: resumeLive,
    resumeLive,
    setError(message) { commit({ ...state, error: String(message), status: 'Dashboard receiver unavailable' }); },
  };
}
