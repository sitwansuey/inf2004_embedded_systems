import test from 'node:test';
import assert from 'node:assert/strict';
import { createTelemetryStore, DEMO_PACKET, readingState, validateTelemetry, WIRE_TOPIC } from '../src/telemetry.js';
import { EDR16_FIELDS, receiverSnapshot, replaceReading } from './telemetry-fixture.mjs';

const NOW = 1_000_000;
const initial = () => receiverSnapshot({ last_received_at: NOW / 1000 });
const LEFT = EDR16_FIELDS[0][2];
const SERVO = EDR16_FIELDS[2][2];

test('the 15-record EDR16 snapshot maps all 17 fields, preserves zero and supplies actual numeric histories', () => {
  const store = createTelemetryStore();
  const snapshot = initial();
  assert.equal(snapshot.mqtt_topic, WIRE_TOPIC);
  assert.equal(snapshot.messages.length, 17);
  store.ingestSnapshot(snapshot, NOW);
  const state = store.getSnapshot();
  assert.equal(state.receivedPackets, 15);
  assert.equal(state.lastSequence, 17);
  for (const [group, field, , , expected] of EDR16_FIELDS) {
    assert.equal(state.packet[group][field], expected, `${group}.${field}`);
    assert.equal(state.receivedAt[`${group}.${field}`], NOW);
    const source = store.getSource(group, field);
    assert.equal(source.status, 'connected');
    assert.deepEqual(source.getHistory(), typeof expected === 'number'
      ? [{ timestamp: NOW, value: expected, channel: `${group}.${field}` }] : []);
  }
  assert.equal(state.packet.motion.left_speed_mps, undefined, 'Encoder counts do not imply calibrated speed');
  assert.equal(state.packet.terrain.heading_deg, undefined, 'Magnetic axes do not imply a measured heading');
  assert.equal(state.packet.events, undefined, 'EDR16 carries no event text');
  assert.equal(readingState(state, 'navigation', 'ir_left', NOW), 'current');
  assert.equal(readingState(state, 'terrain', 'heading_deg', NOW), 'missing');
});

test('polling duplicate sequences does not create samples or refresh unrelated field ages', () => {
  const store = createTelemetryStore();
  const snapshot = initial();
  store.ingestSnapshot(snapshot, NOW);
  const left = store.getSource('motion', 'encoder_left');
  const right = store.getSource('motion', 'encoder_right');
  const samples = [];
  const unsubscribe = left.subscribe(sample => samples.push(sample));
  store.ingestSnapshot(snapshot, NOW + 10_000);
  assert.equal(left.getHistory().length, 1);
  assert.equal(store.getSnapshot().receivedAt['motion.encoder_left'], NOW);
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW + 10_000), 'stale');

  const next = replaceReading(snapshot, LEFT, '101', 18, (NOW + 10_000) / 1000);
  store.ingestSnapshot(next, NOW + 10_000);
  assert.equal(store.getSource('motion', 'encoder_left'), left, 'Sources are stable within a receiver session');
  assert.deepEqual(left.getHistory().map(sample => sample.value), [100, 101]);
  assert.deepEqual(right.getHistory().map(sample => sample.value), [200]);
  assert.equal(samples.length, 1);
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW + 10_000), 'current');
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_right', NOW + 10_000), 'stale');
  unsubscribe();
  store.ingestSnapshot(replaceReading(next, LEFT, '102', 19, (NOW + 11_000) / 1000), NOW + 11_000);
  assert.equal(samples.length, 1, 'Unsubscribed charts do not receive further samples');
});

test('receiver restart clears old values, histories and subscriptions before accepting a lower sequence', () => {
  const store = createTelemetryStore();
  store.ingestSnapshot(initial(), NOW);
  const old = store.getSource('motion', 'encoder_left');
  let oldSamples = 0;
  old.subscribe(() => oldSamples++);
  const epoch = store.getSnapshot().epoch;
  store.ingestSnapshot(receiverSnapshot({
    receiver_id: 'replacement-receiver', received_packets: 1,
    last_received_at: (NOW + 1000) / 1000,
    messages: [{ topic: SERVO, payload: '45', sequence: 1, received_at: (NOW + 1000) / 1000 }],
  }), NOW + 1000);
  assert.equal(store.getSnapshot().epoch, epoch + 1);
  assert.equal(store.getSnapshot().lastSequence, 1);
  assert.equal(store.getSnapshot().packet.motion?.encoder_left, undefined);
  assert.equal(store.getSnapshot().packet.obstacle.servo_angle_deg, 45);
  assert.deepEqual(old.getHistory(), []);
  assert.equal(old.status, 'disconnected');
  const fresh = store.getSource('motion', 'encoder_left');
  assert.notEqual(fresh, old);
  assert.deepEqual(fresh.getHistory(), []);
  store.receiveMqtt(LEFT, '2', NOW + 2000);
  assert.deepEqual(fresh.getHistory().map(sample => sample.value), [2]);
  assert.equal(oldSamples, 0);
});

test('source histories remain bounded, copied and chronological during long sessions', () => {
  const store = createTelemetryStore();
  store.ingestSnapshot(initial(), NOW);
  const source = store.getSource('motion', 'encoder_left');
  for (let index = 1; index <= 300; index++) store.receiveMqtt(LEFT, String(index), NOW + index);
  const history = source.getHistory();
  assert.equal(history.length, 240);
  assert.equal(history[0].value, 61);
  assert.equal(history.at(-1).value, 300);
  assert.ok(history.every((sample, index) => index === 0 || sample.timestamp > history[index - 1].timestamp));
  history.pop();
  assert.equal(source.getHistory().length, 240, 'Consumers cannot mutate the stored history array');
  assert.throws(() => { history[0].value = -1; }, TypeError);
  source.destroy();
  store.receiveMqtt(LEFT, '301', NOW + 301);
  assert.equal(source.status, 'disconnected');
  assert.deepEqual(source.getHistory(), []);
});

test('disconnect and HTTP failures retain historical values while freshness and connection status recover independently', () => {
  const store = createTelemetryStore();
  const snapshot = initial();
  store.ingestSnapshot(snapshot, NOW);
  const source = store.getSource('motion', 'encoder_left');
  store.ingestSnapshot({ ...snapshot, status: 'MQTT disconnected; reconnecting' }, NOW + 1000);
  assert.equal(source.status, 'disconnected');
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW + 1000), 'stale');
  store.setError('Dashboard receiver unavailable');
  assert.equal(source.status, 'error');
  assert.equal(store.getSnapshot().packet.motion.encoder_left, 100);
  store.ingestSnapshot(snapshot, NOW + 10_000);
  assert.equal(source.status, 'connected');
  assert.equal(store.getSnapshot().error, null);
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW + 10_000), 'stale', 'Transport recovery alone does not refresh a measurement');
  store.ingestSnapshot(replaceReading(snapshot, LEFT, '0', 18, (NOW + 10_000) / 1000), NOW + 10_000);
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW + 10_000), 'current');
  assert.equal(store.getSnapshot().packet.motion.encoder_left, 0);
});

test('demo and imported modes resist polling, validate atomically and clear preview data on resume', () => {
  const store = createTelemetryStore();
  store.ingestSnapshot(initial(), NOW);
  const liveSource = store.getSource('motion', 'encoder_left');
  store.setTelemetry(DEMO_PACKET, 'Demo', NOW + 1000);
  assert.equal(liveSource.status, 'disconnected');
  assert.deepEqual(liveSource.getHistory(), []);
  const demo = store.getSnapshot();
  assert.equal(store.ingestSnapshot(initial(), NOW + 2000), false);
  assert.equal(store.getSnapshot(), demo);
  assert.equal(readingState(demo, 'motion', 'encoder_left', NOW + 100_000), 'snapshot');

  const packet = { motion: { encoder_left: 7, power_percent: 0 }, terrain: { valid: false }, events: [{ message: '<script>literal text</script>' }] };
  store.setTelemetry(packet, 'Imported', NOW + 3000);
  packet.motion.encoder_left = 999;
  const imported = store.getSnapshot();
  assert.equal(imported.packet.motion.encoder_left, 7, 'Imports are copied before becoming state');
  assert.equal(imported.packet.terrain.valid, false);
  assert.equal(imported.packet.events[0].message, '<script>literal text</script>');
  assert.equal(store.ingestSnapshot({ malformed: true }), false, 'Preview is unaffected by an invalid live response');
  assert.throws(() => store.setTelemetry({ motion: [] }, 'Imported'));
  assert.equal(store.getSnapshot(), imported, 'A rejected import does not replace the selected snapshot');
  store.resumeLive();
  assert.equal(store.getSnapshot().mode, 'Live');
  assert.deepEqual(store.getSnapshot().packet, {});
  assert.deepEqual(store.getSource('motion', 'encoder_left').getHistory(), []);
  store.ingestSnapshot(initial(), NOW + 4000);
  assert.equal(store.getSnapshot().packet.motion.encoder_left, 100);
  assert.equal(store.getSnapshot().packet.terrain.valid, undefined);
});

test('malformed receiver snapshots are rejected atomically without leaking partial samples', () => {
  const store = createTelemetryStore();
  const snapshot = initial();
  store.ingestSnapshot(snapshot, NOW);
  const before = store.getSnapshot();
  const source = store.getSource('motion', 'encoder_left');
  const invalid = {
    ...snapshot,
    messages: [
      { topic: LEFT, payload: '999', sequence: 18, received_at: NOW / 1000 },
      { topic: SERVO, payload: '181', sequence: 19, received_at: NOW / 1000 },
    ],
  };
  for (const bad of [invalid, { ...snapshot, mqtt_topic: 'csc02/telemetry' }, { ...snapshot, received_packets: -1 }, { ...snapshot, messages: [{ topic: LEFT, payload: '1', sequence: 18, received_at: NaN }] }]) {
    assert.throws(() => store.ingestSnapshot(bad, NOW));
    assert.equal(store.getSnapshot(), before);
    assert.deepEqual(source.getHistory().map(sample => sample.value), [100]);
  }
  assert.equal(store.receiveMqtt('Robopico/sensors/packed16/v1', '999'), false);
  store.ingestSnapshot({ ...snapshot, messages: [{ topic: 'unknown/field', payload: '123', sequence: 18 }] }, NOW);
  assert.equal(store.getSnapshot().lastSequence, 18);
  assert.equal(store.getSnapshot().packet.motion.encoder_left, 100);
});

test('imports reject unsafe structures and invalid numeric readings while retaining valid unavailable fields', () => {
  const invalid = [
    [], null, { motion: [] }, { motion: { encoder_left: {} } },
    { motion: { encoder_left: -1 } }, { motion: { encoder_left: 4096 } }, { motion: { encoder_left: 1.5 } },
    { motion: { power_percent: 101 } }, { motion: { power_percent: '30' } }, { motion: { fault_flags: 128 } },
    { obstacle: { servo_angle_deg: 181 } }, { terrain: { accel_x_mps2: Infinity } },
    { motion: { invented_speed: 30 } }, { not_a_subsystem: {} },
    { events: Array.from({ length: 101 }, () => ({ message: 'overflow' })) },
    { events: [{ message: { html: 'nested' } }] },
    JSON.parse('{"__proto__":{"polluted":true}}'),
    JSON.parse('{"system":{"constructor":"unsafe"}}'),
  ];
  for (const packet of invalid) assert.throws(() => validateTelemetry(packet));
  assert.equal({}.polluted, undefined);
  const store = createTelemetryStore();
  store.setTelemetry({ motion: { encoder_left: null, power_percent: 0 } }, 'Imported', NOW);
  assert.equal(readingState(store.getSnapshot(), 'motion', 'encoder_left', NOW), 'missing');
  assert.deepEqual(store.getSource('motion', 'encoder_left').getHistory(), []);
  assert.deepEqual(store.getSource('motion', 'power_percent').getHistory(), [{ timestamp: NOW, value: 0, channel: 'motion.power_percent' }]);
});
