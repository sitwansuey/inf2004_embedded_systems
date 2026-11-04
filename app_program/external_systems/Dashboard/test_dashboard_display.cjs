/* Validate the real frontend model against an actual Python receiver snapshot. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');

async function main() {
  const snapshot = JSON.parse(fs.readFileSync(0, 'utf8'));
  const { createTelemetryStore, topicBindings, WIRE_TOPIC } = await import(pathToFileURL(path.join(__dirname, 'src', 'telemetry.js')).href);
  const store = createTelemetryStore();
  assert.equal(WIRE_TOPIC, snapshot.mqtt_topic);
  store.ingestSnapshot(snapshot);
  const state = store.getSnapshot();
  for (const message of snapshot.messages) {
    const binding = topicBindings.find(item => item.topic === message.topic);
    assert.ok(binding, `Missing field binding for ${message.topic}`);
    const group = binding.key ?? binding.group;
    let expected;
    try { expected = JSON.parse(message.payload); } catch { expected = message.payload; }
    assert.equal(state.packet[group][binding.field], expected, message.topic);
    if (typeof expected === 'number') {
      const history = store.getSource(group, binding.field).getHistory();
      assert.equal(history.at(-1).value, expected, `Missing numeric source for ${message.topic}`);
      assert.equal(history.at(-1).channel, `${group}.${binding.field}`);
    }
  }
  assert.equal(state.packet.terrain?.heading_deg, undefined, 'Untransmitted fields remain unavailable');
  assert.equal(state.packet.motion?.left_speed_mps, undefined, 'Counts must not invent calibrated speed');
  const source = store.getSource('motion', 'encoder_left');
  const before = source.getHistory().length;
  store.ingestSnapshot(snapshot);
  assert.equal(source.getHistory().length, before, 'Polling an unchanged snapshot must not duplicate readings');
  console.log(`PASS: ${snapshot.messages.length} decoded fields mapped to the correct cards and NexusUI sources`);
}

main().catch(error => { console.error(error); process.exitCode = 1; });
