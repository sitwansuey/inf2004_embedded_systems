import { test, expect } from '@playwright/test';
import { EDR16_FIELDS, receiverSnapshot, replaceReading } from '../telemetry-fixture.mjs';

const field = (page, name) => page.locator(`.field-row[data-field="${name}"]`).first();
const value = (page, name) => field(page, name).locator('[data-value]');

async function receiver(page, initial = receiverSnapshot()) {
  let snapshot = initial, available = true, requests = 0;
  await page.route('**/api/telemetry', route => {
    requests += 1;
    return available
      ? route.fulfill({ json: snapshot })
      : route.fulfill({ status: 503, json: { error: 'Receiver unavailable' } });
  });
  return {
    set: next => { snapshot = next; },
    available: next => { available = next; },
    requests: () => requests,
  };
}

test('all 17 decoded fields, incremental updates and receiver restart reach the correct cards', async ({ page }) => {
  let snapshot = receiverSnapshot();
  const broker = await receiver(page, snapshot);
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'PicoCar telemetry.' })).toBeVisible();
  await expect(page.locator('#connection')).toHaveText('MQTT subscribed');
  for (const [group, key, , , expected] of EDR16_FIELDS) {
    const reading = value(page, `${group}.${key}`);
    if (typeof expected === 'number') {
      await expect.poll(async () => Number.parseFloat(await reading.textContent())).toBeCloseTo(expected, 2);
    } else await expect(reading).toHaveText(expected);
  }
  await expect(value(page, 'terrain.heading_deg')).toHaveText('—');
  await expect(value(page, 'motion.left_speed_mps')).toHaveText('—');
  await expect(page.locator('#event-count')).toHaveText('0 events');
  await page.screenshot({ path: 'test-results/dashboard-desktop.png', fullPage: true });

  snapshot = replaceReading(snapshot, EDR16_FIELDS[0][2], '101');
  broker.set(snapshot);
  await expect(value(page, 'motion.encoder_left')).toHaveText('101');
  await expect(value(page, 'motion.encoder_right')).toHaveText('200');
  await expect(value(page, 'navigation.ir_left')).toHaveText('0');

  broker.set(receiverSnapshot({
    receiver_id: 'test-receiver-restarted', received_packets: 1,
    messages: [{ topic: EDR16_FIELDS[2][2], payload: '45', sequence: 1, received_at: Date.now() / 1000 }],
  }));
  await expect(value(page, 'obstacle.servo_angle_deg')).toHaveText('45');
  await expect(value(page, 'motion.encoder_left')).toHaveText('—');
  await expect(value(page, 'motion.power_percent')).toHaveText('—');
  expect(errors).toEqual([]);
});

test('old fields and unavailable receivers are marked stale, then recover on fresh readings', async ({ page }) => {
  let snapshot = receiverSnapshot({ last_received_at: Date.now() / 1000 - 30 });
  const broker = await receiver(page, snapshot);
  await page.goto('/');
  await expect(page.locator('#connection')).toHaveText('No recent packets');
  await expect(field(page, 'motion.encoder_left')).toHaveClass(/reading-stale/);
  await expect(value(page, 'motion.encoder_left')).toHaveText('100');

  snapshot = replaceReading(snapshot, EDR16_FIELDS[0][2], '102');
  broker.set(snapshot);
  await expect(page.locator('#connection')).toHaveText('MQTT subscribed');
  await expect(field(page, 'motion.encoder_left')).toHaveClass(/reading-current/);
  await expect(field(page, 'motion.encoder_right')).toHaveClass(/reading-stale/);

  broker.set({ ...snapshot, status: 'MQTT disconnected; reconnecting' });
  await expect(page.locator('#connection')).toContainText('disconnected');
  await expect(field(page, 'motion.encoder_left')).toHaveClass(/reading-stale/);
  broker.available(false);
  await expect(page.locator('#connection')).toHaveText('Receiver unavailable');
  await expect(page.locator('#notice')).toContainText('Last received values are historical');
  await expect(value(page, 'motion.encoder_left')).toHaveText('102');

  broker.available(true);
  broker.set(replaceReading(snapshot, EDR16_FIELDS[0][2], '103', 19));
  await expect(page.locator('#connection')).toHaveText('MQTT subscribed');
  await expect(value(page, 'motion.encoder_left')).toHaveText('103');
  await expect(field(page, 'motion.encoder_left')).toHaveClass(/reading-current/);
});

test('demo and imported snapshots remain explicit through polling and render unsafe text literally', async ({ page }) => {
  let snapshot = receiverSnapshot();
  const broker = await receiver(page, snapshot);
  await page.goto('/');
  await expect(value(page, 'motion.power_percent')).toHaveText('30');
  await page.getByRole('button', { name: 'Load demo', exact: true }).click();
  await expect(page.locator('#connection')).toHaveText('Demo snapshot');
  const demoPower = await value(page, 'motion.power_percent').textContent();
  const beforeDemoPolls = broker.requests();
  snapshot = replaceReading(snapshot, EDR16_FIELDS[12][2], '88');
  broker.set(snapshot);
  await expect.poll(broker.requests).toBeGreaterThan(beforeDemoPolls + 1);
  await expect(value(page, 'motion.power_percent')).toHaveText(demoPower);
  await expect(page.locator('#notice')).toContainText('Live updates are paused');

  const unsafeText = '<img src=x onerror="window.__telemetryXss = true">';
  await page.locator('#import').setInputFiles({ name: 'snapshot.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify({
    motion: { power_percent: 17 },
    system: { state: unsafeText },
    events: [{ time: '00:01', source: 'Imported test', level: 'WARN', message: unsafeText }],
  })) });
  await expect(page.locator('#connection')).toHaveText('Imported snapshot');
  await expect(value(page, 'motion.power_percent')).toHaveText('17');
  await expect(value(page, 'system.state')).toHaveText(unsafeText);
  await expect(page.locator('#history')).toContainText(unsafeText);
  expect(await page.evaluate(() => window.__telemetryXss)).toBeUndefined();
  await expect(page.locator('#history img, #panels img')).toHaveCount(0);
  const beforeImportPolls = broker.requests();
  snapshot = replaceReading(snapshot, EDR16_FIELDS[12][2], '99', 19);
  broker.set(snapshot);
  await expect.poll(broker.requests).toBeGreaterThan(beforeImportPolls + 1);
  await expect(value(page, 'motion.power_percent')).toHaveText('17');

  await page.locator('#import').setInputFiles({ name: 'bad.json', mimeType: 'application/json', buffer: Buffer.from('{"motion":[]}') });
  await expect(page.getByRole('alert')).toContainText('Import failed:');
  await expect(value(page, 'motion.power_percent')).toHaveText('17');
  await page.getByRole('button', { name: 'Resume live', exact: true }).click();
  await expect(page.locator('#connection')).toHaveText('MQTT subscribed');
  await expect(value(page, 'motion.power_percent')).toHaveText('99');
  await expect(value(page, 'system.state')).toHaveText('—');
  await expect(page.getByRole('alert')).toHaveCount(0);
});

test('portrait layout retains readable cards, subsystem filtering and keyboard access without overflow', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await receiver(page);
  await page.goto('/');
  await expect(value(page, 'motion.power_percent')).toHaveText('30');
  await expect.poll(() => page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  await page.screenshot({ path: 'test-results/dashboard-mobile.png', fullPage: true });
  await page.keyboard.press('Tab');
  await expect(page.getByRole('link', { name: 'Skip to telemetry' })).toBeFocused();
  await page.getByRole('tab', { name: 'Sensors' }).click();
  await page.getByLabel('Show subsystem', { exact: true }).selectOption('motion');
  await expect(page.locator('.sensor-panel')).toHaveCount(3);
  await expect(page.getByRole('heading', { name: 'Left wheel encoder', exact: true })).toBeVisible();
  await expect(page.getByRole('heading', { name: 'Accelerometer', exact: true })).toHaveCount(0);
  await page.getByLabel('Show subsystem', { exact: true }).selectOption('all');
  await page.locator('#import').setInputFiles({ name: 'long-text.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify({
    system: { state: 'A'.repeat(300) }, events: [{ message: 'Long event '.repeat(50) }],
  })) });
  await expect(page.locator('#connection')).toHaveText('Imported snapshot');
  await expect.poll(() => page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
  await page.screenshot({ path: 'test-results/dashboard-mobile-long-text.png', fullPage: true });
  await page.getByRole('tab', { name: 'Events' }).click();
  await expect(page.locator('#history')).toContainText('Long event');
  await expect.poll(() => page.evaluate(() => document.documentElement.scrollWidth <= innerWidth)).toBe(true);
});

test('tabs switch views, follow the URL hash, support arrow keys and open filtered subsystems', async ({ page }) => {
  await receiver(page);
  await page.goto('/#sensors');
  const tab = name => page.getByRole('tab', { name, exact: true });
  await expect(tab('Sensors')).toHaveAttribute('aria-selected', 'true');
  await expect(page.locator('#panel-sensors')).toBeVisible();
  await expect(page.locator('#panel-overview')).toBeHidden();

  await tab('Live views').click();
  await expect(page).toHaveURL(/#live$/);
  await expect(page.getByRole('heading', { name: 'Proximity radar' })).toBeVisible();
  await expect(page.getByRole('heading', { name: 'Left wheel encoder', exact: true })).toBeHidden();

  await tab('Live views').press('ArrowRight');
  await expect(tab('Trends')).toBeFocused();
  await expect(tab('Trends')).toHaveAttribute('aria-selected', 'true');
  await expect(page.locator('#panel-trends canvas')).toHaveCount(5);
  await page.keyboard.press('End');
  await expect(tab('Events')).toHaveAttribute('aria-selected', 'true');
  await expect(page.locator('#event-count')).toBeVisible();
  await page.keyboard.press('Home');
  await expect(tab('Overview')).toHaveAttribute('aria-selected', 'true');
  await expect(page).toHaveURL(/#overview$/);

  await page.getByRole('button', { name: /^Show Navigation sensors \(3 of 4 readings supplied\)$/ }).click();
  await expect(tab('Sensors')).toHaveAttribute('aria-selected', 'true');
  await expect(page.getByLabel('Show subsystem', { exact: true })).toHaveValue('navigation');
  await expect(page.locator('.sensor-panel')).toHaveCount(3);
  await expect(page.getByRole('heading', { name: 'Barcode IR', exact: true })).toBeVisible();
});
