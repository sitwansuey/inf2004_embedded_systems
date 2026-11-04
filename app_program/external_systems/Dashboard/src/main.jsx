import { useCallback, useEffect, useRef, useState, useSyncExternalStore } from 'react';
import { createRoot } from 'react-dom/client';
import './dashboard.css';
import { createTelemetryStore, DEMO_PACKET, FIELD_GROUPS, readingState, WIRE_TOPIC, topicBindings, eventTopic } from './telemetry.js';
import { AccelerationPad, FaultBits, NexusDial, PALETTE, SensorBars } from './nexus.jsx';
import { HistoryPanel } from './HistoryChart.jsx';
import { ProximityRadar, VehicleOrientation } from './VehicleVisuals.jsx';
import { StatusChip, Tile } from './ui.jsx';

const store = createTelemetryStore();
const TABS = [
  { id: 'overview', label: 'Overview', intro: 'Key readings and receiver health at a glance.' },
  { id: 'live', label: 'Live views', intro: 'Each view draws the latest received reading only. It does not reconstruct scans or paths.' },
  { id: 'trends', label: 'Trends', intro: 'Received history for this browser session, plotted against reception time.' },
  { id: 'sensors', label: 'Sensors', intro: 'A dash means no reading was supplied. Readings are marked stale after 5 seconds without an update to that field.' },
  { id: 'events', label: 'Events', intro: 'The EDR16 wire format has no event text. Events appear here when included in an imported or demo snapshot.' },
];
const SUBSYSTEMS = [
  ['motion', 'Motion', '02'], ['navigation', 'Navigation', '03'], ['terrain', 'Terrain', '04'],
  ['obstacle', 'Obstacles', '05'], ['system', 'Controller', '05'],
];
const SUBSYSTEM_LABEL = Object.fromEntries(SUBSYSTEMS.map(([id, label]) => [id, label]));
const DIRECTION_ICON = { FORWARD: ['arrow', 0], RIGHT: ['arrow', 90], REVERSE: ['arrow', 180], LEFT: ['arrow', 270], PIVOT_RIGHT: ['rotate-cw', 0], PIVOT_LEFT: ['rotate-ccw', 0], STOP: ['stop', 0] };

function Icon({ name, size = 18 }) {
  const paths = {
    car: <><path d="m5 8 2-4h10l2 4M3 9h18v9H3zM5 18v3m14-3v3M6 13h2m8 0h2M3 9l-1-2m19 2 1-2" /><path d="M7 8h10" /></>,
    pulse: <path d="M2 12h5l3-8 4 16 3-8h5" />,
    upload: <path d="M12 16V3m-5 5 5-5 5 5M4 16v5h16v-5" />,
    play: <path d="m8 4 13 8-13 8z" />,
    reset: <path d="M3 11a9 9 0 1 1 2 7M3 4v7h7" />,
    arrow: <path d="M12 20V4m-7 7 7-7 7 7" />,
    'rotate-cw': <path d="M20 12a8 8 0 1 1-2.3-5.7L20 8.6M20 3v5.6h-5.6" />,
    'rotate-ccw': <path d="M4 12a8 8 0 1 0 2.3-5.7L4 8.6M4 3v5.6h5.6" />,
    stop: <rect x="6" y="6" width="12" height="12" rx="2" />,
    chevron: <path d="m9 6 6 6-6 6" />,
    list: <path d="M8 6h13M8 12h13M8 18h13M3 6h.01M3 12h.01M3 18h.01" />,
    alert: <path d="M12 3 2 20h20zM12 10v4M12 17v.5" />,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name] || paths.pulse}</svg>;
}

function valueText(value) {
  if (value === undefined || value === null) return '—';
  if (typeof value === 'boolean') return value ? 'Yes' : 'No';
  if (typeof value === 'number' && !Number.isInteger(value)) return Number(value.toFixed(3)).toString();
  return String(value);
}

function connectionStatus(state) {
  if (state.mode !== 'Live') return 'disconnected';
  if (state.error || /reject|fail|error|unavailable/i.test(state.status)) return 'error';
  if (state.status === 'MQTT subscribed') return 'connected';
  if (/disconnect/i.test(state.status)) return 'disconnected';
  return 'connecting';
}

function combinedStatus(statuses) {
  const supplied = statuses.filter(status => status !== 'missing');
  if (!supplied.length) return 'missing';
  if (supplied.includes('stale')) return 'stale';
  return supplied.includes('snapshot') ? 'snapshot' : 'current';
}

function useReceiver() {
  const [latency, setLatency] = useState(undefined);
  useEffect(() => {
    let active = true, timer, controller;
    async function poll() {
      controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 5000);
      const started = performance.now();
      try {
        const response = await fetch('/api/telemetry', { cache: 'no-store', signal: controller.signal });
        if (!response.ok) throw new Error('Receiver unavailable');
        const snapshot = await response.json();
        if (!active) return;
        store.ingestSnapshot(snapshot);
        setLatency(Math.round(performance.now() - started));
      } catch {
        if (active && store.getSnapshot().mode === 'Live') store.setError('Dashboard receiver unavailable');
      } finally {
        clearTimeout(timeout);
        if (active) timer = setTimeout(poll, 250);
      }
    }
    poll();
    return () => { active = false; clearTimeout(timer); controller?.abort(); };
  }, []);
  return latency;
}

const tabFromHash = () => {
  const id = window.location.hash.slice(1);
  return TABS.some(tab => tab.id === id) ? id : null;
};

/** Selected tab, kept in the URL hash so a reload or shared link reopens the same view. */
function useTabs() {
  const [tab, setTab] = useState(() => tabFromHash() || TABS[0].id);
  useEffect(() => {
    const follow = () => { const id = tabFromHash(); if (id) setTab(id); };
    window.addEventListener('hashchange', follow);
    return () => window.removeEventListener('hashchange', follow);
  }, []);
  const select = useCallback(id => {
    setTab(id);
    if (window.location.hash !== `#${id}`) window.history.replaceState(null, '', `#${id}`);
  }, []);
  return [tab, select];
}

function TabPanel({ id, active, intro, aside, children }) {
  return <section role="tabpanel" id={`panel-${id}`} aria-labelledby={`tab-${id}`} hidden={!active} tabIndex={0} className="tab-panel">
    <div className="panel-intro"><p>{intro}</p>{aside}</div>
    {children}
  </section>;
}

function StatCard({ state, now, group, field, title, unit, min, max, note }) {
  const value = state.packet[group]?.[field];
  const status = readingState(state, group, field, now);
  return <section className={`card stat-card reading-${status}`} aria-label={title}>
    <div className="card-header"><h2 className="card-title">{title}</h2><StatusChip status={status} /></div>
    <div className="stat-body">
      <div className="stat-copy">
        <p className="stat-value">{valueText(value)}<span className={unit === '°' ? 'deg' : undefined}>{unit}</span></p>
        <p className="stat-note">{note}</p>
      </div>
      <div className="stat-gauge">
        <NexusDial value={typeof value === 'number' ? value : undefined} min={min} max={max} color={PALETTE.cyan} size={96} />
        <span className="stat-scale">{min}–{max}{unit === '°' ? unit : ` ${unit}`}</span>
      </div>
    </div>
  </section>;
}

function MotorCard({ state, now }) {
  const motion = state.packet.motion || {};
  const status = readingState(state, 'motion', 'state', now);
  const directionStale = readingState(state, 'motion', 'direction', now) === 'stale';
  const faultStale = readingState(state, 'motion', 'fault_flags', now) === 'stale';
  const tone = motion.state == null ? 'missing' : /fault/i.test(motion.state) ? 'fault' : /moving/i.test(motion.state) ? 'moving' : 'idle';
  const [icon, rotation] = DIRECTION_ICON[motion.direction] || ['arrow', 0];
  return <section className={`card stat-card motion-card reading-${status}`} aria-label="Motor control">
    <div className="card-header"><h2 className="card-title">Motor control</h2><StatusChip status={status} /></div>
    <p className={`motion-state tone-${tone}`}><i aria-hidden="true" />{valueText(motion.state)}</p>
    <p className="motion-direction">
      <span className="direction-icon" style={{ transform: `rotate(${rotation}deg)` }}><Icon name={motion.direction == null ? 'arrow' : icon} size={13} /></span>
      {valueText(motion.direction)}{directionStale && <span className="field-state">Stale</span>}
    </p>
    <div className="fault-block">
      <div className="fault-line"><span>{faultStale ? 'Fault mask · stale' : 'Fault mask'}</span><strong>{valueText(motion.fault_flags)}</strong></div>
      <FaultBits mask={motion.fault_flags} />
    </div>
  </section>;
}

function ReceiverCard({ state, latency, connectionClass }) {
  const live = state.mode === 'Live';
  const transport = connectionStatus(state);
  const text = !live ? 'Paused for snapshot' : state.error ? 'Unavailable'
    : { connected: 'Online', connecting: 'Connecting', disconnected: 'Disconnected', error: 'Error' }[transport];
  const chip = { connected: 'current', connecting: 'connecting', preview: 'snapshot', stale: 'stale', disconnected: 'stale', error: 'error' }[connectionClass];
  return <section className="card receiver-card" aria-label="Telemetry connection" title="Latency measures the HTTP round trip to the dashboard receiver.">
    <div className="card-header"><h2 className="card-title">Receiver</h2><StatusChip status={chip}>{text}</StatusChip></div>
    <div className="tiles tiles-2">
      <Tile label="Latency">{live && !state.error && latency !== undefined ? <>{latency}<small>ms</small></> : '—'}</Tile>
      <Tile label="Wire format">EDR16<small>2 bytes</small></Tile>
      <Tile label="Decoded packets">{live ? state.receivedPackets : '—'}</Tile>
      <Tile label="Rejected packets" className={live && state.rejectedPackets ? 'stale' : ''}>{live ? state.rejectedPackets : '—'}</Tile>
      <Tile label="Endpoint">/api/telemetry</Tile>
      <Tile label="Poll interval">250<small>ms</small></Tile>
    </div>
  </section>;
}

function SubsystemCard({ state, now, onOpen }) {
  const rows = SUBSYSTEMS.map(([id, label, owner]) => {
    const keys = [...new Set(FIELD_GROUPS.filter(group => group.id === id).flatMap(group => group.fields.map(field => field.key)))];
    const supplied = keys.filter(key => state.packet[id]?.[key] != null).length;
    const stale = keys.filter(key => readingState(state, id, key, now) === 'stale').length;
    return { id, label, owner, total: keys.length, supplied, stale };
  });
  return <section className="card subsystem-card">
    <div className="card-header"><h2 className="card-title">Subsystems</h2><span className="chip plain">Readings supplied</span></div>
    <ul className="subsystem-list">{rows.map(row => <li key={row.id}>
      <button type="button" className="subsystem-row" onClick={() => onOpen(row.id)}
        aria-label={`Show ${row.label} sensors (${row.supplied} of ${row.total} readings supplied${row.stale ? `, ${row.stale} stale` : ''})`}>
        <span className="subsystem-name">{row.label}<small>B{row.owner}</small>{row.stale ? <em>{row.stale} stale</em> : null}</span>
        <span className="coverage" aria-hidden="true"><span style={{ width: `${row.supplied / row.total * 100}%` }} /></span>
        <span className="subsystem-count">{row.supplied}/{row.total}</span>
        <Icon name="chevron" size={15} />
      </button>
    </li>)}</ul>
  </section>;
}

function LineSensorCard({ state, now }) {
  const readings = [
    ['ir_left', 'Left IR', PALETTE.cyan], ['ir_barcode', 'Barcode IR', PALETTE.purple], ['ir_right', 'Right IR', PALETTE.mint],
  ].map(([field, label, color]) => ({ field, label, color, value: state.packet.navigation?.[field], status: readingState(state, 'navigation', field, now) }));
  const status = combinedStatus(readings.map(reading => reading.status));
  return <section id="line-sensors" className="card visual-card" data-state={status}>
    <div className="card-header"><h2 className="card-title">Line sensors</h2><StatusChip status={status} /></div>
    <div className="visual-stage bars-stage"><SensorBars readings={readings} height={226} /></div>
    <div className="tiles tiles-3">
      {readings.map(reading => <Tile key={reading.field} label={reading.label} swatch={reading.color} className={reading.status}>{valueText(reading.value)}</Tile>)}
    </div>
    <p className={`card-caption ${status === 'stale' ? 'stale' : ''}`}>Raw 12-bit readings on a 0–4095 scale.</p>
  </section>;
}

function AccelerationCard({ state, now }) {
  const terrain = state.packet.terrain || {};
  const axes = [
    ['accel_x_mps2', 'Axis X', PALETTE.cyan], ['accel_y_mps2', 'Axis Y', PALETTE.mint], ['accel_z_mps2', 'Axis Z', PALETTE.purple],
  ].map(([field, label, color]) => ({ field, label, color, value: terrain[field], status: readingState(state, 'terrain', field, now) }));
  const status = combinedStatus(axes.map(axis => axis.status));
  return <section id="acceleration-vector" className="card visual-card" data-state={status}>
    <div className="card-header"><h2 className="card-title">Acceleration vector</h2><StatusChip status={status} /></div>
    <div className={`visual-stage reading-${status}`}><AccelerationPad x={terrain.accel_x_mps2} y={terrain.accel_y_mps2} z={terrain.accel_z_mps2} /></div>
    <div className="tiles tiles-3">
      {axes.map(axis => <Tile key={axis.field} label={axis.label} swatch={axis.color} className={axis.status}>
        {typeof axis.value === 'number' ? axis.value.toFixed(2) : '—'}<small>m/s²</small>
      </Tile>)}
    </div>
    <p className={`card-caption ${status === 'stale' ? 'stale' : ''}`}>X and Y on the pad, Z on the rail. Scale ±20 m/s²; the inner ring marks 1 g.</p>
  </section>;
}

function SensorCard({ group, state, now }) {
  const supplied = group.fields.filter(field => state.packet[group.id]?.[field.key] != null).length;
  const panelId = `section-${group.title.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/-$/, '')}`;
  return <section id={panelId} className="card sensor-panel">
    <div className="card-header">
      <div><h2 className="card-title">{group.title}</h2><p className="card-meta">{SUBSYSTEM_LABEL[group.id]} · {supplied} of {group.fields.length} supplied</p></div>
      <span className="owner">B{String(group.owner).padStart(2, '0')}</span>
    </div>
    <dl className="fields">{group.fields.map(field => {
      const status = readingState(state, group.id, field.key, now);
      return <div className={`field-row reading-${status}`} data-field={`${group.id}.${field.key}`} key={field.key}>
        <dt>{field.label}{status === 'stale' && <span className="field-state">Stale</span>}</dt>
        <dd><span data-value>{valueText(state.packet[group.id]?.[field.key])}</span>{field.unit && <span className={field.unit === '°' ? 'unit deg' : 'unit'}>{field.unit}</span>}</dd>
      </div>;
    })}</dl>
    <p className="card-caption">{group.note}</p>
  </section>;
}

function eventTime(event, fallback) {
  if (typeof event.time === 'string' && event.time) return event.time;
  const timestamp = Number.isFinite(event.time) ? event.time : fallback;
  return timestamp ? new Date(timestamp).toLocaleTimeString() : '—';
}

function App() {
  const state = useSyncExternalStore(store.subscribe, store.getSnapshot);
  const [now, setNow] = useState(Date.now());
  const [importError, setImportError] = useState('');
  const [category, setCategory] = useState('all');
  const [tab, selectTab] = useTabs();
  const fileInput = useRef(null);
  const panelsRef = useRef(null);
  const firstTab = useRef(true);
  const latency = useReceiver();
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer); }, []);
  useEffect(() => {
    // On a tab change, bring the new view's top into sight if the page was scrolled past it.
    if (firstTab.current) { firstTab.current = false; return; }
    const header = document.querySelector('.topbar');
    const top = panelsRef.current.getBoundingClientRect().top + window.scrollY - (header?.offsetHeight || 0) - 16;
    if (window.scrollY > top) window.scrollTo({ top, behavior: 'instant' });
  }, [tab]);

  const transport = connectionStatus(state);
  const live = state.mode === 'Live';
  const old = live && (state.error || transport !== 'connected' || (state.lastReceivedAt !== null && now - state.lastReceivedAt > 5000));
  const eventEntries = (state.packet.events || []).map((event, index) => ({
    id: index,
    time: eventTime(event, state.lastReceivedAt || now),
    source: event.source == null ? '' : String(event.source),
    message: event.message == null ? '' : String(event.message),
    severity: /error|fault/i.test(event.level) ? 'error' : /warn/i.test(event.level) ? 'warn' : 'info',
  })).reverse();
  let notice = 'Receiving EDR16 readings. Packet reception does not establish sensor measurement freshness.';
  if (!live) notice = `${state.mode === 'Demo' ? 'DEMO · Synthetic values' : 'IMPORTED · File snapshot'}. Live updates are paused. Resume live to return to the MQTT receiver.`;
  else if (state.error) notice = 'Dashboard receiver unavailable. Last received values are historical. Retrying automatically.';
  else if (transport !== 'connected') notice = `${state.status}. Last received values may be historical.`;
  else if (state.lastReceivedAt === null) notice = 'Waiting for telemetry. The receiver is subscribed; no sensor packets have arrived yet.';
  else if (old) notice = 'No recent packets. Last received values are historical.';
  const connectionText = !live ? `${state.mode} snapshot` : state.error ? 'Receiver unavailable'
    : state.status === 'MQTT subscribed' ? old ? 'No recent packets' : 'MQTT subscribed' : state.status;
  const connectionClass = !live ? 'preview' : old ? 'stale' : transport;
  const lane = (group, field, label, color, unit) => ({
    id: `${group}.${field}`, source: store.getSource(group, field), label, color, unit,
    value: valueText(state.packet[group]?.[field]), status: readingState(state, group, field, now),
  });

  function onTabKey(event) {
    const index = TABS.findIndex(item => item.id === tab);
    const next = { ArrowRight: (index + 1) % TABS.length, ArrowLeft: (index - 1 + TABS.length) % TABS.length, Home: 0, End: TABS.length - 1 }[event.key];
    if (next === undefined) return;
    event.preventDefault();
    selectTab(TABS[next].id);
    document.getElementById(`tab-${TABS[next].id}`)?.focus();
  }

  async function importFile(event) {
    const file = event.target.files[0];
    if (!file) return;
    try {
      if (file.size > 1048576) throw new Error('Use a file smaller than 1 MB.');
      store.setTelemetry(JSON.parse(await file.text()), 'Imported');
      setImportError('');
    } catch (error) { setImportError(`Import failed: ${error.message}`); }
    finally { event.target.value = ''; }
  }

  const intro = Object.fromEntries(TABS.map(item => [item.id, item.intro]));
  return <>
    <a href="#main" className="skip-link">Skip to telemetry</a>
    <header className="topbar">
      <div className="topbar-inner">
        <span className="brand">
          <span className="brand-mark"><Icon name="car" size={20} /></span>
          <strong>PicoCar<span>INF2004 · Mission control</span></strong>
        </span>
        <div className="tabs" role="tablist" aria-label="Dashboard views" onKeyDown={onTabKey}>
          {TABS.map(item => <button key={item.id} type="button" role="tab" id={`tab-${item.id}`} aria-controls={`panel-${item.id}`}
            aria-selected={tab === item.id} tabIndex={tab === item.id ? 0 : -1} className="tab" onClick={() => selectTab(item.id)}>
            {item.label}{item.id === 'events' && eventEntries.length > 0 && <span className="tab-count" aria-hidden="true">{eventEntries.length}</span>}
          </button>)}
        </div>
        <span id="connection" className={`chip connection-badge connection-${connectionClass}`} title={connectionText}><i aria-hidden="true" />{connectionText}</span>
      </div>
    </header>

    <main id="main" tabIndex={-1}>
      <section className="page-intro">
        <div>
          <h1>PicoCar telemetry<span className="heading-dot">.</span></h1>
          <p id="notice" className={`notice ${!live ? 'notice-preview' : old ? 'notice-stale' : 'notice-live'}`} role="status"><Icon name={!live || old ? 'alert' : 'pulse'} size={16} /><span>{notice}</span></p>
        </div>
        <div className="toolbar">
          <button type="button" onClick={() => { store.setTelemetry(DEMO_PACKET, 'Demo'); setImportError(''); }}><Icon name="play" size={15} />Load demo</button>
          <button type="button" onClick={() => fileInput.current.click()}><Icon name="upload" size={15} />Import JSON</button>
          <input ref={fileInput} id="import" type="file" accept=".json,application/json" aria-label="Import telemetry JSON" onChange={importFile} hidden />
          {!live && <button type="button" className="primary-button" onClick={() => { store.resumeLive(); setImportError(''); }}><Icon name="pulse" size={15} />Resume live</button>}
          <button type="button" aria-label="Reset" title="Clear this browser’s readings and reload receiver state" onClick={() => { store.reset(); setImportError(''); }}><Icon name="reset" size={15} />Reset</button>
        </div>
      </section>
      {importError && <p className="import-error" role="alert">{importError}</p>}

      <div className="tab-panels" ref={panelsRef}>
        <TabPanel id="overview" active={tab === 'overview'} intro={intro.overview}>
          <div id="summary" className="grid grid-4" role="group" aria-label="Key readings">
            <StatCard state={state} now={now} group="motion" field="power_percent" title="Commanded power" unit="%" min={0} max={100} note="PWM output, not measured speed." />
            <StatCard state={state} now={now} group="obstacle" field="distance_cm" title="Obstacle distance" unit="cm" min={0} max={400} note="Ultrasonic range. The dial stops at 400 cm." />
            <StatCard state={state} now={now} group="obstacle" field="servo_angle_deg" title="Servo angle" unit="°" min={0} max={180} note="Scan actuator position." />
            <MotorCard state={state} now={now} />
          </div>
          <div className="grid grid-2">
            <ReceiverCard state={state} latency={latency} connectionClass={connectionClass} />
            <SubsystemCard state={state} now={now} onOpen={id => { setCategory(id); selectTab('sensors'); }} />
          </div>
        </TabPanel>

        <TabPanel id="live" active={tab === 'live'} intro={intro.live}>
          <div className="grid grid-2">
            <ProximityRadar state={state} now={now} />
            <VehicleOrientation state={state} now={now} />
            <LineSensorCard state={state} now={now} />
            <AccelerationCard state={state} now={now} />
          </div>
        </TabPanel>

        <TabPanel id="trends" active={tab === 'trends'} intro={intro.trends}>
          <div className={`grid grid-1 ${old ? 'trends-stale' : ''}`} role="group" aria-label="Received telemetry history">
            <HistoryPanel title="Wheel encoder counts" caption="Unsigned counts wrap at 4096. No speed calibration is applied." epoch={state.epoch} active={tab === 'trends'} lanes={[
              lane('motion', 'encoder_left', 'Left wheel', PALETTE.cyan, 'ticks'),
              lane('motion', 'encoder_right', 'Right wheel', PALETTE.purple, 'ticks'),
            ]} />
            <HistoryPanel title="Acceleration" caption="Individual axes, with reception time on the horizontal axis." epoch={state.epoch} active={tab === 'trends'} lanes={[
              lane('terrain', 'accel_x_mps2', 'Axis X', PALETTE.cyan, 'm/s²'),
              lane('terrain', 'accel_y_mps2', 'Axis Y', PALETTE.mint, 'm/s²'),
              lane('terrain', 'accel_z_mps2', 'Axis Z', PALETTE.purple, 'm/s²'),
            ]} />
          </div>
        </TabPanel>

        <TabPanel id="sensors" active={tab === 'sensors'} intro={intro.sensors} aside={
          <label className="category-filter">Show
            <select aria-label="Show subsystem" value={category} onChange={event => setCategory(event.target.value)}>
              <option value="all">All subsystems</option><option value="motion">Motion · Buddy 02</option><option value="navigation">Navigation · Buddy 03</option>
              <option value="terrain">Terrain · Buddy 04</option><option value="obstacle">Obstacles · Buddy 05</option><option value="system">Controller · Buddy 05</option>
            </select>
          </label>}>
          <div id="panels" className="masonry">{FIELD_GROUPS.filter(group => category === 'all' || category === group.id).map(group => <SensorCard key={group.title} group={group} state={state} now={now} />)}</div>
        </TabPanel>

        <TabPanel id="events" active={tab === 'events'} intro={intro.events}>
          <section id="history" className="card events-card">
            <div className="card-header"><h2 className="card-title">Events &amp; faults</h2><span id="event-count" className="chip plain">{eventEntries.length} events</span></div>
            {eventEntries.length ? <ol className="event-list">{eventEntries.map(entry => <li key={entry.id} className={`event severity-${entry.severity}`}>
              <span className="event-time">{entry.time}</span>
              <span className="event-level">{entry.severity}</span>
              <div className="event-body"><p>{entry.message}</p>{entry.source && <span className="event-source">{entry.source}</span>}</div>
            </li>)}</ol>
              : <div className="empty-events"><Icon name="list" size={20} /><p>No events received.</p></div>}
          </section>
        </TabPanel>
      </div>

      <footer>
        <span>PicoCar · INF2004 · <code>{WIRE_TOPIC}</code></span>
        <span id="updated">Last received: {state.lastReceivedAt === null ? '—' : new Date(state.lastReceivedAt).toLocaleTimeString()}</span>
        <span>Display only · NexusUI</span>
      </footer>
    </main>
  </>;
}

window.dashboard = {
  setTelemetry: packet => store.setTelemetry(packet, 'Imported'),
  receiveMqtt: store.receiveMqtt,
  subscriptions: [WIRE_TOPIC],
  topicBindings,
  eventTopic,
};

createRoot(document.getElementById('root')).render(<App />);
