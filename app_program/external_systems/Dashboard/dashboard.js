/* PicoCar mission dashboard: EDR16 telemetry, robot test commands and barcode actions.
   Plain browser JavaScript served by dashboard.py. There is no build step and no network
   asset, so the page works offline on the robot hotspot. The layout follows main's
   React/NexusUI dashboard; its widgets are redrawn here as inline SVG and HTML. */
'use strict';
const groups = [
  ['navigation','Left IR','03','Raw left sensor reading.',['ir_left','line_state','black_mask','line_error']],
  ['navigation','Right IR','03','Raw right sensor reading.',['ir_right']],
  ['navigation','Barcode IR','03','GP28 digital barcode sensor. A barcode ends 30 mm after its last bar; elements read (bars + spaces, 29 for a full code) and the decode result are for the current or last barcode.',['ir_barcode','elements_read','decode_result','barcode','navigation']],
  ['motion','Left wheel encoder','02','Packed counts wrap at 4096; speed requires calibrated wheel feedback.',['encoder_left','left_speed_mps']],
  ['motion','Right wheel encoder','02','Packed counts wrap at 4096; speed requires calibrated wheel feedback.',['encoder_right','right_speed_mps']],
  ['terrain','Accelerometer','04','Acceleration and impact information from the GY-511.',['accel_x_mps2','accel_y_mps2','accel_z_mps2','impact','imu_status']],
  ['terrain','Magnetometer','04','Heading uses the magnetometer. Pitch, roll and turn rate are derived estimates; the GY-511 has no gyroscope.',['mag_x_ut','mag_y_ut','mag_z_ut','heading_deg','pitch_deg','roll_deg','turn_rate_dps','valid']],
  ['obstacle','Ultrasonic Ranger','05','Range and obstacle geometry from the ultrasonic scan.',['distance_cm','width_cm','width_left_cm','width_right_cm','depth_cm','left_clearance_cm','right_clearance_cm']],
  ['terrain','Terrain & hump analysis','04','Derived motion and hump estimates.',['motion_class','hump_state','hump_height_cm','run_max_height_cm','valid']],
  ['obstacle','Servo Motor','05','Scan actuator position and avoidance decisions.',['state','servo_angle_deg','scan_state','bypass_side','route']],
  ['motion','Motor control','02','Commanded PWM output is separate from measured wheel motion.',['state','direction','power_percent','fault_flags','speed_mps','distance_m','last_error','last_command_ms']],
  ['system','Vehicle controller','05','Mission state, safety and telemetry timing.',['state','fault_code','fault_cause','fault_message','safety_state','uptime_ms','telemetry_period_ms','latency_ms','sequence','quality','available','dropped']]
];
// Topic names are case-sensitive. Keep the two roots exactly as agreed.
const wireTopic = 'Robopico/sensors/edr16/v1';
const topicRoots = { sensors: 'Robopico/sensors', actuators: 'Robopico/Actuators' };
const sectionTopics = {
  'Left IR': ['sensors','left_ir'],
  'Right IR': ['sensors','right_ir'],
  'Barcode IR': ['sensors','barcode_ir'],
  'Left wheel encoder': ['sensors','left_encoder'],
  'Right wheel encoder': ['sensors','right_encoder'],
  'Accelerometer': ['sensors','accelerometer'],
  'Magnetometer': ['sensors','magnetometer'],
  'Ultrasonic Ranger': ['sensors','ultrasonic_ranger'],
  'Terrain & hump analysis': ['sensors','terrain'],
  'Servo Motor': ['actuators','servo_motor'],
  'Motor control': ['actuators','motors'],
  'Vehicle controller': ['actuators','vehicle_controller']
};
const topicBindings = groups.flatMap(([key,title,,,fields])=>{
  const [category,section]=sectionTopics[title];
  return fields.map(field=>({topic:`${topicRoots[category]}/${section}/${field}`,key,field}));
});
const eventTopic = `${topicRoots.actuators}/vehicle_controller/events`;
let mqttPacket = {};
const labels = {
  ir_left:['Raw reading'],ir_barcode:['GP28 level (1 = black)'],ir_right:['Raw reading'],
  elements_read:['Elements read','/ 29'],decode_result:['Decoding'],barcode:['Decoded letter'],
  fault_flags:['Motor fault bits'],encoder_left:['Pulse count (mod 4096)'],encoder_right:['Pulse count (mod 4096)'],left_speed_mps:['Wheel speed','m/s'],right_speed_mps:['Wheel speed','m/s'],
  accel_x_mps2:['Acceleration X','m/s²'],accel_y_mps2:['Acceleration Y','m/s²'],accel_z_mps2:['Acceleration Z','m/s²'],
  mag_x_ut:['Magnetic field X','µT'],mag_y_ut:['Magnetic field Y','µT'],mag_z_ut:['Magnetic field Z','µT'],
  heading_deg:['Heading / yaw alias','°'],pitch_deg:['Pitch','°'],roll_deg:['Roll','°'],turn_rate_dps:['Estimated turn rate','°/s'],
  distance_cm:['Obstacle distance','cm'],width_cm:['Estimated width','cm'],width_left_cm:['Centre to left edge','cm'],width_right_cm:['Centre to right edge','cm'],bypass_side:['Bypass side'],depth_cm:['Depth','cm'],left_clearance_cm:['Left clearance','cm'],right_clearance_cm:['Right clearance','cm'],
  line_error:['Line position error'],hump_height_cm:['Current hump peak','cm'],run_max_height_cm:['Run maximum height','cm'],servo_angle_deg:['Servo angle','°'],
  power_percent:['Commanded power','%'],distance_m:['Travelled distance','m'],last_command_ms:['Command timestamp','ms'],heartbeat_age_ms:['Heartbeat age','ms'],
  fault_cause:['Fault cause'],uptime_ms:['Robot uptime','ms'],telemetry_period_ms:['Telemetry period','ms'],latency_ms:['Reported delivery latency','ms'],valid:['Estimate validity']
};
function fieldLabel(field) { return labels[field] || [field.replaceAll('_',' ').replace(/^./,c=>c.toUpperCase())]; }
const el = id => document.getElementById(id);
function text(parent, tag, value, cls) { const n=document.createElement(tag); n.textContent=value; if(cls)n.className=cls; parent.append(n); return n; }
function display(v) { return v === null || v === undefined ? '—' : typeof v==='boolean' ? (v ? 'Yes' : 'No') : String(v); }

/* ---------- Design constants (from main's dashboard) ---------- */
const PALETTE = Object.freeze({
  navyDeep: '#060d1e', navy: '#0a1530', track: '#132650', slate: '#26407a', text: '#e9f0ff',
  cyan: '#38d9f5', mint: '#5cf2bf', purple: '#a58bff', azure: '#63a4ff', amber: '#f6c46a', coral: '#ff7d8e'
});
const COLORS = {
  grid: 'rgba(150, 175, 230, 0.16)', gridStrong: 'rgba(150, 175, 230, 0.3)', label: '#7d8db3',
  cyan: PALETTE.cyan, mint: PALETTE.mint, purple: PALETTE.purple, idle: '#5a6a90'
};
const STATUS_TEXT = { current: 'Received', snapshot: 'Snapshot', stale: 'Stale', missing: 'Awaiting', unavailable: 'Unavailable' };
const SUBSYSTEMS = [['motion','Motion','02'],['navigation','Navigation','03'],['terrain','Terrain','04'],['obstacle','Obstacles','05'],['system','Controller','05']];
const SUBSYSTEM_LABEL = Object.fromEntries(SUBSYSTEMS.map(([id,label])=>[id,label]));
const SUBSYSTEM_FIELDS = Object.fromEntries(SUBSYSTEMS.map(([id])=>[id,[...new Set(groups.filter(([key])=>key===id).flatMap(group=>group[4]))]]));
const FAULT_NAMES = ['none','sensor','line','shock','transport','run timeout','uncalibrated','motor','lifted'];
const DIRECTION_ICON = { FORWARD:['arrow',0], RIGHT:['arrow',90], REVERSE:['arrow',180], LEFT:['arrow',270], PIVOT_RIGHT:['rotate-cw',0], PIVOT_LEFT:['rotate-ccw',0], STOP:['stop',0] };
const TURN_ICON = { Left:['arrow',270], Right:['arrow',90], Forward:['arrow',0], Straight:['arrow',0], 'U-turn':['rotate-ccw',0] };
const TURN_VALUES = { Forward:'forward', Left:'left', Right:'right', 'U-turn':'uturn', Ignore:'ignore' };
const TABS = ['overview','barcodes','live','trends','sensors','events'];
const STALE_AFTER_MS = 5000, HISTORY_LIMIT = 240, WINDOW_MS = 60000, GUTTER = 48;
const ICON_PATHS = {
  car: '<path d="m5 8 2-4h10l2 4M3 9h18v9H3zM5 18v3m14-3v3M6 13h2m8 0h2M3 9l-1-2m19 2 1-2"/><path d="M7 8h10"/>',
  pulse: '<path d="M2 12h5l3-8 4 16 3-8h5"/>',
  upload: '<path d="M12 16V3m-5 5 5-5 5 5M4 16v5h16v-5"/>',
  play: '<path d="m8 4 13 8-13 8z"/>',
  reset: '<path d="M3 11a9 9 0 1 1 2 7M3 4v7h7"/>',
  arrow: '<path d="M12 20V4m-7 7 7-7 7 7"/>',
  'rotate-cw': '<path d="M20 12a8 8 0 1 1-2.3-5.7L20 8.6M20 3v5.6h-5.6"/>',
  'rotate-ccw': '<path d="M4 12a8 8 0 1 0 2.3-5.7L4 8.6M4 3v5.6h5.6"/>',
  stop: '<rect x="6" y="6" width="12" height="12" rx="2"/>',
  chevron: '<path d="m9 6 6 6-6 6"/>',
  list: '<path d="M8 6h13M8 12h13M8 18h13M3 6h.01M3 12h.01M3 18h.01"/>',
  alert: '<path d="M12 3 2 20h20zM12 10v4M12 17v.5"/>'
};
const TREND_PANELS = [
  { id: 'trend-encoders', title: 'Wheel encoder counts', caption: 'Unsigned counts wrap at 4096. No speed calibration is applied.',
    lanes: [['motion','encoder_left','Left wheel',PALETTE.cyan,'ticks'],['motion','encoder_right','Right wheel',PALETTE.purple,'ticks']] },
  { id: 'trend-line', title: 'Line sensors', caption: 'Left and right IR readings with the line position error used for steering. A regular swing here means the robot is weaving across the line.',
    lanes: [['navigation','ir_left','Left IR',PALETTE.cyan,''],['navigation','ir_right','Right IR',PALETTE.mint,''],['navigation','line_error','Line error',PALETTE.purple,'']] },
  { id: 'trend-accel', title: 'Acceleration', caption: 'Individual axes, with reception time on the horizontal axis.',
    lanes: [['terrain','accel_x_mps2','Axis X',PALETTE.cyan,'m/s²'],['terrain','accel_y_mps2','Axis Y',PALETTE.mint,'m/s²'],['terrain','accel_z_mps2','Axis Z',PALETTE.purple,'m/s²']] }
];
const DEMO_PACKET = {motion:{state:'MOVING',direction:'FORWARD',power_percent:35,left_speed_mps:0.24,right_speed_mps:0.25,distance_m:3.8,encoder_left:420,encoder_right:424,last_error:0,last_command_ms:12000},navigation:{state:'TRACKING',ir_left:650,ir_barcode:2100,ir_right:720,line_error:0.03,barcode:'F',navigation:'Left',decode_result:'Successful',elements_read:29,waypoint:'WP-02',requested_turn:'LEFT'},terrain:{valid:true,heading_deg:92,pitch_deg:4.2,roll_deg:0.8,turn_rate_dps:1.4,accel_x_mps2:0.1,accel_y_mps2:0.02,accel_z_mps2:9.8,motion_class:'CLIMBING',impact:false,hump_state:'ASCENDING',hump_height_cm:2.1,run_max_height_cm:3.6,imu_status:'OK'},obstacle:{state:'SCANNING',distance_cm:32,width_cm:14,depth_cm:20,left_clearance_cm:40,right_clearance_cm:18,servo_angle_deg:60,bypass_side:'LEFT',route:'Pending controller'},system:{state:'LINE_TRACK',fault_code:0,fault_cause:'None',fault_message:'None',safety_state:'NORMAL',uptime_ms:12500,telemetry_period_ms:10,latency_ms:24,sequence:1250},events:[{time:'00:12.4',source:'Buddy 4',level:'INFO',message:'Suspected hump; reduce speed requested.'},{time:'00:12.5',source:'Buddy 5',level:'INFO',message:'Obstacle scan started.'}]};

/* ---------- State ---------- */
let packet = {}, mode = 'Waiting', renderedAt = null;
let fieldTimes = {}, samples = {};
let barcodeMap = null, category = 'all', currentTab = 'overview', batching = false;
let lastSequence = 0, receiverId = null, lastHistorySignature = null;
const receiver = {
  polling: location.protocol === 'http:' || location.protocol === 'https:',
  status: 'Connecting to dashboard receiver', error: false, mqttTopic: wireTopic,
  receivedPackets: null, rejectedPackets: null, lastReceivedAt: null, latency: null
};

/* ---------- Small helpers ---------- */
const esc = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[c]);
const finite = value => typeof value === 'number' && Number.isFinite(value);
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const fx = value => Math.round(value * 100) / 100;
const number = (value, digits = 0) => finite(value) ? value.toFixed(digits) : '—';
function valueText(value) {
  if (value === undefined || value === null) return '—';
  if (typeof value === 'boolean') return value ? 'Yes' : 'No';
  if (typeof value === 'number' && !Number.isInteger(value)) return Number(value.toFixed(3)).toString();
  return String(value);
}
function rgba(hex, alpha) {
  const value = Number.parseInt(hex.slice(1), 16);
  return `rgba(${value >> 16 & 255}, ${value >> 8 & 255}, ${value & 255}, ${alpha})`;
}
function icon(name, size = 18) {
  return `<svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">${ICON_PATHS[name] || ICON_PATHS.pulse}</svg>`;
}
function turnIcon(action, size = 14) {
  const spec = TURN_ICON[action];
  return spec ? `<span class="turn-icon" style="transform:rotate(${spec[1]}deg)">${icon(spec[0], size)}</span>` : '';
}
function chip(status, label) {
  return `<span class="chip status-${status}"><i aria-hidden="true"></i><span>${esc(label || STATUS_TEXT[status] || status)}</span></span>`;
}
function tile(label, valueHtml, status = '', swatch = '', valueClass = '') {
  const cls = status === 'missing' || status === 'unavailable' ? ' missing' : status === 'stale' ? ' stale' : '';
  const dot = swatch ? `<i class="swatch" style="background:${swatch}" aria-hidden="true"></i>` : '';
  return `<div class="tile${cls}"><span class="tile-label">${dot}${esc(label)}${status === 'stale' ? ' · stale' : ''}</span><span class="tile-value${valueClass ? ` ${valueClass}` : ''}">${valueHtml}</span></div>`;
}
// Write markup only when it changed, so hover, focus and selection survive the 250 ms poll.
const htmlCache = new Map();
function setHTML(id, html) {
  if (htmlCache.get(id) === html) return;
  htmlCache.set(id, html);
  el(id).innerHTML = html;
}

/* ---------- Reading state (main's rules) ---------- */
function isLive() { return mode === 'MQTT' || mode === 'Waiting'; }
function readingState(key, field, now = Date.now()) {
  if (packet[key]?.[field] == null) return 'missing';
  if (!isLive()) return 'snapshot';
  const timestamp = fieldTimes[`${key}.${field}`];
  if (receiver.error || (receiver.polling && /disconnect|reject|fail|error|unavailable|stale|connecting|subscribing/i.test(receiver.status))
      || timestamp == null || now - timestamp > STALE_AFTER_MS) return 'stale';
  return 'current';
}
function combinedStatus(statuses) {
  const supplied = statuses.filter(status => status !== 'missing');
  if (!supplied.length) return 'missing';
  if (supplied.includes('stale')) return 'stale';
  return supplied.includes('snapshot') ? 'snapshot' : 'current';
}
function transportStatus() {
  if (!isLive()) return 'disconnected';
  if (!receiver.polling) return mode === 'MQTT' ? 'connected' : 'disconnected';
  if (receiver.error || /reject|fail|error|unavailable/i.test(receiver.status)) return 'error';
  if (receiver.status === 'MQTT subscribed') return 'connected';
  if (/disconnect/i.test(receiver.status)) return 'disconnected';
  return 'connecting';
}
function connectionClass(now) {
  if (!isLive()) return 'preview';
  const transport = transportStatus();
  if (transport !== 'connected') return transport;
  return receiver.lastReceivedAt !== null && now - receiver.lastReceivedAt > STALE_AFTER_MS ? 'stale' : 'connected';
}
const isOld = now => isLive() && connectionClass(now) !== 'connected';

/* ---------- Widgets: plain SVG/HTML stand-ins for main's NexusUI widgets ---------- */
// Dial: arc from 6 o'clock clockwise through 360°·value, wedge, pointer and centre screw.
function dialHtml(value, min, max) {
  const size = 96, c = 48, r = 45.6, width = 4.8;
  let paths = '';
  if (finite(value)) {
    const theta = 2 * Math.PI * clamp((clamp(value, min, max) - min) / (max - min), 0, 0.9999);
    const x = fx(c - r * Math.sin(theta)), y = fx(c + r * Math.cos(theta));
    const arc = `A${r} ${r} 0 ${theta > Math.PI ? 1 : 0} 1 ${x} ${y}`;
    paths = `<path d="M${c} ${c}L${c} ${fx(c + r)}${arc}Z" fill="${PALETTE.cyan}" fill-opacity="0.3"/>`
      + `<path d="M${c} ${fx(c + r)}${arc}" fill="none" stroke="${PALETTE.cyan}" stroke-width="${width}"/>`
      + `<path d="M${c} ${c}L${x} ${y}" stroke="${PALETTE.cyan}" stroke-width="${width}" stroke-linecap="round"/>`;
  }
  return `<div class="nx nx-dial" aria-hidden="true"><svg width="${size}" height="${size}" viewBox="0 0 ${size} ${size}"><circle cx="${c}" cy="${c}" r="${r}" fill="${PALETTE.navyDeep}"/>${paths}<circle cx="${c}" cy="${c}" r="8" fill="${PALETTE.cyan}"/></svg></div>`;
}
// Motor fault mask as 7 cells, bit 6 down to bit 0.
function faultBitsHtml(mask) {
  const valid = Number.isInteger(mask) && mask >= 0;
  const bits = [6, 5, 4, 3, 2, 1, 0];
  return `<div class="fault-bits"><div class="fault-cells" aria-hidden="true">${bits.map(bit => `<span${valid && (mask >> bit) & 1 ? ' class="on"' : ''}></span>`).join('')}</div>`
    + `<div class="bit-labels" aria-hidden="true">${bits.map(bit => `<span>${bit}</span>`).join('')}</div></div>`;
}
// IR bars. Each bar uses its own scale: 0/1 in digital mode (GP28 is always digital), else 0-4095.
function barsHtml(readings) {
  const ticks = [4095, 3072, 2048, 1024, 0];
  const scale = ticks.map(t => `<span style="top:${fx((1 - t / 4095) * 100)}%">${t}</span>`).join('');
  const bars = readings.map(reading => {
    const max = finite(reading.value) && reading.value <= 1 ? 1 : 4095;
    const height = finite(reading.value) ? fx(clamp(reading.value, 0, max) / max * 100) : 0;
    const opacity = reading.status === 'stale' ? 0.38 : reading.status === 'missing' ? 0 : 1;
    return `<div class="ir-bar" style="height:${height}%;opacity:${opacity};background:linear-gradient(${rgba(reading.color, 0.95)},${rgba(reading.color, 0.08)});box-shadow:inset 0 3px 0 ${reading.color}"></div>`;
  }).join('');
  return `<div class="bars-chart"><div class="bars-scale" style="height:226px" aria-hidden="true">${scale}</div>`
    + `<div class="nx ir-bars" aria-hidden="true"><i class="gridline" style="top:25%"></i><i class="gridline" style="top:50%"></i><i class="gridline" style="top:75%"></i>${bars}</div></div>`;
}
// Acceleration: X/Y knob on the pad (Y up), Z on the rail; each axis clamped to ±20 m/s².
function padHtml(x, y, z) {
  const range = 20;
  const px = finite(x) ? clamp(x, -range, range) : 0, py = finite(y) ? clamp(y, -range, range) : 0, pz = finite(z) ? clamp(z, -range, range) : 0;
  const fill = fx((pz + range) / (2 * range) * 100);
  return `<div class="accel-stage"><div class="nx nx-pad" aria-hidden="true"><span class="pad-knob" style="left:${fx(50 + px / range * 50)}%;top:${fx(50 - py / range * 50)}%"></span></div>`
    + `<div class="accel-rail"><div class="nx nx-rail" aria-hidden="true"><span class="rail-fill" style="height:${fill}%"></span><span class="rail-knob" style="bottom:${fill}%"></span></div><span>Z</span></div></div>`;
}

/* ---------- Robot test card (shown on every tab) ---------- */
function faultText(system) {
  const code = system.fault_code;
  if (code == null) return '—';
  let label = esc(valueText(code));
  if (Number.isInteger(code) && code >= 0 && code < FAULT_NAMES.length) label += ` · ${FAULT_NAMES[code]}`;
  const cause = system.fault_cause;
  if (cause != null && !/^none/i.test(String(cause))) label += ` · ${esc(cause)}`;
  return label;
}
function renderMission(now) {
  const system = packet.system || {}, navigation = packet.navigation || {};
  const letter = navigation.barcode, instruction = navigation.navigation;
  const hasLetter = typeof letter === 'string' && /^[A-Z]$/.test(letter);
  const letterStatus = hasLetter ? readingState('navigation', 'barcode', now) : 'missing';
  setHTML('mission-tiles', [
    tile('Mission state', esc(valueText(system.state)), readingState('system', 'state', now)),
    tile('Motion', esc(valueText(packet.motion?.state)), readingState('motion', 'state', now)),
    tile('Terrain validity', esc(valueText(packet.terrain?.valid)), readingState('terrain', 'valid', now)),
    tile('Fault code', faultText(system), readingState('system', 'fault_code', now)),
    tile('Barcode', hasLetter ? letter : '—', letterStatus, '', 'big'),
    tile('Instruction', `${turnIcon(instruction, 15)}${esc(valueText(instruction))}`, readingState('navigation', 'navigation', now))
  ].join(''));
  const fault = system.fault_code, safety = system.safety_state, status = readingState('system', 'safety_state', now);
  let cls = '', label = 'Awaiting';
  if (finite(fault) && fault > 0) { cls = 'status-error'; label = `Fault ${fault}${FAULT_NAMES[fault] ? ` · ${FAULT_NAMES[fault]}` : ''}`; }
  else if (safety != null) {
    label = String(safety);
    cls = status === 'stale' ? 'status-stale' : status === 'snapshot' ? 'status-snapshot' : safety === 'Armed' ? 'status-current' : '';
    if (status === 'stale') label += ' · stale';
  }
  el('robot-chip').className = `chip${cls ? ` ${cls}` : ''}`;
  el('robot-chip-text').textContent = label;
}

/* ---------- Overview ---------- */
function statCard(key, field, title, unit, min, max, note, now) {
  const value = packet[key]?.[field];
  const status = readingState(key, field, now);
  const degree = unit === '°';
  return `<section class="card stat-card reading-${status}" aria-label="${esc(title)}">`
    + `<div class="card-header"><h2 class="card-title">${esc(title)}</h2>${chip(status)}</div>`
    + `<div class="stat-body"><div class="stat-copy"><p class="stat-value">${esc(valueText(value))}<span${degree ? ' class="deg"' : ''}>${unit}</span></p><p class="stat-note">${note}</p></div>`
    + `<div class="stat-gauge">${dialHtml(typeof value === 'number' ? value : undefined, min, max)}<span class="stat-scale">${min}–${max}${degree ? unit : ` ${unit}`}</span></div></div></section>`;
}
function motorCard(now) {
  const motion = packet.motion || {};
  const status = readingState('motion', 'state', now);
  const directionStale = readingState('motion', 'direction', now) === 'stale';
  const faultStale = readingState('motion', 'fault_flags', now) === 'stale';
  const tone = motion.state == null ? 'missing' : /fault/i.test(motion.state) ? 'fault' : /moving/i.test(motion.state) ? 'moving' : 'idle';
  const [name, rotation] = DIRECTION_ICON[motion.direction] || ['arrow', 0];
  return `<section class="card stat-card motion-card reading-${status}" aria-label="Motor control">`
    + `<div class="card-header"><h2 class="card-title">Motor control</h2>${chip(status)}</div>`
    + `<p class="motion-state tone-${tone}"><i aria-hidden="true"></i>${esc(valueText(motion.state))}</p>`
    + `<p class="motion-direction"><span class="direction-icon" style="transform:rotate(${rotation}deg)">${icon(motion.direction == null ? 'arrow' : name, 13)}</span>${esc(valueText(motion.direction))}${directionStale ? '<span class="field-state">Stale</span>' : ''}</p>`
    + `<div class="fault-block"><div class="fault-line"><span>${faultStale ? 'Fault mask · stale' : 'Fault mask'}</span><strong>${esc(valueText(motion.fault_flags))}</strong></div>${faultBitsHtml(motion.fault_flags)}</div></section>`;
}
function receiverCardHtml(now) {
  const cls = connectionClass(now), transport = transportStatus();
  const label = !receiver.polling ? 'Not polling' : !isLive() ? 'Paused for snapshot' : receiver.error ? 'Unavailable'
    : { connected: 'Online', connecting: 'Connecting', disconnected: 'Disconnected', error: 'Error' }[transport];
  const status = !receiver.polling ? 'missing'
    : { connected: 'current', connecting: 'connecting', preview: 'snapshot', stale: 'stale', disconnected: 'stale', error: 'error' }[cls];
  const show = receiver.polling && !receiver.error;
  const latency = show && receiver.latency !== null ? `${receiver.latency}<small>ms</small>` : '—';
  const rejected = show ? receiver.rejectedPackets : null;
  return `<div class="card-header"><h2 class="card-title">Receiver</h2>${chip(status, label)}</div><div class="tiles tiles-2">`
    + tile('Latency', latency)
    + tile('Wire format', 'EDR16<small>2 bytes</small>')
    + tile('Decoded packets', show ? esc(valueText(receiver.receivedPackets)) : '—')
    + tile('Rejected packets', esc(valueText(rejected)), rejected ? 'stale' : '')
    + tile('Endpoint', '/api/telemetry')
    + tile('Poll interval', '250<small>ms</small>')
    + '</div>';
}
function subsystemCardHtml(now) {
  const rows = SUBSYSTEMS.map(([id, label, owner]) => {
    const keys = SUBSYSTEM_FIELDS[id];
    const supplied = keys.filter(key => packet[id]?.[key] != null).length;
    const stale = keys.filter(key => readingState(id, key, now) === 'stale').length;
    return `<li><button type="button" class="subsystem-row" data-category="${id}" aria-label="Show ${label} sensors (${supplied} of ${keys.length} readings supplied${stale ? `, ${stale} stale` : ''})">`
      + `<span class="subsystem-name">${label}<small>B${owner}</small>${stale ? `<em>${stale} stale</em>` : ''}</span>`
      + `<span class="coverage" aria-hidden="true"><span style="width:${fx(supplied / keys.length * 100)}%"></span></span>`
      + `<span class="subsystem-count">${supplied}/${keys.length}</span>${icon('chevron', 15)}</button></li>`;
  }).join('');
  return `<div class="card-header"><h2 class="card-title">Subsystems</h2><span class="chip plain">Readings supplied</span></div><ul class="subsystem-list">${rows}</ul>`;
}
function renderOverview(now) {
  setHTML('summary', statCard('motion', 'power_percent', 'Commanded power', '%', 0, 100, 'PWM output, not measured speed.', now)
    + statCard('obstacle', 'distance_cm', 'Obstacle distance', 'cm', 0, 400, 'Ultrasonic range. The dial stops at 400 cm.', now)
    + statCard('obstacle', 'servo_angle_deg', 'Servo angle', '°', 0, 180, 'Scan actuator position.', now)
    + motorCard(now));
  setHTML('receiver-card', receiverCardHtml(now));
  setHTML('subsystem-card', subsystemCardHtml(now));
}

/* ---------- Barcodes: last read, A-Z codebook and read history ---------- */
function renderBarcodeLive(now) {
  const navigation = packet.navigation || {};
  const letter = navigation.barcode, instruction = navigation.navigation, result = navigation.decode_result;
  const elements = navigation.elements_read, level = navigation.ir_barcode;
  const hasLetter = typeof letter === 'string' && /^[A-Z]$/.test(letter);
  const letterStatus = hasLetter ? readingState('navigation', 'barcode', now) : 'missing';
  const resultStatus = { Successful: 'current', 'Reading...': 'connecting', Unsuccessful: 'error' }[result] || 'missing';
  const count = finite(elements) ? clamp(elements, 0, 29) : 0;
  const mapped = hasLetter && barcodeMap ? `Codebook: ${letter} = ${esc(barcodeMap[letter] || 'No turn')}` : 'Instruction for the next junction, as reported by the robot.';
  setHTML('barcode-live', `<div class="card-header"><h2 class="card-title">Last barcode</h2>${chip(resultStatus, result == null ? 'Awaiting' : valueText(result))}</div>`
    + `<div class="barcode-hero reading-${letterStatus}"><span class="barcode-letter${hasLetter ? '' : ' missing'}" role="img" aria-label="Decoded letter ${hasLetter ? letter : 'unavailable'}">${hasLetter ? letter : '—'}</span>`
    + `<div class="barcode-copy"><p class="barcode-instruction">${turnIcon(instruction, 22)}<span>${esc(valueText(instruction))}</span></p><p class="barcode-sub">${mapped}</p></div></div>`
    + `<div class="elements-line"><span>Elements</span><strong>${finite(elements) ? elements : '—'} / 29</strong></div>`
    + `<span class="coverage elements-bar" aria-hidden="true"><span style="width:${fx(count / 29 * 100)}%"></span></span>`
    + `<div class="tiles tiles-3">${tile('Decoding', esc(valueText(result)), readingState('navigation', 'decode_result', now))}`
    + tile('Elements', `${esc(valueText(elements))}<small>/ 29</small>`, readingState('navigation', 'elements_read', now))
    + tile('GP28 level', esc(valueText(level)), readingState('navigation', 'ir_barcode', now)) + '</div>'
    + '<p class="card-caption">A barcode ends 30 mm after its last bar. A full code is 29 elements (bars + spaces).</p>');
  renderCodebook();
}
function renderCodebook() {
  const selected = String(el('map-letter').value || '').trim().toUpperCase();
  const lastRead = packet.navigation?.barcode;
  let mapped = 0;
  const tiles = Array.from({ length: 26 }, (_, index) => {
    const letter = String.fromCharCode(65 + index);
    const action = barcodeMap ? barcodeMap[letter] : undefined;
    const isMapped = Boolean(action) && action !== 'Ignore';
    if (isMapped) mapped += 1;
    const label = !barcodeMap ? '—' : action ? display(action) : 'No turn';
    const classes = ['letter-tile', isMapped ? 'mapped' : action ? '' : 'unmapped', letter === selected ? 'selected' : '', letter === lastRead ? 'last-read' : ''].filter(Boolean).join(' ');
    return `<button type="button" class="${classes}" data-letter="${letter}" title="${letter}: ${esc(label)}${letter === lastRead ? ' (last read)' : ''}"><strong>${letter}</strong><span>${isMapped ? turnIcon(action, 12) : ''}${esc(label)}</span></button>`;
  }).join('');
  setHTML('barcode-map', tiles);
  el('codebook-count').textContent = barcodeMap ? `${mapped} of 26 mapped` : 'Not loaded';
  el('codebook-caption').textContent = barcodeMap
    ? 'Shows the mapping last acknowledged by the robot. Letters marked No turn are read without turning. The dot marks the last letter read.'
    : 'Mapping appears when dashboard.py is running.';
}
function renderBarcodeHistory(rows) {
  const list=Array.isArray(rows)?rows.slice().reverse():[];
  const signature=JSON.stringify(list);
  if(signature===lastHistorySignature)return;
  lastHistorySignature=signature;
  const out=[];
  if(!list.length){const row=document.createElement('tr');const cell=text(row,'td','No barcode reads yet.','empty-cell');cell.colSpan=7;out.push(row);}
  list.forEach(item=>{
    const read=item||{};
    const row=document.createElement('tr');row.className=read.result==='Successful'?'read-ok':'read-fail';
    [read.time,read.pass,`${read.elements} / 29`,`${read.seconds} s`,read.result,read.letter||'—',read.turn||'—'].forEach((value,index)=>{
      if(index!==4){text(row,'td',display(value));return;}
      const cell=text(row,'td','');text(cell,'span','','result-dot');text(cell,'span',display(value));
    });
    out.push(row);
  });
  el('barcode-history').replaceChildren(...out);
  el('barcode-count').textContent=`${list.length} reads`;
}
// Letter -> action as last acknowledged by RoboPico (dashboard.py barcode_map).
function renderBarcodeMap(map) {
  barcodeMap=map&&typeof map==='object'&&!Array.isArray(map)?map:null;
  renderCodebook();
}

/* ---------- Live views ---------- */
function radarHtml(now) {
  const obstacle = packet.obstacle || {};
  const distance = obstacle.distance_cm, servo = obstacle.servo_angle_deg;
  const hasRange = finite(distance) && distance >= 0;
  const hasServo = finite(servo) && servo >= 0 && servo <= 180;
  const distanceStatus = readingState('obstacle', 'distance_cm', now);
  const servoStatus = readingState('obstacle', 'servo_angle_deg', now);
  const status = !hasRange ? 'missing' : distanceStatus === 'stale' || (hasServo && servoStatus === 'stale') ? 'stale' : distanceStatus;
  const plotted = hasRange && hasServo;
  const displayRange = Math.max(100, Math.ceil((hasRange ? distance : 0) / 100) * 100);
  // A servo angle of 0 points left; 90 points forward; 180 points right.
  const CX = 220, CY = 222, R = 184;
  const polar = (degrees, radius) => {
    const angle = degrees * Math.PI / 180;
    return { x: fx(CX - Math.cos(angle) * radius), y: fx(CY - Math.sin(angle) * radius) };
  };
  const bearing = hasServo ? servo : 90;
  const edge = polar(bearing, R), beamStart = polar(bearing - 9, R), beamEnd = polar(bearing + 9, R);
  const observation = polar(bearing, Math.min(1, (hasRange ? distance : 0) / displayRange) * R);
  const labelLeft = observation.x > CX + 60;
  const caption = !hasRange ? 'Waiting for an ultrasonic range reading.'
    : !hasServo ? 'Range received. A servo angle is needed to plot its bearing.'
      : 'Latest received range at the servo bearing. No scan history.';
  const accessibleLabel = !hasRange ? 'Proximity radar: awaiting ultrasonic data'
    : !hasServo ? `Proximity radar: ${number(distance)} centimetres, servo angle unavailable`
      : `Proximity radar: ${number(distance)} centimetres at servo ${number(servo)} degrees${status === 'stale' ? ', stale reading' : ''}`;
  const arc = radius => `M${fx(CX - radius)} ${CY}A${fx(radius)} ${fx(radius)} 0 0 1 ${fx(CX + radius)} ${CY}`;
  const rings = [0.25, 0.5, 0.75].map(fraction => `<path d="${arc(R * fraction)}" stroke-dasharray="3 6"/>`).join('');
  const spokes = [30, 60, 90, 120, 150].map(degrees => { const end = polar(degrees, R); return `<line x1="${CX}" y1="${CY}" x2="${end.x}" y2="${end.y}"/>`; }).join('');
  const ranges = [0.5, 1].map(fraction => `<text x="${CX + R * fraction - 4}" y="${CY - 6}" text-anchor="end" font-size="14">${Math.round(displayRange * fraction)} cm</text>`).join('');
  const reading = plotted ? `<g opacity="${status === 'stale' ? 0.35 : 1}">`
    + `<path d="M${CX} ${CY}L${beamStart.x} ${beamStart.y}A${R} ${R} 0 0 1 ${beamEnd.x} ${beamEnd.y}Z" fill="url(#radar-beam)"/>`
    + `<line x1="${CX}" y1="${CY}" x2="${edge.x}" y2="${edge.y}" stroke="${COLORS.mint}" stroke-width="2.5" stroke-linecap="round"/>`
    + `<circle cx="${observation.x}" cy="${observation.y}" r="17" fill="${COLORS.purple}" fill-opacity="0.14" stroke="${COLORS.purple}" stroke-opacity="0.55" stroke-width="1.5"/>`
    + `<circle cx="${observation.x}" cy="${observation.y}" r="6.5" fill="${COLORS.purple}"/>`
    + `<text x="${fx(observation.x + (labelLeft ? -24 : 24))}" y="${fx(observation.y + 6)}" text-anchor="${labelLeft ? 'end' : 'start'}" fill="#e4dcff" font-size="19" font-weight="600" font-family="inherit">${number(distance)} cm</text></g>` : '';
  const svg = `<svg viewBox="0 0 440 256" role="img" aria-label="${esc(accessibleLabel)}"><defs>`
    + `<radialGradient id="radar-field" cx="${CX}" cy="${CY}" r="${R}" gradientUnits="userSpaceOnUse"><stop offset="0%" stop-color="${COLORS.cyan}" stop-opacity="0.14"/><stop offset="100%" stop-color="${COLORS.cyan}" stop-opacity="0.02"/></radialGradient>`
    + `<radialGradient id="radar-beam" cx="${CX}" cy="${CY}" r="${R}" gradientUnits="userSpaceOnUse"><stop offset="0%" stop-color="${COLORS.mint}" stop-opacity="0.5"/><stop offset="100%" stop-color="${COLORS.mint}" stop-opacity="0.04"/></radialGradient></defs>`
    + `<path d="${arc(R)}Z" fill="url(#radar-field)"/>`
    + `<g fill="none" stroke="${COLORS.grid}" stroke-width="1.5">${rings}<path d="${arc(R)}" stroke="${COLORS.gridStrong}"/>${spokes}<line x1="${CX - R}" y1="${CY}" x2="${CX + R}" y2="${CY}" stroke="${COLORS.gridStrong}"/></g>`
    + `<g fill="${COLORS.label}" font-size="15" font-family="inherit" aria-hidden="true"><text x="${CX}" y="24" text-anchor="middle" letter-spacing="1">FORWARD · 90°</text><text x="${CX - R}" y="${CY + 18}" text-anchor="start">0°</text><text x="${CX + R}" y="${CY + 18}" text-anchor="end">180°</text>${ranges}</g>`
    + reading
    + `<rect x="${CX - 19}" y="${CY + 4}" width="38" height="26" rx="8" fill="#0f1d3f" stroke="${COLORS.gridStrong}" stroke-width="1.5"/>`
    + `<circle cx="${CX}" cy="${CY}" r="7" fill="${plotted ? COLORS.mint : COLORS.idle}" stroke="#0b1631" stroke-width="3"/></svg>`;
  return `<div class="card-header"><h2 class="card-title">Proximity radar</h2>${chip(status)}</div><div class="visual-stage">${svg}</div><div class="tiles tiles-3">`
    + tile('Range', `${hasRange ? number(distance) : '—'}<small>cm</small>`, distanceStatus)
    + tile('Servo bearing', `${hasServo ? number(servo) : '—'}<small class="deg">°</small>`, hasServo ? servoStatus : 'missing')
    + tile('Display range', `${displayRange}<small>cm</small>`)
    + `</div><p class="card-caption${status === 'stale' ? ' stale' : ''}">${status === 'stale' ? `Stale. ${caption}` : caption}</p>`;
}
function orientationHtml(now) {
  const terrain = packet.terrain || {};
  const hasHeading = finite(terrain.heading_deg);
  const heading = hasHeading ? ((terrain.heading_deg % 360) + 360) % 360 : 0;
  const status = hasHeading ? readingState('terrain', 'heading_deg', now) : 'unavailable';
  const accent = hasHeading ? COLORS.mint : COLORS.idle;
  const accessibleLabel = hasHeading
    ? `Vehicle orientation: heading ${number(heading)} degrees${status === 'stale' ? ', stale reading' : ''}`
    : 'Vehicle orientation: heading unavailable; vehicle shown in a neutral position';
  const ticks = Array.from({ length: 24 }, (_, index) => index * 15).map(degrees => `<line x1="220" y1="30" x2="220" y2="${degrees % 90 ? 40 : 50}" stroke="${degrees % 90 ? COLORS.grid : COLORS.gridStrong}" transform="rotate(${degrees} 220 220)"/>`).join('');
  const compass = hasHeading ? `<g fill="${COLORS.label}" font-size="24" font-weight="600" font-family="inherit" text-anchor="middle" aria-hidden="true"><text x="220" y="72" fill="${COLORS.mint}">N</text><text x="376" y="228">E</text><text x="220" y="386">S</text><text x="64" y="228">W</text></g>` : '';
  const svg = `<svg viewBox="0 0 440 440" role="img" aria-label="${esc(accessibleLabel)}"><defs>`
    + `<radialGradient id="orient-dial" cx="220" cy="220" r="200" gradientUnits="userSpaceOnUse"><stop offset="0%" stop-color="${COLORS.purple}" stop-opacity="0.1"/><stop offset="100%" stop-color="${COLORS.purple}" stop-opacity="0.01"/></radialGradient>`
    + `<linearGradient id="orient-body" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stop-color="#26407a"/><stop offset="100%" stop-color="#13244c"/></linearGradient></defs>`
    + `<circle cx="220" cy="220" r="196" fill="url(#orient-dial)" stroke="${COLORS.gridStrong}" stroke-width="1.5"/>`
    + `<circle cx="220" cy="220" r="140" fill="none" stroke="${COLORS.grid}" stroke-width="1.5" stroke-dasharray="3 7"/>`
    + `<g stroke-width="2" stroke-linecap="round">${ticks}</g>${compass}`
    + `<g transform="rotate(${fx(heading)} 220 220)" opacity="${status === 'stale' ? 0.35 : 1}">`
    + `<path d="M220 128V106" stroke="${accent}" stroke-width="5" stroke-linecap="round"/><path d="M207 110l13-18 13 18z" fill="${accent}"/>`
    + '<g fill="#0b1733" stroke="#3a5591" stroke-width="1.5"><rect x="160" y="150" width="20" height="50" rx="7"/><rect x="260" y="150" width="20" height="50" rx="7"/><rect x="160" y="250" width="20" height="50" rx="7"/><rect x="260" y="250" width="20" height="50" rx="7"/></g>'
    + `<rect x="176" y="128" width="88" height="194" rx="30" fill="url(#orient-body)" stroke="${COLORS.cyan}" stroke-width="3"/>`
    + '<rect x="192" y="174" width="56" height="66" rx="13" fill="#0a1530" stroke="#4d6aa8" stroke-width="1.5"/><rect x="196" y="258" width="48" height="44" rx="10" fill="#0d1a38"/>'
    + `<rect x="204" y="136" width="32" height="12" rx="6" fill="${accent}" opacity="0.85"/></g></svg>`;
  const fields = [['heading_deg', 'Heading', hasHeading ? heading : undefined, 0], ['pitch_deg', 'Pitch', terrain.pitch_deg, 1], ['roll_deg', 'Roll', terrain.roll_deg, 1]];
  const caption = !hasHeading ? 'Heading unavailable. Waiting for an explicit orientation estimate.'
    : status === 'stale' ? 'Stale heading. Showing the last received orientation.'
      : `${status === 'snapshot' ? 'Snapshot' : 'Received'} heading, clockwise from north.`;
  return `<div class="card-header"><h2 class="card-title">Vehicle orientation</h2>${chip(status)}</div><div class="visual-stage">${svg}</div><div class="tiles tiles-3">`
    + fields.map(([field, label, value, digits]) => tile(label, `${number(value, digits)}<small class="deg">°</small>`, finite(value) ? readingState('terrain', field, now) : 'missing')).join('')
    + `</div><p class="card-caption${status === 'stale' ? ' stale' : ''}">${caption}</p>`;
}
function lineSensorHtml(now) {
  const navigation = packet.navigation || {};
  const readings = [['ir_left', 'Left IR', PALETTE.cyan], ['ir_barcode', 'Barcode IR', PALETTE.purple], ['ir_right', 'Right IR', PALETTE.mint]]
    .map(([field, label, color]) => ({ field, label, color, value: navigation[field], status: readingState('navigation', field, now) }));
  const status = combinedStatus(readings.map(reading => reading.status));
  return `<div class="card-header"><h2 class="card-title">Line sensors</h2>${chip(status)}</div>`
    + `<div class="visual-stage bars-stage">${barsHtml(readings)}</div>`
    + `<div class="tiles tiles-3">${readings.map(reading => tile(reading.label, esc(valueText(reading.value)), reading.status, reading.color)).join('')}</div>`
    + `<p class="card-caption${status === 'stale' ? ' stale' : ''}">Raw 12-bit readings (0–4095); digital mode shows 0/1 (1 = black).</p>`;
}
function accelerationHtml(now) {
  const terrain = packet.terrain || {};
  const axes = [['accel_x_mps2', 'Axis X', PALETTE.cyan], ['accel_y_mps2', 'Axis Y', PALETTE.mint], ['accel_z_mps2', 'Axis Z', PALETTE.purple]]
    .map(([field, label, color]) => ({ field, label, color, value: terrain[field], status: readingState('terrain', field, now) }));
  const status = combinedStatus(axes.map(axis => axis.status));
  return `<div class="card-header"><h2 class="card-title">Acceleration vector</h2>${chip(status)}</div>`
    + `<div class="visual-stage reading-${status}">${padHtml(terrain.accel_x_mps2, terrain.accel_y_mps2, terrain.accel_z_mps2)}</div>`
    + `<div class="tiles tiles-3">${axes.map(axis => tile(axis.label, `${finite(axis.value) ? axis.value.toFixed(2) : '—'}<small>m/s²</small>`, axis.status, axis.color)).join('')}</div>`
    + `<p class="card-caption${status === 'stale' ? ' stale' : ''}">X and Y on the pad, Z on the rail. Scale ±20 m/s²; the inner ring marks 1 g.</p>`;
}
function renderLive(now) {
  setHTML('proximity-radar', radarHtml(now));
  setHTML('vehicle-orientation', orientationHtml(now));
  setHTML('line-sensors', lineSensorHtml(now));
  setHTML('acceleration-vector', accelerationHtml(now));
}

/* ---------- Trends: canvas lanes ported from main's HistoryChart ---------- */
function buildTrends() {
  setHTML('trends-grid', TREND_PANELS.map(panel => `<section class="card chart-card" aria-labelledby="${panel.id}-title">`
    + `<div class="card-header"><h2 class="card-title" id="${panel.id}-title">${panel.title}</h2><span class="chip plain">60 s window</span></div>`
    + `<div class="chart-stack">${panel.lanes.map(([key, field, label, color]) => `<div class="lane reading-missing" id="lane-${key}-${field}">`
      + `<div class="lane-head"><span class="lane-label"><i class="swatch" style="background:${color}" aria-hidden="true"></i>${label}</span><span class="lane-value" id="lane-value-${key}-${field}">—</span></div>`
      + `<canvas id="canvas-${key}-${field}" role="img" aria-label="${label} received history"></canvas></div>`).join('')}</div>`
    + '<div class="chart-axis" aria-hidden="true"><span>−60 s</span><span>−45 s</span><span>−30 s</span><span>−15 s</span><span>now</span></div>'
    + `<p class="card-caption">${panel.caption}</p></section>`).join(''));
}
function renderTrendValues(now) {
  TREND_PANELS.forEach(panel => panel.lanes.forEach(([key, field, , , unit]) => {
    el(`lane-${key}-${field}`).className = `lane reading-${readingState(key, field, now)}`;
    setHTML(`lane-value-${key}-${field}`, `${esc(valueText(packet[key]?.[field]))}${unit ? `<small>${unit}</small>` : ''}`);
  }));
  el('trends-grid').className = `grid grid-1${isOld(now) ? ' trends-stale' : ''}`;
}
const trendLanes = [];
let trendFrame = 0, trendLast = 0, trendInterval = 33;
// One precision per lane, chosen from the visible span, so both axis labels read alike.
const tickLabel = (value, span) => value.toFixed(span >= 20 ? 0 : span >= 2 ? 1 : 2);
function drawLane(lane) {
  const { ctx: context, width, height, color } = lane;
  if (!width || !height) return;
  const font = '10.5px "JetBrains Mono", "Cascadia Mono", "SF Mono", Consolas, monospace', labelColor = '#6a7aa1';
  context.clearRect(0, 0, width, height);
  const plotWidth = Math.max(10, width - GUTTER);
  const top = 8, bottom = height - 8;
  context.strokeStyle = 'rgba(150, 175, 230, 0.09)';
  context.lineWidth = 1;
  context.setLineDash([2, 4]);
  context.beginPath();
  for (const fraction of [0, 0.5, 1]) {
    const y = Math.round(top + (bottom - top) * fraction) + 0.5;
    context.moveTo(0, y); context.lineTo(plotWidth, y);
  }
  for (const fraction of [0.25, 0.5, 0.75]) {
    const x = Math.round(plotWidth * fraction) + 0.5;
    context.moveTo(x, top); context.lineTo(x, bottom);
  }
  context.stroke();
  context.setLineDash([]);
  const now = Date.now();
  const start = now - WINDOW_MS;
  const visible = (samples[`${lane.key}.${lane.field}`] || []).filter(point => point.t >= start - 1000 && point.t <= now + 1000);
  if (!visible.length) {
    context.fillStyle = labelColor;
    context.font = font;
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText('Awaiting readings', plotWidth / 2, height / 2);
    return;
  }
  const floor = Math.min(...visible.map(point => point.v));
  let low = floor;
  let high = Math.max(...visible.map(point => point.v));
  if (high - low < 1e-9) {
    const pad = Math.abs(high) * 0.1 || 1;
    low -= pad; high += pad;
  } else {
    const pad = (high - low) * 0.14;
    low -= pad; high += pad;
  }
  if (floor >= 0) low = Math.max(0, low); // counts never go negative
  const x = timestamp => Math.max(0, Math.min(plotWidth, (timestamp - start) / WINDOW_MS * plotWidth));
  const y = value => top + (1 - (value - low) / (high - low)) * (bottom - top);
  context.fillStyle = labelColor;
  context.font = font;
  context.textAlign = 'right';
  context.textBaseline = 'top';
  context.fillText(tickLabel(high, high - low), width, top - 3);
  context.textBaseline = 'bottom';
  context.fillText(tickLabel(low, high - low), width, bottom + 3);
  if (visible.length > 1) {
    const area = context.createLinearGradient(0, top, 0, bottom);
    area.addColorStop(0, rgba(color, 0.24));
    area.addColorStop(1, rgba(color, 0));
    context.beginPath();
    context.moveTo(x(visible[0].t), bottom);
    for (const point of visible) context.lineTo(x(point.t), y(point.v));
    context.lineTo(x(visible[visible.length - 1].t), bottom);
    context.closePath();
    context.fillStyle = area;
    context.fill();
    context.beginPath();
    visible.forEach((point, index) => { if (index) context.lineTo(x(point.t), y(point.v)); else context.moveTo(x(point.t), y(point.v)); });
    context.strokeStyle = color;
    context.lineWidth = 1.8;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    context.shadowColor = rgba(color, 0.5);
    context.shadowBlur = 8;
    context.stroke();
    context.shadowBlur = 0;
  }
  const latest = visible[visible.length - 1];
  const lx = x(latest.t), ly = y(latest.v);
  context.fillStyle = rgba(color, 0.18);
  context.beginPath(); context.arc(lx, ly, 7, 0, Math.PI * 2); context.fill();
  context.fillStyle = color;
  context.beginPath(); context.arc(lx, ly, 3, 0, Math.PI * 2); context.fill();
}
function initTrendCanvases() {
  if (typeof ResizeObserver !== 'function' || typeof requestAnimationFrame !== 'function') return;
  if (typeof window.matchMedia === 'function' && window.matchMedia('(prefers-reduced-motion: reduce)').matches) trendInterval = 1000;
  TREND_PANELS.forEach(panel => panel.lanes.forEach(([key, field, , color]) => {
    const canvas = el(`canvas-${key}-${field}`);
    if (!canvas || typeof canvas.getContext !== 'function') return;
    const lane = { key, field, color, canvas, ctx: canvas.getContext('2d'), width: 0, height: 0 };
    if (!lane.ctx) return;
    new ResizeObserver(([entry]) => {
      lane.width = entry.contentRect.width;
      lane.height = entry.contentRect.height;
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(lane.width * ratio));
      canvas.height = Math.max(1, Math.round(lane.height * ratio));
      lane.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      drawLane(lane);
    }).observe(canvas);
    trendLanes.push(lane);
  }));
}
function trendLoop(time) {
  if (currentTab !== 'trends') { trendFrame = 0; return; }
  if (time - trendLast >= trendInterval) { trendLast = time; trendLanes.forEach(drawLane); }
  trendFrame = requestAnimationFrame(trendLoop);
}
function updateTrendLoop() {
  if (currentTab === 'trends' && !trendFrame && trendLanes.length && typeof requestAnimationFrame === 'function') trendFrame = requestAnimationFrame(trendLoop);
}

/* ---------- Sensors ---------- */
function renderSensors(now) {
  const cards=[];
  groups.forEach(([key,title,owner,note,fields])=>{
    if(category!=='all'&&category!==key)return;
    const [root,section]=sectionTopics[title];
    const card=document.createElement('section');card.className='card sensor-panel';card.id='section-'+title.toLowerCase().replaceAll(/[^a-z0-9]+/g,'-');
    const head=text(card,'div','','card-header');const titleBox=text(head,'div','');
    text(titleBox,'h2',title,'card-title');
    const supplied=fields.filter(field=>packet[key]?.[field]!==undefined&&packet[key]?.[field]!==null).length;
    text(titleBox,'p',`${SUBSYSTEM_LABEL[key]} · ${supplied} of ${fields.length} supplied`,'card-meta');
    text(head,'span',`B${owner}`,'owner');
    const list=text(card,'dl','','fields');
    fields.forEach(field=>{
      const [label,unit]=fieldLabel(field);
      const status=readingState(key,field,now);
      // Node structure [dt, dd] and the title are read by test_dashboard_display.cjs.
      const row=text(list,'div','',`field-row reading-${status}`);
      row.title=`Display field: ${topicRoots[root]}/${section}/${field}`;
      const dt=text(row,'dt',label);
      if(status==='stale')text(dt,'span','Stale','field-state');
      const dd=text(row,'dd',display(packet[key]?.[field]));
      if(unit)text(dd,'span',unit,unit==='°'?'unit deg':'unit');
    });
    text(card,'p',note,'card-caption');
    cards.push(card);
  });
  el('panels').replaceChildren(...cards);
}
function setCategory(id) {
  category = id || 'all';
  el('category').value = category;
  renderSensors(Date.now());
}

/* ---------- Events ---------- */
function renderEvents() {
  const events=Array.isArray(packet.events)?packet.events.slice(-100).reverse():[];
  el('event-count').textContent=`${events.length} events`;
  el('tab-count-events').textContent=String(events.length);
  el('tab-count-events').hidden=!events.length;
  setHTML('events', events.length
    ? `<ol class="event-list">${events.map(event=>{
      const item=event||{};
      const level=item.level==null?'info':String(item.level);
      const severity=/error|fault/i.test(level)?'error':/warn/i.test(level)?'warn':'info';
      return `<li class="event severity-${severity}"><span class="event-time">${esc(display(item.time))}</span><span class="event-level">${esc(level)}</span>`
        +`<div class="event-body"><p>${esc(display(item.message))}</p>${item.source!=null?`<span class="event-source">${esc(item.source)}</span>`:''}</div></li>`;
    }).join('')}</ol>`
    : `<div class="empty-events">${icon('list',20)}<p>No events received.</p></div>`);
}

/* ---------- Header, notice and footer ---------- */
function connectionText() {
  if (!isLive()) return mode === 'Demo' ? 'Demo snapshot' : 'Imported snapshot';
  if (!receiver.polling) return mode === 'MQTT' ? 'MQTT data supplied' : 'Awaiting data';
  return receiver.error ? 'Dashboard receiver unavailable' : String(receiver.status);
}
function noticeFor(now) {
  if (!isLive()) return ['preview', `${mode === 'Demo' ? 'DEMO · Synthetic values' : 'IMPORTED · File snapshot'}. Live updates are paused. Resume live to return to the MQTT receiver.`];
  if (!receiver.polling) return mode === 'MQTT'
    ? ['live', 'MQTT topic updates supplied · Broker connection is managed by the transport adapter.']
    : ['stale', 'Dashboard shell · No robot connection. Values appear when telemetry is supplied.'];
  if (receiver.error) return ['stale', 'Start dashboard.py and open its HTTP URL to receive MQTT updates.'];
  if (receiver.status !== 'MQTT subscribed') return ['stale', `${receiver.status} · Displayed values may be from earlier messages.`];
  const subscribed = `Subscribed to ${receiver.mqttTopic} · ${receiver.receivedPackets} packets decoded · ${receiver.rejectedPackets} rejected · Readings keep their latest value (marked stale after 5 s without an update); mission availability flags indicate sensor validity.`;
  if (receiver.lastReceivedAt === null) return ['stale', `Waiting for telemetry · ${subscribed}`];
  if (now - receiver.lastReceivedAt > STALE_AFTER_MS) return ['stale', `No recent packets · ${subscribed}`];
  return ['live', subscribed];
}
function renderChrome(now) {
  const label = connectionText();
  el('connection').className = `chip connection-badge connection-${connectionClass(now)}`;
  el('connection').title = label;
  el('connection-text').textContent = label;
  const [tone, message] = noticeFor(now);
  el('notice').className = `notice notice-${tone}`;
  setHTML('notice-icon', icon(tone === 'live' ? 'pulse' : 'alert', 16));
  el('notice-text').textContent = message;
  if (receiver.polling) {
    el('updated').textContent = receiver.lastReceivedAt
      ? `Last packet received: ${new Date(receiver.lastReceivedAt).toLocaleTimeString()}`
      : 'Last packet received: —';
  } else {
    el('updated').textContent = renderedAt ? `Last received: ${new Date(renderedAt).toLocaleTimeString()}` : 'Last received: —';
  }
  el('resume').hidden = isLive();
}

/* ---------- Render ---------- */
function paint() {
  const now = Date.now();
  for (const renderer of [renderMission, renderOverview, renderBarcodeLive, renderLive, renderTrendValues, renderSensors, renderEvents, renderChrome]) {
    try { renderer(now); } catch (error) { console.error(`Dashboard ${renderer.name} failed:`, error); }
  }
}
function render(nextPacket={}, nextMode='Waiting') {
  if(nextMode!=='MQTT')mqttPacket={};
  packet=nextPacket||{};
  mode=nextMode;
  renderedAt=nextMode==='Waiting'?null:Date.now();
  paint();
}
function validate(packet){if(!packet||typeof packet!=='object'||Array.isArray(packet))throw new Error('Expected a JSON object.');groups.forEach(([key])=>{if(packet[key]!=null&&(typeof packet[key]!=='object'||Array.isArray(packet[key])))throw new Error(`${key} must be an object.`);});if(packet.events!=null&&(!Array.isArray(packet.events)||packet.events.some(e=>!e||typeof e!=='object'||Array.isArray(e))))throw new Error('events must be an array of objects.');return packet;}
function showImportError(message) {
  el('import-error').textContent = message;
  el('import-error').hidden = !message;
}
// Clear this browser's readings and history, return to live mode and re-read the receiver snapshot.
function resumeLive() {
  fieldTimes = {}; samples = {}; lastSequence = 0;
  showImportError('');
  render();
}
el('demo').addEventListener('click',()=>{render(DEMO_PACKET,'Demo');showImportError('');});
el('reset').addEventListener('click',resumeLive);
el('resume').addEventListener('click',resumeLive);
el('import-button').addEventListener('click',()=>el('import').click());
el('import').addEventListener('change',async event=>{const file=event.target.files[0];if(!file)return;try{if(file.size>1048576)throw new Error('Use a file smaller than 1 MB.');render(validate(JSON.parse(await file.text())),'Imported');showImportError('');}catch(error){showImportError(`Import failed: ${error.message}`);}finally{event.target.value='';}});
el('category').addEventListener('change',event=>setCategory(event.target.value));
el('subsystem-card').addEventListener('click',event=>{
  const row=event.target&&event.target.closest?event.target.closest('[data-category]'):null;
  if(!row)return;
  setCategory(row.getAttribute('data-category'));
  selectTab('sensors');
});
function receiveMqtt(topic,payload) {
  const binding=topicBindings.find(item=>item.topic===topic);
  if(!binding && topic!==eventTopic)return false;
  let value=payload;
  if(typeof payload==='string') {
    try { value=JSON.parse(payload); } catch { value=payload; }
  }
  if(topic===eventTopic) {
    if(!value || typeof value!=='object' || Array.isArray(value))throw new Error('Event payload must be a JSON object.');
    mqttPacket.events=[...(mqttPacket.events || []),value].slice(-100);
  } else {
    if(value!==null && !['string','number','boolean'].includes(typeof value))throw new Error('Reading payload must be a scalar value.');
    if(typeof value==='number' && !Number.isFinite(value))throw new Error('Reading must be finite.');
    mqttPacket[binding.key] ??= {};
    mqttPacket[binding.key][binding.field]=value;
    const id=`${binding.key}.${binding.field}`, now=Date.now();
    if(value!==null)fieldTimes[id]=now;
    if(typeof value==='number'){const list=samples[id]??=[];list.push({t:now,v:value});if(list.length>HISTORY_LIMIT)list.shift();}
  }
  if(!batching)render(mqttPacket,'MQTT');
  return true;
}

/* ---------- Tabs (browser only) ---------- */
function selectTab(id, scroll = true, writeHash = true) {
  if (!TABS.includes(id)) return;
  const changed = id !== currentTab;
  currentTab = id;
  TABS.forEach(tab => {
    const on = tab === id;
    el(`tab-${tab}`).setAttribute('aria-selected', String(on));
    el(`tab-${tab}`).tabIndex = on ? 0 : -1;
    el(`panel-${tab}`).hidden = !on;
  });
  if (writeHash && location.hash !== `#${id}`) window.history.replaceState(null, '', `#${id}`);
  if (scroll && changed) {
    // Bring the new view's top into sight if the page was scrolled past it.
    const header = document.querySelector('.topbar');
    const top = el('tab-panels').getBoundingClientRect().top + window.scrollY - (header ? header.offsetHeight : 0) - 16;
    if (window.scrollY > top) window.scrollTo({ top, behavior: 'instant' });
  }
  updateTrendLoop();
}
TABS.forEach(id => el(`tab-${id}`).addEventListener('click', () => selectTab(id)));
el('tablist').addEventListener('keydown', event => {
  const index = TABS.indexOf(currentTab);
  const next = { ArrowRight: (index + 1) % TABS.length, ArrowLeft: (index - 1 + TABS.length) % TABS.length, Home: 0, End: TABS.length - 1 }[event.key];
  if (next === undefined) return;
  event.preventDefault();
  selectTab(TABS[next]);
  el(`tab-${TABS[next]}`).focus();
});

buildTrends();
renderBarcodeHistory([]);
renderBarcodeMap(null);
window.dashboard={
  setTelemetry:packet=>render(validate(packet),'Imported'),
  renderBarcodeHistory,
  renderBarcodeMap,
  receiveMqtt,
  subscriptions:[wireTopic],
  topicBindings,
  eventTopic
};
render();

// Browser-only features. The display test runs this file with a minimal DOM that has none of them.
if (typeof window.addEventListener === 'function') {
  const tabFromHash = () => { const id = String(location.hash || '').slice(1); return TABS.includes(id) ? id : null; };
  window.addEventListener('hashchange', () => { const id = tabFromHash(); if (id) selectTab(id, false); });
  initTrendCanvases();
  selectTab(tabFromHash() || currentTab, false, false);
  if (typeof setInterval === 'function') setInterval(paint, 1000); // staleness clock
}

// Python owns the MQTT TCP subscription; the browser polls latest readings.
if(receiver.polling) {
  const clock=()=>typeof performance!=='undefined'?performance.now():Date.now();
  async function pollBroker() {
    const controller=typeof AbortController==='function'?new AbortController():null;
    const timeout=controller?setTimeout(()=>controller.abort(),5000):null;
    const started=clock();
    try {
      const response=await fetch('/api/telemetry',{cache:'no-store',signal:controller?controller.signal:undefined});
      if(!response.ok)throw new Error('Receiver unavailable');
      const snapshot=await response.json();
      receiver.latency=Math.round(clock()-started);
      if(receiverId!==snapshot.receiver_id) {
        if(receiverId!==null){fieldTimes={};samples={};if(isLive())render();}
        receiverId=snapshot.receiver_id;
        lastSequence=0;
      }
      Object.assign(receiver,{error:false,status:String(snapshot.status),mqttTopic:snapshot.mqtt_topic||wireTopic,
        receivedPackets:snapshot.received_packets,rejectedPackets:snapshot.rejected_packets,
        lastReceivedAt:snapshot.last_received_at?snapshot.last_received_at*1000:null});
      const messages=Array.isArray(snapshot.messages)?snapshot.messages:[];
      let applied=false;
      // Demo and imported snapshots pause live ingestion until Resume live.
      if(isLive()) {
        batching=true;
        try {
          for(const message of messages) {
            if(message.sequence<=lastSequence)continue;
            try { if(receiveMqtt(message.topic,message.payload))applied=true; }
            catch(error) { console.warn('Rejected MQTT message:',message.topic,error.message); }
          }
        } finally { batching=false; }
        if(messages.length)lastSequence=Math.max(lastSequence,...messages.map(message=>message.sequence));
      }
      renderBarcodeHistory(snapshot.barcode_history);
      renderBarcodeMap(snapshot.barcode_map);
      if(applied)render(mqttPacket,'MQTT');else paint();
    } catch(error) {
      receiver.error=true;receiver.latency=null;
      paint();
    } finally { if(timeout)clearTimeout(timeout); setTimeout(pollBroker,250); }
  }
  pollBroker();
}

// Explicit button presses are the only source of robot command requests.
async function postCommand(body) {
  const response = await fetch('/api/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  });
  const text = await response.text();
  let result;
  try { result = JSON.parse(text); }
  catch { throw new Error(`Dashboard returned HTTP ${response.status} instead of JSON. Open http://127.0.0.1:8001/ and reload the page.`); }
  if (!response.ok) throw new Error(result.error || 'Command failed.');
  return result;
}
// Each button disables only itself, so STOP is never blocked by another request in flight.
for (const action of ['reset', 'start', 'stop']) {
  document.getElementById('robot-' + action).addEventListener('click', async () => {
    const status = document.getElementById('command-status');
    const button = document.getElementById('robot-' + action);
    button.disabled = true;
    status.textContent = 'Sending ' + action + '…';
    try {
      status.textContent = (await postCommand({action})).message;
    } catch (error) {
      status.textContent = error.message;
    } finally {
      button.disabled = false;
    }
  });
}
// Barcode actions: one request at a time; the codebook follows accepted ACKs only.
const mapButtons = ['map-send', 'map-reset', 'map-send-all'];
async function sendMap(body, label) {
  mapButtons.forEach(id => { el(id).disabled = true; });
  el('map-status').textContent = `Sending ${label}…`;
  try {
    const result = await postCommand(body);
    el('map-status').textContent = result.message;
    if (result.barcode_map) renderBarcodeMap(result.barcode_map);
  } catch (error) {
    el('map-status').textContent = error.message;
  } finally {
    mapButtons.forEach(id => { el(id).disabled = false; });
  }
}
el('map-send').addEventListener('click', () => {
  const letter = String(el('map-letter').value || '').trim().toUpperCase();
  if (!/^[A-Z]$/.test(letter)) { el('map-status').textContent = 'Enter one letter A-Z.'; return; }
  const turn = el('map-turn').value;
  const names = {forward: 'Forward', left: 'Left', right: 'Right', uturn: 'U-turn', ignore: 'Ignore'};
  sendMap({action: 'map', letter, turn}, `${letter} = ${names[turn] || turn}`);
});
el('map-letter').addEventListener('keydown', event => { if (event.key === 'Enter') el('map-send').click(); });
el('map-letter').addEventListener('input', () => renderCodebook());
el('map-reset').addEventListener('click', () => sendMap({action: 'map_reset'}, 'codebook reset'));
el('map-send-all').addEventListener('click', () => sendMap({action: 'send_all'}, 'all mappings'));
// A codebook tile only loads its letter and current action into the form; Send applies it.
el('barcode-map').addEventListener('click', event => {
  const button = event.target && event.target.closest ? event.target.closest('.letter-tile') : null;
  if (!button) return;
  const letter = button.getAttribute('data-letter');
  el('map-letter').value = letter;
  el('map-turn').value = TURN_VALUES[barcodeMap && barcodeMap[letter]] || 'ignore';
  el('map-status').textContent = `${letter} selected. Choose an action and press Send.`;
  renderCodebook();
  el('map-turn').focus();
});
