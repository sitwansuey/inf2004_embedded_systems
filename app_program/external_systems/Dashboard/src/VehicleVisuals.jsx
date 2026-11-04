import { useId } from 'react';
import { readingState } from './telemetry.js';
import { StatusChip, Tile } from './ui.jsx';

const COLORS = {
  grid: 'rgba(150, 175, 230, 0.16)', gridStrong: 'rgba(150, 175, 230, 0.3)', label: '#7d8db3',
  cyan: '#38d9f5', mint: '#5cf2bf', purple: '#a58bff', idle: '#5a6a90',
};
const finite = value => typeof value === 'number' && Number.isFinite(value);
const number = (value, digits = 0) => finite(value) ? value.toFixed(digits) : '—';
const safeId = id => id.replace(/[^a-zA-Z0-9]/g, '');

/** Draw one received ultrasonic observation on the servo's 180° field. EDR16 does not provide scan history. */
export function ProximityRadar({ state, now = Date.now() }) {
  const id = safeId(useId());
  const distance = state.packet.obstacle?.distance_cm;
  const servo = state.packet.obstacle?.servo_angle_deg;
  const hasRange = finite(distance) && distance >= 0;
  const hasServo = finite(servo) && servo >= 0 && servo <= 180;
  const distanceStatus = readingState(state, 'obstacle', 'distance_cm', now);
  const servoStatus = readingState(state, 'obstacle', 'servo_angle_deg', now);
  const status = !hasRange ? 'missing'
    : distanceStatus === 'stale' || (hasServo && servoStatus === 'stale') ? 'stale'
      : distanceStatus;
  const plotted = hasRange && hasServo;
  const displayRange = Math.max(100, Math.ceil((hasRange ? distance : 0) / 100) * 100);
  // A servo angle of 0 points left; 90 points forward; 180 points right.
  const CX = 220, CY = 222, R = 184;
  const polar = (degrees, radius) => {
    const angle = degrees * Math.PI / 180;
    return { x: CX - Math.cos(angle) * radius, y: CY - Math.sin(angle) * radius };
  };
  const bearing = hasServo ? servo : 90;
  const edge = polar(bearing, R);
  const beamStart = polar(bearing - 9, R), beamEnd = polar(bearing + 9, R);
  const observation = polar(bearing, Math.min(1, (hasRange ? distance : 0) / displayRange) * R);
  const labelLeft = observation.x > CX + 60;
  const caption = !hasRange ? 'Waiting for an ultrasonic range reading.'
    : !hasServo ? 'Range received. A servo angle is needed to plot its bearing.'
      : 'Latest received range at the servo bearing. No scan history.';
  const accessibleLabel = !hasRange ? 'Proximity radar: awaiting ultrasonic data'
    : !hasServo ? `Proximity radar: ${number(distance)} centimetres, servo angle unavailable`
      : `Proximity radar: ${number(distance)} centimetres at servo ${number(servo)} degrees${status === 'stale' ? ', stale reading' : ''}`;
  const arc = radius => `M${CX - radius} ${CY}A${radius} ${radius} 0 0 1 ${CX + radius} ${CY}`;

  return <section id="proximity-radar" className="card visual-card" data-state={status} data-bearing={plotted ? servo : undefined}>
    <div className="card-header"><h2 className="card-title">Proximity radar</h2><StatusChip status={status} /></div>
    <div className="visual-stage">
      <svg viewBox="0 0 440 256" role="img" aria-label={accessibleLabel}>
        <defs>
          <radialGradient id={`${id}-field`} cx={CX} cy={CY} r={R} gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={COLORS.cyan} stopOpacity="0.14" />
            <stop offset="100%" stopColor={COLORS.cyan} stopOpacity="0.02" />
          </radialGradient>
          <radialGradient id={`${id}-beam`} cx={CX} cy={CY} r={R} gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={COLORS.mint} stopOpacity="0.5" />
            <stop offset="100%" stopColor={COLORS.mint} stopOpacity="0.04" />
          </radialGradient>
        </defs>
        <path d={`${arc(R)}Z`} fill={`url(#${id}-field)`} />
        <g fill="none" stroke={COLORS.grid} strokeWidth="1.5">
          {[0.25, 0.5, 0.75].map(fraction => <path key={fraction} d={arc(R * fraction)} strokeDasharray="3 6" />)}
          <path d={arc(R)} stroke={COLORS.gridStrong} />
          {[30, 60, 90, 120, 150].map(degrees => { const end = polar(degrees, R); return <line key={degrees} x1={CX} y1={CY} x2={end.x} y2={end.y} />; })}
          <line x1={CX - R} y1={CY} x2={CX + R} y2={CY} stroke={COLORS.gridStrong} />
        </g>
        <g fill={COLORS.label} fontSize="15" fontFamily="inherit" aria-hidden="true">
          <text x={CX} y="24" textAnchor="middle" letterSpacing="1">FORWARD · 90°</text>
          <text x={CX - R} y={CY + 18} textAnchor="start">0°</text>
          <text x={CX + R} y={CY + 18} textAnchor="end">180°</text>
          {[0.5, 1].map(fraction => <text key={fraction} x={CX + R * fraction - 4} y={CY - 6} textAnchor="end" fontSize="14">{Math.round(displayRange * fraction)} cm</text>)}
        </g>
        {plotted && <g opacity={status === 'stale' ? 0.35 : 1} data-testid="radar-observation">
          <path d={`M${CX} ${CY}L${beamStart.x} ${beamStart.y}A${R} ${R} 0 0 1 ${beamEnd.x} ${beamEnd.y}Z`} fill={`url(#${id}-beam)`} />
          <line x1={CX} y1={CY} x2={edge.x} y2={edge.y} stroke={COLORS.mint} strokeWidth="2.5" strokeLinecap="round" />
          <circle cx={observation.x} cy={observation.y} r="17" fill={COLORS.purple} fillOpacity="0.14" stroke={COLORS.purple} strokeOpacity="0.55" strokeWidth="1.5" />
          <circle cx={observation.x} cy={observation.y} r="6.5" fill={COLORS.purple} />
          <text x={observation.x + (labelLeft ? -24 : 24)} y={observation.y + 6} textAnchor={labelLeft ? 'end' : 'start'} fill="#e4dcff" fontSize="19" fontWeight="600" fontFamily="inherit">{number(distance)} cm</text>
        </g>}
        <rect x={CX - 19} y={CY + 4} width="38" height="26" rx="8" fill="#0f1d3f" stroke={COLORS.gridStrong} strokeWidth="1.5" />
        <circle cx={CX} cy={CY} r="7" fill={plotted ? COLORS.mint : COLORS.idle} stroke="#0b1631" strokeWidth="3" />
      </svg>
    </div>
    <div className="tiles tiles-3">
      <Tile label="Range" className={distanceStatus}>{hasRange ? number(distance) : '—'}<small>cm</small></Tile>
      <Tile label="Servo bearing" className={hasServo ? servoStatus : 'missing'}>{hasServo ? number(servo) : '—'}<small className="deg">°</small></Tile>
      <Tile label="Display range">{displayRange}<small>cm</small></Tile>
    </div>
    <p className={`card-caption ${status === 'stale' ? 'stale' : ''}`}>{status === 'stale' ? `Stale. ${caption}` : caption}</p>
  </section>;
}

/** Heading is only available when telemetry explicitly supplies a heading field. */
export function VehicleOrientation({ state, now = Date.now() }) {
  const id = safeId(useId());
  const terrain = state.packet.terrain || {};
  const hasHeading = finite(terrain.heading_deg);
  const heading = hasHeading ? ((terrain.heading_deg % 360) + 360) % 360 : 0;
  const status = hasHeading ? readingState(state, 'terrain', 'heading_deg', now) : 'unavailable';
  const fields = [
    ['heading_deg', 'Heading', hasHeading ? heading : undefined],
    ['pitch_deg', 'Pitch', terrain.pitch_deg],
    ['roll_deg', 'Roll', terrain.roll_deg],
  ];
  const accessibleLabel = hasHeading
    ? `Vehicle orientation: heading ${number(heading)} degrees${status === 'stale' ? ', stale reading' : ''}`
    : 'Vehicle orientation: heading unavailable; vehicle shown in a neutral position';
  const ticks = Array.from({ length: 24 }, (_, index) => index * 15);

  return <section id="vehicle-orientation" className="card visual-card" data-state={status} data-heading={hasHeading ? heading : undefined}>
    <div className="card-header"><h2 className="card-title">Vehicle orientation</h2><StatusChip status={status} /></div>
    <div className="visual-stage">
      <svg viewBox="0 0 440 440" role="img" aria-label={accessibleLabel}>
        <defs>
          <radialGradient id={`${id}-dial`} cx="220" cy="220" r="200" gradientUnits="userSpaceOnUse">
            <stop offset="0%" stopColor={COLORS.purple} stopOpacity="0.1" />
            <stop offset="100%" stopColor={COLORS.purple} stopOpacity="0.01" />
          </radialGradient>
          <linearGradient id={`${id}-body`} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#26407a" />
            <stop offset="100%" stopColor="#13244c" />
          </linearGradient>
        </defs>
        <circle cx="220" cy="220" r="196" fill={`url(#${id}-dial)`} stroke={COLORS.gridStrong} strokeWidth="1.5" />
        <circle cx="220" cy="220" r="140" fill="none" stroke={COLORS.grid} strokeWidth="1.5" strokeDasharray="3 7" />
        <g strokeWidth="2" strokeLinecap="round">
          {ticks.map(degrees => <line key={degrees} x1="220" y1="30" x2="220" y2={degrees % 90 ? 40 : 50}
            stroke={degrees % 90 ? COLORS.grid : COLORS.gridStrong} transform={`rotate(${degrees} 220 220)`} />)}
        </g>
        {hasHeading && <g fill={COLORS.label} fontSize="24" fontWeight="600" fontFamily="inherit" textAnchor="middle" aria-hidden="true">
          <text x="220" y="72" fill={COLORS.mint}>N</text><text x="376" y="228">E</text>
          <text x="220" y="386">S</text><text x="64" y="228">W</text>
        </g>}
        <g transform={`rotate(${heading} 220 220)`} opacity={status === 'stale' ? 0.35 : 1} data-testid="vehicle-body">
          <path d="M220 128V106" stroke={hasHeading ? COLORS.mint : COLORS.idle} strokeWidth="5" strokeLinecap="round" />
          <path d="M207 110l13-18 13 18z" fill={hasHeading ? COLORS.mint : COLORS.idle} />
          <g fill="#0b1733" stroke="#3a5591" strokeWidth="1.5">
            <rect x="160" y="150" width="20" height="50" rx="7" />
            <rect x="260" y="150" width="20" height="50" rx="7" />
            <rect x="160" y="250" width="20" height="50" rx="7" />
            <rect x="260" y="250" width="20" height="50" rx="7" />
          </g>
          <rect x="176" y="128" width="88" height="194" rx="30" fill={`url(#${id}-body)`} stroke={COLORS.cyan} strokeWidth="3" />
          <rect x="192" y="174" width="56" height="66" rx="13" fill="#0a1530" stroke="#4d6aa8" strokeWidth="1.5" />
          <rect x="196" y="258" width="48" height="44" rx="10" fill="#0d1a38" />
          <rect x="204" y="136" width="32" height="12" rx="6" fill={hasHeading ? COLORS.mint : COLORS.idle} opacity="0.85" />
        </g>
      </svg>
    </div>
    <div className="tiles tiles-3">
      {fields.map(([field, label, value]) => {
        const fieldStatus = finite(value) ? readingState(state, 'terrain', field, now) : 'missing';
        return <Tile key={field} className={fieldStatus} data-field={field} label={`${label}${fieldStatus === 'stale' ? ' · stale' : ''}`}>
          {number(value, field === 'heading_deg' ? 0 : 1)}<small className="deg">°</small>
        </Tile>;
      })}
    </div>
    <p className={`card-caption ${status === 'stale' ? 'stale' : ''}`}>
      {!hasHeading ? 'Heading unavailable. Waiting for an explicit orientation estimate.'
        : status === 'stale' ? 'Stale heading. Showing the last received orientation.'
          : `${status === 'snapshot' ? 'Snapshot' : 'Received'} heading, clockwise from north.`}
    </p>
  </section>;
}
