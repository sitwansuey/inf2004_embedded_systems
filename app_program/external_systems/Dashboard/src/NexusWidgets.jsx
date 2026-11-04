import { useEffect, useRef, useState } from 'react';
import Nexus from 'nexusui';

const sourceIds = new WeakMap();
let nextSourceId = 0;
function sourceId(source) {
  if (!sourceIds.has(source)) sourceIds.set(source, ++nextSourceId);
  return sourceIds.get(source);
}

function numericHistory(source) {
  return source.getHistory().filter(point => Number.isFinite(point.value) && Number.isFinite(point.timestamp));
}

function formatValue(value) {
  return Number.isFinite(value) ? Number(value.toFixed(3)).toLocaleString() : '—';
}

// Nexus widgets normally accept drag input. These displays only observe the
// telemetry store; they must never turn a local gesture into a motor command.
function makeReadOnly(widget) {
  for (const method of ['preClick', 'preMove', 'preRelease', 'preTouch', 'preTouchMove', 'preTouchRelease']) {
    widget[method] = () => {};
  }
  widget.element.style.pointerEvents = 'none';
  widget.element.style.cursor = 'default';
  widget.element.setAttribute('aria-hidden', 'true');
  widget.element.setAttribute('focusable', 'false');
  widget.element.removeAttribute('tabindex');
  widget.parent.inert = true;
}

export function NexusDial({ source, min = 0, max = 100, color = '#83d8e8', label = 'Reading', size = 104 }) {
  const visual = useRef(null);
  const [value, setValue] = useState(() => numericHistory(source).at(-1)?.value);

  useEffect(() => {
    const target = visual.current;
    const dial = new Nexus.Dial(target, { size: [size, size], min, max, step: 0, value: min });
    makeReadOnly(dial);
    dial.colorize('fill', '#202c40');
    dial.colorize('accent', color);
    // Keep the native ring and pointer; the accompanying reading supplies the
    // exact number and unit without implying that this is an editable control.
    dial.handleFill.style.display = 'none';
    dial.handle2Fill.style.display = 'none';

    function update() {
      const next = numericHistory(source).at(-1)?.value;
      dial.element.style.visibility = next === undefined ? 'hidden' : 'visible';
      if (next !== undefined) dial.value = next;
      setValue(next);
    }
    update();
    const unsubscribe = source.subscribe(update);
    const observer = new ResizeObserver(() => {
      const width = Math.min(size, target.parentElement.clientWidth || size);
      if (width > 0 && (dial.width !== width || dial.height !== width)) dial.resize(width, width);
    });
    observer.observe(target.parentElement);
    return () => {
      unsubscribe();
      observer.disconnect();
      dial.destroy();
    };
  }, [source, min, max, color, size]);

  return <div className="nexus-dial" role="img" aria-label={`${label}: ${formatValue(value)}. Read-only telemetry display.`}
    data-label={label} data-value={value ?? ''} data-available={value !== undefined}
    style={{ width: size, height: size, maxWidth: '100%', position: 'relative', pointerEvents: 'none' }}>
    <div ref={visual} className="nexus-dial-visual" aria-hidden="true" />
    {value === undefined && <span className="nexus-dial-empty" aria-hidden="true" style={{ position: 'absolute', inset: 0, display: 'grid', placeItems: 'center', border: '3px solid #2e435b', borderRadius: '50%', color: '#91a5c0' }}>—</span>}
  </div>;
}

function historiesFor(series, windowMs) {
  const histories = series.map(item => numericHistory(item.source));
  const newest = Math.max(0, ...histories.flatMap(history => history.map(point => point.timestamp)));
  // Anchor a retained snapshot to its newest reception, so a disconnected
  // receiver does not cause a still-visible history to scroll or fabricate data.
  const cutoff = newest - windowMs;
  return histories.map(history => history.filter(point => point.timestamp >= cutoff).slice(-240));
}

function historySummary(histories) {
  const values = histories.flatMap(history => history.map(point => point.value));
  const timestamps = histories.flatMap(history => history.map(point => point.timestamp));
  const low = values.length ? Math.min(...values) : 0;
  const high = values.length ? Math.max(...values) : 1;
  const padding = low === high ? Math.max(Math.abs(low) * 0.1, 1) : (high - low) * 0.12;
  return {
    histories, min: low - padding, max: high + padding,
    count: Math.max(0, ...histories.map(history => history.length)),
    available: histories.some(history => history.length >= 2),
    first: timestamps.length ? Math.min(...timestamps) : null,
    last: timestamps.length ? Math.max(...timestamps) : null,
  };
}

function receptionTime(timestamp) {
  return new Date(timestamp).toLocaleTimeString([], { hour12: false });
}

export function NexusHistory({ series, height = 220, windowMs = 60000, epoch }) {
  const frame = useRef(null);
  const layers = useRef([]);
  const [summary, setSummary] = useState(() => historySummary(historiesFor(series, windowMs)));
  // Callers can construct the series array while rendering. Only a change to
  // its sources or presentation rebuilds the native widgets, not every poll.
  const configuration = JSON.stringify(series.map(item => [sourceId(item.source), item.label, item.color, item.unit]));

  useEffect(() => {
    const width = Math.max(1, Math.round(frame.current.clientWidth));
    const widgets = series.map((item, index) => {
      const widget = new Nexus.Multislider(layers.current[index], {
        size: [width, height], numberOfSliders: 2, min: 0, max: 1,
        values: [0, 0], smoothing: 0, mode: 'line',
      });
      makeReadOnly(widget);
      widget.colorize('fill', 'transparent');
      widget.colorize('accent', item.color || '#83d8e8');
      widget.element.style.visibility = 'hidden';
      return widget;
    });

    function update() {
      const next = historySummary(historiesFor(series, windowMs));
      widgets.forEach((widget, index) => {
        const history = next.histories[index];
        const target = layers.current[index];
        target.dataset.samples = String(history.length);
        target.dataset.values = JSON.stringify(history.map(point => point.value));
        target.dataset.timestamps = JSON.stringify(history.map(point => point.timestamp));
        if (history.length < 2) {
          widget.element.style.visibility = 'hidden';
          return;
        }
        widget.min = next.min;
        widget.max = next.max;
        widget.setAllSliders(history.map(point => point.value));
        // Nexus rebuilds its native SVG when the sample count changes. Restyle
        // the rebuilt elements while keeping Nexus responsible for the line.
        widget.fill.style.display = 'none';
        widget.nodes.forEach(node => { node.style.display = 'none'; });
        widget.line.setAttribute('stroke-width', '2.5');
        widget.line.setAttribute('stroke-linecap', 'round');
        widget.line.setAttribute('stroke-linejoin', 'round');
        widget.element.style.visibility = 'visible';
      });
      setSummary(next);
    }
    update();
    const unsubscribers = series.map(item => item.source.subscribe(update));
    const observer = new ResizeObserver(() => {
      const nextWidth = Math.max(1, Math.round(frame.current.clientWidth));
      widgets.forEach(widget => {
        if (widget.width !== nextWidth || widget.height !== height) widget.resize(nextWidth, height);
      });
    });
    observer.observe(frame.current);
    return () => {
      observer.disconnect();
      unsubscribers.forEach(unsubscribe => unsubscribe());
      widgets.forEach(widget => widget.destroy());
    };
  }, [configuration, height, windowMs, epoch]);

  return <div className="nexus-history" data-samples={summary.count} data-available={summary.available}
    data-values={JSON.stringify(summary.histories.map(history => history.map(point => point.value)))}>
    <div className="nexus-history-body" style={{ position: 'relative', paddingLeft: 48 }}>
      <div className="nexus-history-scale" aria-hidden="true"><span>{formatValue(summary.max)}</span><span>{formatValue(summary.min)}</span></div>
      <div ref={frame} className="nexus-history-frame" role="img" aria-label={`Read-only history for ${series.map(item => item.label).join(', ')}. ${summary.count} received samples. Horizontal position is sample order, not elapsed time.`}
        style={{ position: 'relative', height, width: '100%', pointerEvents: 'none' }}>
        <svg className="nexus-history-grid" width="100%" height="100%" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true" style={{ position: 'absolute', inset: 0 }}>
          {[0, 25, 50, 75, 100].map(y => <line key={y} x1="0" x2="100" y1={y} y2={y} stroke="#30415a" strokeWidth="1" strokeDasharray="6 7" vectorEffect="non-scaling-stroke" />)}
        </svg>
        {series.map((item, index) => <div key={`${sourceId(item.source)}-${index}`} ref={node => { layers.current[index] = node; }}
          className="nexus-history-layer" data-label={item.label} aria-hidden="true" style={{ position: 'absolute', inset: 0, pointerEvents: 'none' }} />)}
        {!summary.available && <p className="nexus-history-empty" style={{ position: 'absolute', inset: 0, display: 'grid', placeItems: 'center', margin: 0 }}>Waiting for two received samples per line</p>}
      </div>
    </div>
    <div className="nexus-history-legend">{series.map((item, index) => <span key={`${sourceId(item.source)}-${index}`} style={{ color: item.color || '#83d8e8' }}><i style={{ background: item.color || '#83d8e8' }} />{item.label}<strong>{formatValue(summary.histories[index]?.at(-1)?.value)}{item.unit && ` ${item.unit}`}</strong></span>)}</div>
    <div className="nexus-history-caption"><span>Received sample order · up to 240 per channel</span><span>{summary.first == null ? 'No samples received' : `Received ${receptionTime(summary.first)}–${receptionTime(summary.last)}`}</span></div>
  </div>;
}
