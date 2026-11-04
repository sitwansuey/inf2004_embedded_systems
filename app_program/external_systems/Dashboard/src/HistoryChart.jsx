import { useEffect, useRef } from 'react';

const GUTTER = 48;
const GRID = 'rgba(150, 175, 230, 0.09)';
const LABEL = '#6a7aa1';
const MONO = '10.5px "JetBrains Mono", "Cascadia Mono", "SF Mono", Consolas, monospace';

function rgba(hex, alpha) {
  const value = Number.parseInt(hex.slice(1), 16);
  return `rgba(${value >> 16 & 255}, ${value >> 8 & 255}, ${value & 255}, ${alpha})`;
}
// One precision per lane, chosen from the visible span, so both axis labels read alike.
function tick(value, span) {
  return value.toFixed(span >= 20 ? 0 : span >= 2 ? 1 : 2);
}

/** One lane of received history drawn on canvas from a telemetry source. Animates only while its tab is shown. */
function Lane({ source, label, color, unit, value, status, windowMs, active }) {
  const canvasRef = useRef(null);
  useEffect(() => {
    if (!active) return undefined;
    const canvas = canvasRef.current;
    const context = canvas.getContext('2d');
    const points = source.getHistory();
    const unsubscribe = source.subscribe(point => {
      points.push(point);
      if (points.length > 240) points.shift();
    });
    const reduceMotion = window.matchMedia?.('(prefers-reduced-motion: reduce)').matches;
    let width = 0, height = 0, frame = 0, last = 0;

    function draw() {
      if (!width || !height) return;
      context.clearRect(0, 0, width, height);
      const plotWidth = Math.max(10, width - GUTTER);
      const top = 8, bottom = height - 8;
      context.strokeStyle = GRID;
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
      const start = now - windowMs;
      const visible = points.filter(point => point.timestamp >= start - 1000 && point.timestamp <= now + 1000);
      if (!visible.length) {
        context.fillStyle = LABEL;
        context.font = MONO;
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText('Awaiting readings', plotWidth / 2, height / 2);
        return;
      }
      const floor = Math.min(...visible.map(point => point.value));
      let low = floor;
      let high = Math.max(...visible.map(point => point.value));
      if (high - low < 1e-9) {
        const pad = Math.abs(high) * 0.1 || 1;
        low -= pad; high += pad;
      } else {
        const pad = (high - low) * 0.14;
        low -= pad; high += pad;
      }
      if (floor >= 0) low = Math.max(0, low); // counts never go negative
      const x = timestamp => Math.max(0, Math.min(plotWidth, (timestamp - start) / windowMs * plotWidth));
      const y = sample => top + (1 - (sample - low) / (high - low)) * (bottom - top);

      context.fillStyle = LABEL;
      context.font = MONO;
      context.textAlign = 'right';
      context.textBaseline = 'top';
      context.fillText(tick(high, high - low), width, top - 3);
      context.textBaseline = 'bottom';
      context.fillText(tick(low, high - low), width, bottom + 3);

      if (visible.length > 1) {
        const area = context.createLinearGradient(0, top, 0, bottom);
        area.addColorStop(0, rgba(color, 0.24));
        area.addColorStop(1, rgba(color, 0));
        context.beginPath();
        context.moveTo(x(visible[0].timestamp), bottom);
        for (const point of visible) context.lineTo(x(point.timestamp), y(point.value));
        context.lineTo(x(visible.at(-1).timestamp), bottom);
        context.closePath();
        context.fillStyle = area;
        context.fill();

        context.beginPath();
        visible.forEach((point, index) => {
          const method = index ? 'lineTo' : 'moveTo';
          context[method](x(point.timestamp), y(point.value));
        });
        context.strokeStyle = color;
        context.lineWidth = 1.8;
        context.lineJoin = 'round';
        context.lineCap = 'round';
        context.shadowColor = rgba(color, 0.5);
        context.shadowBlur = 8;
        context.stroke();
        context.shadowBlur = 0;
      }
      const latest = visible.at(-1);
      const lx = x(latest.timestamp), ly = y(latest.value);
      context.fillStyle = rgba(color, 0.18);
      context.beginPath(); context.arc(lx, ly, 7, 0, Math.PI * 2); context.fill();
      context.fillStyle = color;
      context.beginPath(); context.arc(lx, ly, 3, 0, Math.PI * 2); context.fill();
    }

    const observer = new ResizeObserver(([entry]) => {
      width = entry.contentRect.width;
      height = entry.contentRect.height;
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(width * ratio));
      canvas.height = Math.max(1, Math.round(height * ratio));
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    });
    observer.observe(canvas);
    const interval = reduceMotion ? 1000 : 33;
    const loop = time => {
      if (time - last >= interval) { last = time; draw(); }
      frame = requestAnimationFrame(loop);
    };
    frame = requestAnimationFrame(loop);
    return () => { cancelAnimationFrame(frame); observer.disconnect(); unsubscribe(); };
  }, [source, color, windowMs, active]);

  return <div className={`lane reading-${status}`}>
    <div className="lane-head">
      <span className="lane-label"><i className="swatch" style={{ background: color }} aria-hidden="true" />{label}</span>
      <span className="lane-value">{value}{unit && <small>{unit}</small>}</span>
    </div>
    <canvas ref={canvasRef} role="img" aria-label={`${label} received history, latest ${value}${unit ? ` ${unit}` : ''}`} />
  </div>;
}

/** Card of stacked history lanes sharing one reception-time axis. */
export function HistoryPanel({ title, caption, lanes, epoch, active = true, windowMs = 60000 }) {
  return <section className="card chart-card">
    <div className="card-header"><h2 className="card-title">{title}</h2><span className="chip plain">{Math.round(windowMs / 1000)} s window</span></div>
    <div className="chart-stack" key={epoch}>
      {lanes.map(lane => <Lane key={lane.id} {...lane} windowMs={windowMs} active={active} />)}
    </div>
    <div className="chart-axis" aria-hidden="true"><span>−60 s</span><span>−45 s</span><span>−30 s</span><span>−15 s</span><span>now</span></div>
    <p className="card-caption">{caption}</p>
  </section>;
}
