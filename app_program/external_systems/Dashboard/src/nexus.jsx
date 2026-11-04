import { useEffect, useId, useRef } from 'react';
import Nexus from 'nexusui';

/** Shared dashboard palette. Navy carries the surfaces; cyan, mint and purple carry data. */
export const PALETTE = Object.freeze({
  navyDeep: '#060d1e',
  navy: '#0a1530',
  track: '#132650',
  slate: '#26407a',
  text: '#e9f0ff',
  cyan: '#38d9f5',
  mint: '#5cf2bf',
  purple: '#a58bff',
  azure: '#63a4ff',
  amber: '#f6c46a',
  coral: '#ff7d8e',
});

// NexusUI starts a Web Audio clock when it loads. The dashboard plays no audio, so release it.
try { Nexus.clock?.stop(); } catch { /* no audio clock */ }
try { Nexus.context?.close?.()?.catch?.(() => {}); } catch { /* audio unavailable */ }

Object.assign(Nexus.colors, {
  accent: PALETTE.cyan,
  fill: PALETTE.navy,
  dark: PALETTE.navyDeep,
  light: PALETTE.text,
  mediumLight: PALETTE.slate,
  mediumDark: PALETTE.track,
});

const SVG_NS = 'http://www.w3.org/2000/svg';
const finite = value => typeof value === 'number' && Number.isFinite(value);
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const svgNode = (name, attributes = {}) => {
  const node = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  return node;
};
const measure = (element, fallback) => Math.floor(element?.clientWidth || fallback);

/**
 * Mount a NexusUI widget into a host element and destroy it on unmount.
 * The host is display-only: CSS disables pointer input, so widgets never emit commands.
 */
function useNexus(create, { measureRef, onResize } = {}) {
  const host = useRef(null);
  const widget = useRef(null);
  useEffect(() => {
    const element = host.current;
    widget.current = create(element);
    let observer;
    if (measureRef?.current && onResize && typeof ResizeObserver !== 'undefined') {
      let last = measure(measureRef.current, 0);
      observer = new ResizeObserver(() => {
        const width = measure(measureRef.current, 0);
        if (width > 0 && width !== last && widget.current) {
          last = width;
          onResize(widget.current, width);
        }
      });
      observer.observe(measureRef.current);
    }
    return () => {
      observer?.disconnect();
      const instance = widget.current;
      widget.current = null;
      try { instance?.destroy(); } catch { /* already detached */ }
      element.replaceChildren();
    };
  }, []);
  return [host, widget];
}

/** Read-only NexusUI dial. Values outside the dial scale are pinned to its ends. */
export function NexusDial({ value, min, max, color, size = 104 }) {
  const [host, dial] = useNexus(element => {
    const widget = new Nexus.Dial(element, {
      size: [size, size], interaction: 'radial', mode: 'relative', min, max, step: 0, value: min,
    });
    widget.colorize('fill', PALETTE.navyDeep);
    widget.colorize('accent', color);
    return widget;
  });
  useEffect(() => {
    if (dial.current) dial.current.value = finite(value) ? clamp(value, min, max) : min;
  }, [value, min, max]);
  return <div className="nx nx-dial" ref={host} aria-hidden="true" />;
}

function decorateBars(widget, colors, id) {
  const svg = widget.element;
  if (!svg.querySelector('defs')) {
    const defs = svgNode('defs');
    colors.forEach((color, index) => {
      const gradient = svgNode('linearGradient', { id: `${id}-${index}`, x1: 0, y1: 0, x2: 0, y2: 1 });
      gradient.append(
        svgNode('stop', { offset: '0%', 'stop-color': color, 'stop-opacity': 0.95 }),
        svgNode('stop', { offset: '100%', 'stop-color': color, 'stop-opacity': 0.08 }),
      );
      defs.append(gradient);
    });
    const grid = svgNode('g', { class: 'nx-grid' });
    for (const y of ['25%', '50%', '75%']) grid.append(svgNode('line', { x1: '0', x2: '100%', y1: y, y2: y }));
    svg.insertBefore(grid, svg.firstChild);
    svg.insertBefore(defs, svg.firstChild);
  }
  const slot = widget.width / widget.values.length;
  const gap = Math.min(30, slot * 0.3);
  widget.bars.forEach((bar, index) => {
    bar.setAttribute('x', index * slot + gap / 2);
    bar.setAttribute('width', Math.max(4, slot - gap));
    bar.setAttribute('rx', 7);
    bar.setAttribute('fill', `url(#${id}-${index})`);
  });
  widget.caps.forEach((cap, index) => {
    cap.setAttribute('x', index * slot + gap / 2);
    cap.setAttribute('width', Math.max(4, slot - gap));
    cap.setAttribute('height', 3);
    cap.setAttribute('rx', 1.5);
    cap.setAttribute('fill', colors[index]);
  });
}

/** NexusUI multislider showing raw 12-bit IR readings as one bar per sensor. */
export function SensorBars({ readings, max = 4095, height = 150 }) {
  const wrap = useRef(null);
  const id = `bars${useId().replace(/[^a-zA-Z0-9]/g, '')}`;
  const colors = readings.map(reading => reading.color);
  const [host, bars] = useNexus(element => {
    const widget = new Nexus.Multislider(element, {
      size: [measure(wrap.current, 240), height], numberOfSliders: readings.length,
      min: 0, max, step: 0, candycane: 1, values: readings.map(() => 0), smoothing: 0, mode: 'bar',
    });
    widget.colorize('fill', 'transparent');
    decorateBars(widget, colors, id);
    return widget;
  }, { measureRef: wrap, onResize: (widget, width) => { widget.resize(width, height); decorateBars(widget, colors, id); } });
  const signature = readings.map(reading => `${reading.value}:${reading.status}`).join('|');
  useEffect(() => {
    const widget = bars.current;
    if (!widget) return;
    widget.setAllSliders(readings.map(reading => finite(reading.value) ? clamp(reading.value, 0, max) : 0));
    readings.forEach((reading, index) => {
      const opacity = reading.status === 'stale' ? 0.38 : reading.status === 'missing' ? 0 : 1;
      widget.bars[index]?.setAttribute('opacity', opacity);
      widget.caps[index]?.setAttribute('opacity', opacity);
    });
  }, [signature]);
  // Quarter steps of the full 12-bit range (0, 1024, 2048, 3072) plus the top value.
  const ticks = [max, ...[0.75, 0.5, 0.25].map(fraction => Math.round((max + 1) * fraction)), 0];
  return <div className="bars-chart">
    <div className="bars-scale" style={{ height }} aria-hidden="true">
      {ticks.map(tick => <span key={tick} style={{ top: `${(1 - tick / max) * 100}%` }}>{tick}</span>)}
    </div>
    <div className="nx-wrap" ref={wrap}><div className="nx nx-bars" ref={host} aria-hidden="true" /></div>
  </div>;
}

/** NexusUI position pad (X/Y) with a vertical slider rail (Z) for the accelerometer. */
export function AccelerationPad({ x, y, z, range = 20 }) {
  const wrap = useRef(null);
  const rail = 20;
  const sizeFor = width => clamp(Math.floor(width - rail - 22), 112, 224);
  const [padHost, pad] = useNexus(element => {
    const side = sizeFor(measure(wrap.current, 220));
    const widget = new Nexus.Position(element, {
      size: [side, side], mode: 'absolute', minX: -range, maxX: range, minY: -range, maxY: range,
      stepX: 0, stepY: 0, x: 0, y: 0,
    });
    widget.colorize('fill', 'transparent');
    widget.colorize('accent', PALETTE.cyan);
    return widget;
  }, { measureRef: wrap, onResize: (widget, width) => { const side = sizeFor(width); widget.resize(side, side); widget.render(); } });
  const [railHost, slider] = useNexus(element => {
    const widget = new Nexus.Slider(element, {
      size: [rail, sizeFor(measure(wrap.current, 220))], mode: 'absolute', min: -range, max: range, step: 0, value: 0,
    });
    widget.colorize('fill', PALETTE.track);
    widget.colorize('accent', PALETTE.purple);
    return widget;
  }, { measureRef: wrap, onResize: (widget, width) => { widget.resize(rail, sizeFor(width)); widget.render(); } });
  useEffect(() => {
    if (!pad.current) return;
    pad.current.x = finite(x) ? clamp(x, -range, range) : 0;
    pad.current.y = finite(y) ? clamp(y, -range, range) : 0;
  }, [x, y, range]);
  useEffect(() => {
    if (slider.current) slider.current.value = finite(z) ? clamp(z, -range, range) : 0;
  }, [z, range]);
  return <div className="nx-wrap accel-stage" ref={wrap}>
    <div className="nx nx-pad" ref={padHost} aria-hidden="true" />
    <div className="accel-rail">
      <div className="nx nx-rail" ref={railHost} aria-hidden="true" />
      <span>Z</span>
    </div>
  </div>;
}

/** NexusUI sequencer used as a 7-cell indicator row for the motor fault mask (bit 6 → bit 0). */
export function FaultBits({ mask, cell = 22 }) {
  const [host, matrix] = useNexus(element => {
    const widget = new Nexus.Sequencer(element, {
      size: [cell * 7, cell], mode: 'toggle', rows: 1, columns: 7, paddingRow: 5, paddingColumn: 5,
    });
    widget.colorize('fill', PALETTE.track);
    widget.colorize('accent', PALETTE.coral);
    return widget;
  });
  useEffect(() => {
    const valid = Number.isInteger(mask) && mask >= 0;
    matrix.current?.matrix.set.row(0, Array.from({ length: 7 }, (_, index) => valid && Boolean(mask & (1 << (6 - index)))));
  }, [mask]);
  return <div className="fault-bits">
    <div className="nx nx-bits" ref={host} aria-hidden="true" />
    <div className="bit-labels" aria-hidden="true">{[6, 5, 4, 3, 2, 1, 0].map(bit => <span key={bit}>{bit}</span>)}</div>
  </div>;
}
