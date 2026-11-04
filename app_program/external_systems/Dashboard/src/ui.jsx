/** Shared, colour-neutral building blocks. Colour lives in the graphs and in small status dots only. */
export const STATUS_TEXT = Object.freeze({
  current: 'Received',
  snapshot: 'Snapshot',
  stale: 'Stale',
  missing: 'Awaiting',
  unavailable: 'Unavailable',
});

export function StatusChip({ status, children }) {
  return <span className={`chip status-${status}`}><i aria-hidden="true" />{children || STATUS_TEXT[status]}</span>;
}

export function Tile({ label, swatch, className = '', children, ...rest }) {
  return <div className={`tile ${className}`} {...rest}>
    <span className="tile-label">{swatch && <i className="swatch" style={{ background: swatch }} aria-hidden="true" />}{label}</span>
    <span className="tile-value">{children}</span>
  </div>;
}
