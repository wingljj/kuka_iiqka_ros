// Wrench trend chart split into a pure data model (ring buffer + scaling,
// node-tested) and a thin canvas painter (exercised by the sim smoke).
// Decision 8: ~20 Hz throttled samples, capacity 600 = a 30 s window.

export class TraceBuffer {
  constructor(channels, capacity) {
    this._n = channels;
    this._cap = capacity;
    this._data = [];            // array of sample arrays, oldest first
  }
  get length() { return this._data.length; }
  push(sample) {
    this._data.push(sample.slice(0, this._n));
    if (this._data.length > this._cap) this._data.shift();
  }
  clear() { this._data = []; }
  channel(i) { return this._data.map((s) => s[i]); }
}

// Symmetric range snapped up to a multiple of 5 so axis labels stay
// clean; ±5 floor for flat/empty data.
export function yRange(samples) {
  let peak = 0;
  for (const s of samples) {
    for (const v of s) peak = Math.max(peak, Math.abs(v));
  }
  const m = Math.max(5, Math.ceil(peak / 5) * 5);
  return [-m, m];
}

// Maps a single channel into pixel coordinates (x spread over width,
// y inverted: +range at the top).
export function toPolyline(values, width, height, range) {
  const n = values.length;
  if (n === 0) return [];
  const [lo, hi] = range;
  return values.map((v, i) => [
    n === 1 ? 0 : (i * width) / (n - 1),
    height - ((v - lo) / (hi - lo)) * height,
  ]);
}

// Canvas painter: ctx is anything implementing the 2D context calls used
// here (the browser CanvasRenderingContext2D in production).
const COLORS = ['#e63946', '#2a9d8f', '#457b9d'];

export function drawChart(ctx, width, height, buffer, labels) {
  ctx.clearRect(0, 0, width, height);
  const chans = labels.length;
  const samples = [];
  for (let i = 0; i < buffer.length; i += 1) samples.push([]);
  const range = yRange(
    Array.from({ length: chans }, (_, c) => buffer.channel(c))
      .reduce((acc, ch) => {
        ch.forEach((v, i) => { (acc[i] = acc[i] || []).push(v); });
        return acc;
      }, samples));
  // zero line + range labels
  ctx.strokeStyle = '#888';
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();
  ctx.fillStyle = '#888';
  ctx.font = '10px sans-serif';
  ctx.fillText(String(range[1]), 2, 10);
  ctx.fillText(String(range[0]), 2, height - 2);
  // traces
  for (let c = 0; c < chans; c += 1) {
    const pts = toPolyline(buffer.channel(c), width, height, range);
    if (pts.length < 2) continue;
    ctx.strokeStyle = COLORS[c % COLORS.length];
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length; i += 1) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.stroke();
  }
}
