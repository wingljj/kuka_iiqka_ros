import { test } from 'node:test';
import assert from 'node:assert/strict';
import { TraceBuffer, yRange, toPolyline } from '../../www/js/wrench_chart.js';

test('TraceBuffer keeps the last capacity samples in order', () => {
  const b = new TraceBuffer(3, 4);           // 3 channels, capacity 4
  for (let i = 1; i <= 6; i += 1) b.push([i, i * 10, i * 100]);
  assert.equal(b.length, 4);
  assert.deepEqual(b.channel(0), [3, 4, 5, 6]);
  assert.deepEqual(b.channel(2), [300, 400, 500, 600]);
});

test('TraceBuffer clear resets to empty', () => {
  const b = new TraceBuffer(2, 8);
  b.push([1, 2]);
  b.clear();
  assert.equal(b.length, 0);
  assert.deepEqual(b.channel(0), []);
});

test('yRange expands to a symmetric multiple of 5', () => {
  assert.deepEqual(yRange([[0.5, -7.3, 2.0]]), [-10, 10]);
  assert.deepEqual(yRange([[1, 2], [-4.9, 0]]), [-5, 5]);
  assert.deepEqual(yRange([[12.1]]), [-15, 15]);
});

test('yRange degenerates to ±5 on empty/zero data', () => {
  assert.deepEqual(yRange([]), [-5, 5]);
  assert.deepEqual(yRange([[0, 0, 0]]), [-5, 5]);
});

test('toPolyline maps samples into pixel space', () => {
  // width 100, height 50, range [-10, 10]: value +10 -> y 0,
  // value -10 -> y 50, value 0 -> y 25. Four points span x 0..100.
  const pts = toPolyline([-10, 0, 10, 0], 100, 50, [-10, 10]);
  assert.deepEqual(pts, [[0, 50], [100 / 3, 25], [200 / 3, 0], [100, 25]]);
});
