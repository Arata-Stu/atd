import assert from 'node:assert/strict';
import test from 'node:test';
import { cloudFitCorners } from '../src/map_cloud_fit.ts';

const circuit = Array.from({ length: 400 }, (_, i) => {
  const angle = i * Math.PI / 200;
  return [12 + 10 * Math.cos(angle), -4 + 6 * Math.sin(angle), 0];
});
const distant = Array.from({ length: 60 }, (_, i) => [200 + i * 10, -400 - i, 0]);
const cloud = { frame: 'map', points: [...circuit, ...distant] };

test('cloud-only fit stays on the circuit with an asymmetric halo and preserves all stored points', () => {
  const original = structuredClone(cloud);
  const [[x0, y0], [x1, y1]] = cloudFitCorners(cloud);
  assert.ok(x0 >= 2 && x0 < 3 && x1 > 21 && x1 <= 22);
  assert.ok(y0 >= -10 && y0 < -9 && y1 > 1 && y1 <= 2);
  assert.deepEqual(cloud, original);
});

test('whole-course trajectory supplies its full extent including sparsely mapped ends', () => {
  assert.deepEqual(cloudFitCorners({ ...cloud, trajectory: [[-3, -12, 0], [24, 3, 0]] }),
    [[-3, -12], [24, 3]]);
});

test('a short final localization segment still shows the main cloud', () => {
  const corners = cloudFitCorners({ ...cloud, trajectory: [[12, -4, 0], [13, -3, 0]] });
  assert.deepEqual(corners, cloudFitCorners(cloud));
});

test('empty, legacy, and collinear captures have finite or empty fit bounds', () => {
  assert.deepEqual(cloudFitCorners({ frame: 'map', points: [], trajectory: [null, ['bad', 2], [0, 1, 2]] }), [[0, 1], [0, 1]]);
  assert.deepEqual(cloudFitCorners({ frame: 'map', points: [[NaN, 0, 0]] }), []);
  assert.deepEqual(cloudFitCorners({ frame: 'odom', points: circuit }), []);
  const line = Array.from({ length: 31 }, (_, i) => [i, 10, 0]);
  assert.deepEqual(cloudFitCorners({ frame: 'map', points: [...line, [-300, -900, 0]] }), [[0, 10], [30, 10]]);
});
