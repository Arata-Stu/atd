import assert from 'node:assert/strict';
import test from 'node:test';
import { savedSpeeds, speedColor, speedScale, speedStrokes } from '../src/raceline_speed.ts';

const points = [[0, 0], [4, 0], [4, 2]];

test('old maps can use a stored profile in the same point order', () => {
  const profile = points.map(([x, y], i) => [i, x, y, 0, 0, i + 1, 0]);
  assert.deepEqual(savedSpeeds({ points, profile }), [1, 2, 3]);
  assert.equal(savedSpeeds({ points, profile: [...profile].reverse() }), undefined);
});

test('speed colors follow saved point order around the lap, including the closing segment', () => {
  const speeds = [2, 6, 4];
  const scale = speedScale(speeds, points.length);
  const [rise, fall, closing] = speedStrokes(points, speeds, scale, true);
  assert.deepEqual(scale, { min: 2, max: 6 });
  assert.deepEqual(rise.points, points.slice(0, 2));
  assert.equal(rise.stops[0].color, 'rgb(38, 94, 212)');
  assert.equal(rise.stops.at(-1).color, 'rgb(208, 48, 44)');
  assert.deepEqual(rise.stops.find(stop => stop.offset === .5), { offset: .5, color: speedColor(4, scale) });
  assert.equal(fall.stops[0].color, rise.stops.at(-1).color);
  assert.equal(fall.stops.at(-1).color, speedColor(4, scale));
  assert.deepEqual(closing.points, [points[2], points[0]]);
  assert.deepEqual(speedStrokes(points, speeds, scale, false), [rise, fall]);
});

test('constant profiles use one green stroke and do not add a zero-length seam', () => {
  const repeatedStart = [...points, points[0]];
  const speeds = repeatedStart.map(() => 1.5);
  const scale = speedScale(speeds, speeds.length);
  assert.deepEqual(scale, { min: 1.5, max: 1.5 });
  assert.deepEqual(speedStrokes(repeatedStart, speeds, scale, true), [
    { points: repeatedStart, color: 'rgb(27, 146, 71)' },
  ]);
});

test('a missing or invalid profile leaves the line available in its original style', () => {
  for (const speeds of [undefined, [], [1, 2], [NaN, 2, 3], [Infinity, 2, 3], [-1, 0, 2], [1, null, 3]]) {
    assert.equal(speedScale(speeds, 3), null);
  }
  assert.deepEqual(speedScale([0, 0, 0], 3), { min: 0, max: 0 });
});
