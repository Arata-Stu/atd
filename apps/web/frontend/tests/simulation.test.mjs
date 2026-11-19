import test from 'node:test';
import assert from 'node:assert/strict';
import { sampleAt } from '../src/simulation.ts';

test('cursor follows graph between reports, with continuous heading across ±π', () => {
  const samples = [
    {t_s:0.1, x_m:1, speed_mps:1, yaw_rad:179*Math.PI/180},
    {t_s:0.2, x_m:2, speed_mps:2, yaw_rad:-179*Math.PI/180},
  ];
  const mid = sampleAt(samples, 0.15);
  assert.ok(Math.abs(mid.x_m - 1.5) < 1e-9);
  assert.ok(Math.abs(mid.speed_mps - 1.5) < 1e-9);
  assert.ok(Math.abs(mid.yaw_rad - Math.PI) < 1e-9);
  assert.equal(sampleAt(samples, 0.21), undefined);
  assert.equal(sampleAt(samples, 0.09), undefined);
  assert.equal(sampleAt(samples, 0.20), samples[1]);
  assert.equal(sampleAt([], 0.1), undefined);
});
