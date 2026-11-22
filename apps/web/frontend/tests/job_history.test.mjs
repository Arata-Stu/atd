import assert from 'node:assert/strict';
import test from 'node:test';
import { historyBoundary } from '../src/job_status.ts';

test('local start and inclusive end minute cover every start within the specified minute', () => {
  const start = new Date(2026, 10, 21, 13, 42).getTime() / 1000;
  assert.equal(historyBoundary('2026-11-21', '13:42', false), start);
  assert.equal(historyBoundary('2026-11-21', '13:42', true), start + 60);
});

test('whole local day and open ranges cover the intended endpoints', () => {
  assert.equal(historyBoundary('2026-03-08', '', false), new Date(2026, 2, 8).getTime() / 1000);
  assert.equal(historyBoundary('2026-03-08', '', true), new Date(2026, 2, 9).getTime() / 1000);
  assert.equal(historyBoundary('', '', false), -Infinity);
  assert.equal(historyBoundary('', '', true), Infinity);
});
