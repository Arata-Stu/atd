import assert from 'node:assert/strict';
import test from 'node:test';
import { bagFromHash, calibrationHash, receivedBag, referenceCandidates, validPoseFrame, wheelbaseMetres } from '../src/calibration.ts';

test('an explicit canonical TUM child frame is required', () => {
  for (const frame of ['', '/base_link', 'camera//left', 'camera/', 'unknown frame']) assert.equal(validPoseFrame(frame), false);
  for (const frame of ['base_link', 'camera/camera_link']) assert.equal(validPoseFrame(frame), true);
});

test('wheelbase is measured in millimetres and left out when unknown', () => {
  assert.equal(wheelbaseMetres('257'), 0.257);
  assert.equal(wheelbaseMetres(''), undefined);
  for (const input of ['-1', '0', 'x', 'Infinity']) assert.equal(wheelbaseMetres(input), null);
});

test('reference runs use another bag metadata hash and a populated response table', () => {
  const options = { metadata_sha256: 'current', runs: [
    { id: 'renamed', metadata_sha256: 'current', eligible_samples: 10 },
    { id: 'usable', metadata_sha256: 'other', eligible_samples: 10 },
    { id: 'empty', metadata_sha256: 'third', eligible_samples: 0 },
  ] };
  assert.deepEqual(referenceCandidates(options).map(run => run.id), ['usable']);
  assert.deepEqual(referenceCandidates({ ...options, metadata_sha256: null }), []);
});

test('only a verified finished transfer can preselect its exact bag after navigation and reload', () => {
  const bag = '走行 1/run+A';
  const pull = { action: 'pull', status: 'succeeded', result: { verified: true, record: bag } };
  assert.equal(bagFromHash(calibrationHash(receivedBag(pull))), bag);
  assert.equal(receivedBag({ ...pull, status: 'running' }), '');
  assert.equal(receivedBag({ ...pull, result: { record: bag } }), '');
  assert.equal(bagFromHash('#transfer?bag=stale'), '');
});
