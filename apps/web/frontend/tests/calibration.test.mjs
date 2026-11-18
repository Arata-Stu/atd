import assert from 'node:assert/strict';
import test from 'node:test';
import { bagFromHash, calibrationHash, displayedRun, receivedBag, referenceCandidates, requestedPoseFrame, topExclusions, validPoseFrame, wheelbaseMetres } from '../src/calibration.ts';

test('an explicitly entered legacy TUM child frame must be canonical', () => {
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

test('only a legacy source requests its frame; multiple maps require an explicit selection', () => {
  const legacy = {map_id: 'legacy', base_frame: 'base_link'};
  const known = {map_id: 'known', pose_frame: 'camera_link'};
  assert.equal(requestedPoseFrame([], undefined, 'stale_input'), undefined);
  assert.equal(requestedPoseFrame([known], known, 'stale_input'), undefined);
  assert.equal(requestedPoseFrame([legacy, known], undefined, 'base_link'), null);
  assert.equal(requestedPoseFrame([legacy], legacy, ''), null);
  assert.equal(requestedPoseFrame([legacy], legacy, 'camera_link'), 'camera_link');
});

test('explicit history can select other trips but default results require matching bag and digest', () => {
  const first = {id: 'foreign', bag: 'lab-A/lap', metadata_sha256: 'other'};
  const stale = {id: 'stale', bag: 'lab-B/lap', metadata_sha256: 'replaced'};
  const own = {id: 'own', bag: 'lab-B/lap', metadata_sha256: 'current'};
  const options = {runs: [first, stale, own], metadata_sha256: 'current'};
  assert.equal(displayedRun(options, own.bag, '', 'foreign'), own);
  assert.equal(displayedRun(options, 'lab-C/lap', '', 'foreign'), undefined);
  assert.equal(displayedRun(options, own.bag, first.id, ''), first);
});

test('zero-eligible diagnostic shows up to three actual positive reasons, including legacy empty results', () => {
  assert.deepEqual(topExclusions({}), []);
  assert.deepEqual(topExclusions({exclusion_reasons: {gap: 10, unknown: 3, tracking: 17, stale: 1, empty: 0}}),
    [['tracking', 17], ['gap', 10], ['unknown', 3]]);
});
