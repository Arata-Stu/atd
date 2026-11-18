import assert from 'node:assert/strict';
import test from 'node:test';
import { applicable, adoptionRequest } from '../src/calibration_candidates.ts';

const proposal = {run_id: 'run-1', proposal_sha256: 'hash',
  limits: {accel: {value: 1.2}, decel: {value: null}, curvature_limit: {value: 0.9}, lateral_accel: {value: 2}}};
const map = {id: 'course', revision: 4};

test('adoption binds the displayed report and map revision and selects available fields only', () => {
  assert.equal(adoptionRequest(proposal, undefined, ['accel']), null);
  assert.equal(adoptionRequest(proposal, map, []), null);
  assert.equal(adoptionRequest(proposal, map, ['decel']), null);
  for (const value of [null, 0, -1, Infinity, NaN]) assert.equal(applicable({value}), false);
  assert.deepEqual(adoptionRequest(proposal, map, ['accel', 'accel', 'decel']), {
    revision: 4, run_id: 'run-1', proposal_sha256: 'hash', fields: ['accel'],
  });
});

test('replacing a subset, a different run or a changed proposal remains an explicit adoption', () => {
  const adopted = {run_id: 'run-1', proposal_sha256: 'hash', limits: {curvature_limit: 0.9, accel: 1.2}};
  const saved = {...map, raceline_calibration: adopted};
  assert.equal(adoptionRequest(proposal, saved, ['accel', 'curvature_limit']), null);
  assert.deepEqual(adoptionRequest(proposal, saved, ['curvature_limit']).fields, ['curvature_limit']);
  assert.equal(adoptionRequest({...proposal, run_id: 'run-2'}, saved, ['accel', 'curvature_limit']).run_id, 'run-2');
  assert.equal(adoptionRequest({...proposal, proposal_sha256: 'new'}, saved, ['accel', 'curvature_limit']).proposal_sha256, 'new');
});
