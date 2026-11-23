import assert from 'node:assert/strict';
import test from 'node:test';
import { matchesTuningSession, tuningChanges, tuningLines, tuningNodes } from '../src/drive_tuning.ts';

const params = Object.entries({line: 'centerline', max_speed_mps: 1.5, min_lookahead_m: .3,
  lookahead_time_s: .4, max_lookahead_m: .7, speed_kp: 40, speed_ki: 8, speed_kd: 1})
  .map(([name, value]) => ({name, value, allowed: true, ...(name === 'max_speed_mps' ? {
    floating_point_range: [{from_value: .2, to_value: 1.5, step: 0}]} : {})}));
params.push({name: 'available_lines', value: ['centerline', 'custom'], read_only: true},
  {name: 'launch_speed_cap_mps', value: 1.5, read_only: true});

test('sim and vehicle tuning readback must match the requested node, environment and bridge', () => {
  for (const target of ['jetson', 'gazebo']) {
    const snap = {node: tuningNodes[target], identity: {instance_id: 'run', environment: target}};
    assert.ok(matchesTuningSession(snap, target, 'run'));
    assert.ok(!matchesTuningSession(snap, target, 'other'));
    assert.ok(!matchesTuningSession(snap, target, ''));
    assert.ok(!matchesTuningSession({...snap, node: '/atd_other'}, target, 'run'));
    assert.ok(!matchesTuningSession({...snap, identity: {instance_id: 'run'}}, target, 'run'));
    assert.ok(!matchesTuningSession({...snap, identity: null}, target, 'run'));
  }
});

test('only saved lines with verified speed profiles from the running controller are offered', () => {
  assert.deepEqual(tuningLines(params), ['centerline', 'custom']);
  assert.deepEqual(tuningLines([]), []);
  assert.ok(tuningChanges(params, {line: 'raceline'}).issue);
  assert.ok(tuningChanges(params, {available_lines: 'raceline'}).issue);
});

test('new min/max lookahead and selection are validated together; submit only changed values', () => {
  assert.deepEqual(tuningChanges(params, {line: 'custom', min_lookahead_m: '.8', max_lookahead_m: '1.0',
    max_speed_mps: '1.50', speed_kd: '0'}), {
    values: {line: 'custom', min_lookahead_m: .8, max_lookahead_m: 1, speed_kd: 0}, issue: ''});
  assert.ok(tuningChanges(params, {min_lookahead_m: '.8'}).issue);
  assert.ok(tuningChanges(params, {line: 'custom', max_lookahead_m: '.1'}).issue);
});

test('empty input, nonfinite, negative and out-of-range values cannot be applied', () => {
  for (const draft of [{speed_kp: ''}, {speed_ki: ' '}, {speed_kd: 'Infinity'}, {speed_kp: '-1'},
    {min_lookahead_m: '0'}, {max_speed_mps: '.1'}, {max_speed_mps: '2'}, {lookahead_time_s: 'NaN'}])
    assert.ok(tuningChanges(params, draft).issue, JSON.stringify(draft));
  assert.deepEqual(tuningChanges(params, {lookahead_time_s: '0', max_speed_mps: '0.2'}), {
    values: {lookahead_time_s: 0, max_speed_mps: .2}, issue: ''});
});
