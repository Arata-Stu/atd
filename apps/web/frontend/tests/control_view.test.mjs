import assert from 'node:assert/strict';
import test from 'node:test';
import {controlConfigIssue, controlQuery, controlFrameIssue, controlOverlay, liveControlIssue} from '../src/control_view.ts';

const config = {min_lookahead_m:.35, lookahead_time_s:.4, max_lookahead_m:1.5, wheelbase_m:null};
const pose = {x:1,y:2,yaw:0};
const source = {map_id:'track', map_sha256:'v', hdmap_sha256:'h', session_id:'drive', epoch:1, line:'raceline'};
const control = {...source, controller:'pure_pursuit', stamp:100, remaining_s:.3, stale:false,
  config, target:[1.5,2], requested_lookahead_m:.5, actual_lookahead_m:.5,
  curvature_per_m:0, steering_rad:null, preview_limited:false, valid:true, vehicle_pose:pose};
const status = {connected:true, control, planning:{...source,stamp:100.02,vehicle_pose:{x:1.2,y:2,yaw:0}}};

test('the control chord starts at its own rear axle sample, even when the planner advanced', () => {
  assert.equal(liveControlIssue(status,false), '');
  assert.deepEqual(controlOverlay(control, pose), {origin:[1,2],target:[1.5,2]});
  for (const delta of [{epoch:2},{map_id:'next'},{map_sha256:'other'},{hdmap_sha256:'other'},
    {session_id:'next'},{line:'centerline'},{stale:true},{remaining_s:0},{vehicle_pose:null}]) {
    assert.ok(liveControlIssue({...status,control:{...control,...delta}},false));
  }
  assert.ok(liveControlIssue(status,true));
  assert.ok(liveControlIssue({...status,connected:false},false));
});

test('missing wheelbase leaves angle empty and preserves the target and curvature; rejection clears geometry', () => {
  assert.equal(controlFrameIssue(control), '');
  const measured = {...control, config:{...config,wheelbase_m:.25}, steering_rad:-.11, preview_limited:true};
  assert.equal(controlFrameIssue(measured), '');
  assert.ok(controlFrameIssue({...measured,steering_rad:null}));
  assert.ok(controlFrameIssue({...control,steering_rad:0}));
  for (const change of [{target:[1,NaN]}, {actual_lookahead_m:0}, {requested_lookahead_m:-1}, {curvature_per_m:null},
    {controller:'unknown'}, {valid:false,reason:'追跡途絶'}]) {
    assert.equal(controlOverlay({...control,...change},pose),undefined);
  }
  assert.equal(liveControlIssue({...status, control:{...control,valid:false,reason:'後退中'}},true),'後退中');
});

test('only explicit replay tuning reaches the frame query in SI units, initial or omitted wheelbase stays absent', () => {
  assert.equal(controlQuery(null),'');
  assert.equal(controlConfigIssue(config),'');
  assert.equal(controlQuery(config),'&min_lookahead_m=0.35&lookahead_time_s=0.4&max_lookahead_m=1.5');
  assert.match(controlQuery({...config,wheelbase_m:.257}), /wheelbase_m=0.257$/);
  for(const change of [{min_lookahead_m:0},{max_lookahead_m:.01},{lookahead_time_s:-1},{wheelbase_m:NaN},{wheelbase_m:0}])
    assert.ok(controlConfigIssue({...config,...change}));
});
