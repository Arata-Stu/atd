import assert from 'node:assert/strict';
import test from 'node:test';
import {captureRoute, livePlanIssue, savedLines, usablePlan} from '../src/planning_view.ts';

const id={map_id:'course', map_sha256:'v', hdmap_sha256:'h', session_id:'one', epoch:2};
const frame={...id,valid:true,stale:false,reason:'',line:'custom',progress_m:3,lateral_error_m:-.2,target_speed_mps:1.2,
  estimated_speed_mps:-.1,vehicle_pose:{x:1,y:2,yaw:0},trajectory:[[1,2.2],[3,2.2]],remaining_s:.35};
const live={connected:true,localization:{...id,state:'tracking',stale:false},planning:frame};

test('observed forward path and both speeds require the same map and localization epoch',()=>{
  assert.equal(livePlanIssue(live,false),'');
  for(const value of [{session_id:'other'},{epoch:3},{hdmap_sha256:'new'},{map_sha256:'new'},{map_id:'other'},
    {remaining_s:0},{stale:true},{line:'invalid'}]) {
    assert.ok(livePlanIssue({...live, planning:{...frame,...value}}, false));
  }
  assert.ok(livePlanIssue(live,true));
  assert.ok(livePlanIssue({...live,connected:false},false));
  assert.ok(livePlanIssue({...live,localization:{...live.localization,state:'lost'}},false));
});

test('録画用地図は確認済みの選択ラインを使い、期限切れの前方軌道は消す', () => {
  const current = {map_id: 'course', line: 'custom', speed: {tracking_session_id: 'one', epoch: 2}};
  const auto_drive = {publishers: 1, session_id: 'driver', current};
  const snapshot = {...live, auto_drive};
  assert.deepEqual(captureRoute(snapshot, false), {line: 'custom', plan: frame, issue: ''});
  assert.deepEqual(captureRoute(live, true), {line: 'custom', plan: null, issue: ''});
  assert.equal(captureRoute({...snapshot, planning: null}, true).line, 'custom');
  assert.equal(captureRoute({...live, planning: null}, false).line, null);
  for (const change of [{publishers: 2}, {current: {...current, map_id: 'other'}},
    {current: {...current, line: 'raceline'}}, {current: {...current, speed: {...current.speed, epoch: 3}}}]) {
    const result = captureRoute({...snapshot, auto_drive: {...auto_drive, ...change}}, false);
    assert.ok(result.issue);
    assert.equal(result.line, null);
    assert.equal(result.plan, null);
  }
});

test('fresh rejection explains why even with an immediate expiry; stale old reasons expire',()=>{
  const planning={...frame,valid:false,reason:'入力ラインに速度がありません',remaining_s:0};
  assert.equal(livePlanIssue({...live,planning},true),planning.reason);
  assert.notEqual(livePlanIssue({...live,planning:{...planning,stale:true}},true),planning.reason);
});

test('missing or invalid samples never supply a path; stopped and signed speed are valid',()=>{
  assert.ok(usablePlan({...frame, target_speed_mps:0, trajectory:[[5,0]]}));
  for(const value of [{vehicle_pose:null},{vehicle_pose:{x:1,y:1,yaw:NaN}},{target_speed_mps:null},
    {lateral_error_m:Infinity},{trajectory:[]},{trajectory:[[NaN,0]]}]) assert.equal(usablePlan({...frame,...value}),false);
});
test('custom selects saved sampled geometry, raw editor control points alone do not count',()=>{
  assert.deepEqual(savedLines({custom:[[2,0],[4,1]],lines:{centerline:{points:[[0,0],[1,0]]}}}),['centerline']);
  assert.deepEqual(savedLines({lines:{custom:{points:[[2,0],[2,1]]},raceline:{points:[[0,0],[0,1]]}}}),['raceline','custom']);
});
