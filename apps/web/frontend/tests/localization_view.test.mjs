import assert from 'node:assert/strict';
import test from 'node:test';
import { gazeboGroundTruth, gtFootprintBounds, mapIssue, streamId, trackingPose } from '../src/localization_view.ts';

const source = {map_id:'course', map_sha256:'v', hdmap_sha256:'h', session_id:'node-1', epoch:1, state:'tracking'};
const ref = {document:{id:'course', frame:'map'}, map_sha256:'v', hdmap_sha256:'h'};

test('position overlay requires the deployed visual map and exact HDMap/Raceline', () => {
  assert.equal(mapIssue(source, ref), '');
  for (const change of [{map_id:'other'}, {map_sha256:'different'}, {hdmap_sha256:'older'}])
    assert.notEqual(mapIssue({...source, ...change}, ref), '');
  assert.notEqual(mapIssue(source, {...ref, map_sha256:null}), '');
});

test('車両矢印・軌跡に使う位置は追跡状態と同一時刻で有限の map 座標に限る', () => {
  const pose = {x: 1, y: 2, yaw: -.5, frame: 'map', stamp: 5, stale: false};
  const live = {connected: true, localization: {...source, stamp: 5}, pose};
  assert.equal(trackingPose(live), pose);
  for (const change of [{stamp: 4.99}, {yaw: NaN}, {x: Infinity}, {frame: 'odom'}, {stale: true}])
    assert.equal(trackingPose({...live, pose: {...pose, ...change}}), null);
  assert.equal(trackingPose({...live, localization: {...live.localization, state: 'lost'}}), null);
  assert.equal(trackingPose({...live, connected: false}), null);
});
test('tracking histories break on disconnect, loss, stale data, new map or localization epoch', () => {
  const live = {connected:true, localization:source, identity:{instance_id:'bridge'}};
  const id = streamId(live);
  assert.ok(id);
  for (const change of [{state:'lost'}, {state:'waiting_hint'}, {stale:true}])
    assert.equal(streamId({...live, localization:{...source,...change}}), '');
  assert.equal(streamId({...live, connected:false}), '');
  for (const change of [{epoch:2}, {session_id:'node-2'}, {hdmap_sha256:'changed'}, {map_sha256:'changed'}])
    assert.notEqual(streamId({...live, localization:{...source,...change}}), id);
});

test('GT は地図照合後に独立して表示し、新鮮な追跡姿勢がない場合も読める', () => {
  const footprint = {vehicle_length: .35, vehicle_width: .19, rear_axle_to_rear: .1065, margin: .02};
  const gt = {...source, x: 2, y: -3, yaw: 1.2, frame: 'map', stamp_s: 12, remaining_s: .4, footprint};
  const live = {connected: true, identity: {environment: 'gazebo'}, gazebo: {ground_truth: gt}};
  assert.equal(gazeboGroundTruth(live, ref, false), gt);
  assert.equal(gazeboGroundTruth({...live, localization: {...source, state: 'lost', stale: true}}, ref, false), gt);
  for (const overlay of [null, {...gt, frame: 'world'}, {...gt, map_sha256: 'old'}, {...gt, hdmap_sha256: 'updated'},
    {...gt, yaw: Infinity}, {...gt, remaining_s: 0}, {...gt, stamp_s: 0}, {...gt, footprint: {...footprint, vehicle_width: -1}}])
    assert.equal(gazeboGroundTruth({...live, gazebo: {ground_truth: overlay}}, ref, false), null);
  for (const change of [{identity: {environment: 'jetson'}}, {connected: false}, {localization: {...source, map_id: 'other'}}])
    assert.equal(gazeboGroundTruth({...live, ...change}, ref, false), null);
  assert.equal(gazeboGroundTruth(live, null, false), null);
  assert.equal(gazeboGroundTruth(live, ref, true), null);
});

test('GT の矩形は後輪軸から左右・前後へ接触判定と同じ余裕を取る', () => {
  const bounds = gtFootprintBounds({vehicle_length: .35, vehicle_width: .19, rear_axle_to_rear: .1065, margin: .02});
  for (const [value, expected] of [[bounds.rear, -.1265], [bounds.front, .2635], [bounds.halfWidth, .115]])
    assert.ok(Math.abs(value - expected) < 1e-9);
});
