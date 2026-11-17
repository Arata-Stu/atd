import assert from 'node:assert/strict';
import test from 'node:test';
import { captureCloudNotes, completionUpdate, editingIssue, generationIssue, restoredSettings } from '../src/map_workflow.ts';

const blank = { id: 'course', title: 'course', revision: 2, frame: 'map', closed: true, left: [], right: [], custom: [], lines: {}, source: {} };
const cloud = { points: [[0, 0, 0]], frame: 'map', source_count: 1, truncated: false };
test('NVIDIA pointcloud is required while a manual HDMap remains editable', () => {
  assert.equal(editingIssue(blank, null), '');
  assert.match(editingIssue({ ...blank, source: { kind: 'nvidia_offline', pointcloud_status: 'pending' } }, null), /点群/);
  assert.equal(editingIssue({ ...blank, source: { kind: 'nvidia_offline', pointcloud_status: 'ready' } }, cloud), '');
  assert.match(editingIssue({ ...blank, source: { kind: 'nvidia_offline', pointcloud_status: 'ready' } }, { ...cloud, frame: 'odom' }), /確認できません/);
});
test('Race cannot silently fall back without a saved region centerline', () => {
  assert.match(generationIssue(blank, 'raceline', null), /Centerline/);
  assert.match(generationIssue({ ...blank, closed: false }, 'centerline', null), /閉路/);
  assert.equal(generationIssue({ ...blank, closed: false }, 'custom', null), '');
  assert.match(generationIssue({ ...blank, lines: { centerline: { points: [[0, 0], [1, 1], [0, 1]], method: 'paired' } } }, 'raceline', null), /領域方式/);
  assert.equal(generationIssue({ ...blank, lines: { centerline: { points: [[0, 0], [1, 1], [0, 1]], method: 'region_equal_distance_v1' } } }, 'raceline', null), '');
});
test('Completion refreshes the selected map but preserves unsaved editing', () => {
  const job = { id: 'capture', status: 'succeeded', result: { map: 'course' } };
  assert.deepEqual(completionUpdate(job, 'running', undefined, 'course', false), { mapId: 'course', action: 'refresh' });
  assert.deepEqual(completionUpdate(job, 'running', undefined, 'course', true), { mapId: 'course', action: 'defer' });
  assert.equal(completionUpdate(job, undefined, undefined, 'course', false), null);
});
test('A fast job is opened only while its original selection is unchanged', () => {
  const job = { id: 'build', status: 'succeeded' };
  const tracked = { mapId: 'new-course', selectionId: 'course' };
  assert.deepEqual(completionUpdate(job, undefined, tracked, 'course', false), { mapId: 'new-course', action: 'open' });
  assert.deepEqual(completionUpdate(job, undefined, tracked, 'other', false), { mapId: 'new-course', action: 'catalog' });
  assert.deepEqual(completionUpdate(job, undefined, tracked, 'course', true), { mapId: 'new-course', action: 'catalog' });
});
test('Restored settings retain the two allowed optimizers and fixed region method', () => {
  assert.equal(restoredSettings({ ...blank, lines: { raceline: { points: [], settings: { optimizer: 'local', centerline_method: 'paired' } } } }, 'raceline').optimizer, 'mincurv');
  assert.equal(restoredSettings({ ...blank, lines: { raceline: { points: [], settings: { optimizer: 'mincurv_iqp' } } } }, 'raceline').optimizer, 'mincurv_iqp');
  assert.equal(restoredSettings(blank, 'centerline').centerline_method, 'region');
});
test('Visualization pointcloud reports its scope and only a valid out-of-tolerance tail age', () => {
  const notes = captureCloudNotes({ point_scope: 'last_nonempty_visualization_cloud_not_guaranteed_full_map', cloud_within_tail_tolerance: false, cloud_tail_age_s: 2.346 });
  assert.equal(notes.scope, '可視化点群（全地図の網羅性は未確認）');
  assert.equal(notes.tail, '入力末尾より2.35秒前の点群');
  assert.equal(captureCloudNotes({ cloud_within_tail_tolerance: true, cloud_tail_age_s: 2.346 }).tail, null);
  for (const value of [undefined, null, [], {}, { cloud_within_tail_tolerance: false, cloud_tail_age_s: '2.34' }, { cloud_within_tail_tolerance: false, cloud_tail_age_s: Infinity }, { cloud_within_tail_tolerance: false, cloud_tail_age_s: -1 }]) assert.equal(captureCloudNotes(value).tail, null);
});
