import assert from 'node:assert/strict';
import test from 'node:test';
import { VEHICLE_DIMENSIONS, sameVehicleDraft, sameVehicleProfile, vehicleApplied, vehicleBodyPlan, vehicleCadReference, vehicleCameraAxes, vehicleDraft, vehicleDraftProfile, vehicleIdentityMatches, vehicleProfileIssue, vehicleSourceText, vehicleStatusFresh } from '../src/vehicle_profile.ts';

// Synthetic values exercise units and geometry; they are not measurements of this vehicle.
const profile = () => ({ version: 1,
  dimensions: Object.fromEntries(VEHICLE_DIMENSIONS.map(key => [key, { value_m: null, source: 'unmeasured' }])),
  camera: { enabled: false, translation_m: { x: null, y: null, z: null }, rotation_rpy_rad: { roll: null, pitch: null, yaw: null }, source: 'unmeasured' } });
const state = active => ({ connected: true, identity: { instance_id: 'bridge-a' }, ready: true,
  status: { profile_valid: true, profile_path: '/qa/vehicle.yaml', active_profile: active, tf_published: true,
    published_frames: ['rear_axle'], instance_id: 'vehicle-a', reason: 'ready', age_s: .1, stale: false } });

test('Unmeasured dimensions and camera fields remain blank and null through the UI conversion', () => {
  const value = profile(), draft = vehicleDraft(value);
  assert.equal(vehicleProfileIssue(value), '');
  assert.ok(VEHICLE_DIMENSIONS.every(key => draft.dimensions[key].value_mm === ''));
  assert.deepEqual(draft.camera.translation_mm, { x: '', y: '', z: '' });
  assert.deepEqual(vehicleDraftProfile(draft), { profile: value, issue: '' });
  assert.equal(vehicleBodyPlan(value), null);
  assert.equal(vehicleCameraAxes(value), null);
});
test('UI inputs use mm and degrees, while stored values use metres and radians', () => {
  const draft = vehicleDraft(profile());
  draft.dimensions.wheelbase = { value_mm: '420', source: 'manual' };
  draft.camera.translation_mm = { x: '123.4', y: '-20', z: '0' };
  draft.camera.rotation_rpy_deg = { roll: '0', pitch: '-90', yaw: '180' };
  draft.camera.enabled = true; draft.camera.source = 'manual';
  const result = vehicleDraftProfile(draft);
  assert.equal(result.issue, '');
  assert.equal(result.profile.dimensions.wheelbase.value_m, .42);
  assert.ok(Math.abs(result.profile.camera.translation_m.x - .1234) < 1e-12);
  assert.equal(result.profile.camera.translation_m.y, -.02);
  assert.equal(result.profile.camera.translation_m.z, 0);
  assert.equal(result.profile.camera.rotation_rpy_rad.pitch, -Math.PI / 2);
  assert.equal(result.profile.camera.rotation_rpy_rad.yaw, Math.PI);
  assert.equal(vehicleDraft(result.profile).camera.rotation_rpy_deg.yaw, '180');
});
test('Explicit zero is valid for a camera offset and rear overhang, while a blank is never zero', () => {
  const draft = vehicleDraft(profile());
  draft.dimensions.rear_overhang = { value_mm: '0', source: 'manual' };
  draft.camera.translation_mm.x = '0'; draft.camera.source = 'manual';
  let result = vehicleDraftProfile(draft);
  assert.equal(result.issue, '');
  assert.equal(result.profile.dimensions.rear_overhang.value_m, 0);
  assert.equal(result.profile.camera.translation_m.x, 0);
  assert.equal(result.profile.camera.translation_m.y, null);
  draft.dimensions.wheelbase = { value_mm: '0', source: 'manual' };
  result = vehicleDraftProfile(draft);
  assert.notEqual(result.issue, '');
  assert.equal(result.profile.dimensions.wheelbase.value_m, null);
});
test('Partial camera measurements can be saved while disabled, and enabling requires all six values', () => {
  const draft = vehicleDraft(profile()); draft.camera.translation_mm.x = '100'; draft.camera.source = 'manual';
  assert.equal(vehicleDraftProfile(draft).issue, '');
  draft.camera.enabled = true;
  assert.match(vehicleDraftProfile(draft).issue, /6項目/);
  draft.camera.translation_mm = { x: '100', y: '0', z: '50' };
  draft.camera.rotation_rpy_deg = { roll: '0', pitch: '0', yaw: '0' };
  assert.equal(vehicleDraftProfile(draft).issue, '');
});
test('Invalid and unfinished numeric inputs cannot be saved or become an invented origin', () => {
  for (const text of ['-', '.', 'Infinity', 'NaN', '0x10', '1,000', '20001', '-1']) {
    const draft = vehicleDraft(profile()); draft.dimensions.body_width = { value_mm: text, source: 'manual' };
    const result = vehicleDraftProfile(draft);
    assert.notEqual(result.issue, '', text);
    assert.equal(result.profile.dimensions.body_width.value_m, null);
  }
  const draft = vehicleDraft(profile()); draft.camera.rotation_rpy_deg.yaw = '181'; draft.camera.source = 'manual';
  assert.notEqual(vehicleDraftProfile(draft).issue, '');
  assert.equal(vehicleDraftProfile(draft).profile.camera.rotation_rpy_rad.yaw, null);
});
test('Source provenance follows null versus measured values and rejects incompatible profile shapes', () => {
  const unknown = profile(); unknown.dimensions.body_height.source = 'cad_candidate';
  assert.notEqual(vehicleProfileIssue(unknown), '');
  const known = profile(); known.dimensions.body_height = { value_m: .2, source: 'unmeasured' };
  assert.notEqual(vehicleProfileIssue(known), '');
  known.dimensions.body_height.source = 'cad_candidate';
  assert.equal(vehicleProfileIssue(known), '');
  assert.equal(vehicleDraft(known).dimensions.body_height.source, 'cad_candidate');
  assert.notEqual(vehicleProfileIssue({ ...known, unexpected: true }), '');
  assert.equal(vehicleSourceText('cad_candidate'), 'CAD 候補');
});
test('Body outline requires all three measurements and references the rear axle without using overall height', () => {
  const value = profile(); value.dimensions.body_length = { value_m: .6, source: 'manual' };
  value.dimensions.body_width = { value_m: .3, source: 'manual' };
  assert.equal(vehicleBodyPlan(value), null);
  value.dimensions.rear_overhang = { value_m: .1, source: 'manual' };
  value.dimensions.body_height = { value_m: .2, source: 'manual' };
  assert.deepEqual(vehicleBodyPlan(value), { rear: -.1, front: .5, halfWidth: .15 });
});
test('Fixed-axis RPY projects camera axes with the correct yaw and pitch signs', () => {
  const value = profile(); value.camera.rotation_rpy_rad = { roll: 0, pitch: 0, yaw: 0 };
  const identity = vehicleCameraAxes(value);
  assert.deepEqual(identity[1], [0, 1, 0]);
  value.camera.rotation_rpy_rad.yaw = Math.PI / 2;
  let axes = vehicleCameraAxes(value);
  assert.ok(Math.abs(axes[0][0]) < 1e-12 && Math.abs(axes[0][1] - 1) < 1e-12);
  value.camera.rotation_rpy_rad = { roll: 0, pitch: Math.PI / 2, yaw: 0 };
  axes = vehicleCameraAxes(value);
  assert.ok(Math.abs(axes[0][2] + 1) < 1e-12);
});
test('Dirty draft comparison preserves invalid text and applied profile comparison includes provenance', () => {
  const first = profile(), second = profile();
  assert.equal(sameVehicleProfile(first, second), true);
  const draft = vehicleDraft(first), persisted = vehicleDraft(first);
  assert.equal(sameVehicleDraft(draft, persisted), true);
  draft.camera.translation_mm.x = '-';
  assert.equal(sameVehicleDraft(draft, persisted), false);
  second.dimensions.rear_overhang = { value_m: 0, source: 'manual' };
  assert.equal(sameVehicleProfile(first, second), false);
});
test('Applied status requires fresh node state, matching path and profile, and confirmed TF publication', () => {
  const value = profile(), status = state(value);
  assert.equal(vehicleApplied(status, 1000, 1100, value, '/qa/vehicle.yaml'), true);
  status.status.tf_published = false;
  assert.equal(vehicleApplied(status, 1000, 1100, value, '/qa/vehicle.yaml'), false);
  status.status.tf_published = true;
  assert.equal(vehicleApplied(status, 1000, 1100, value, '/different.yaml'), false);
  assert.equal(vehicleApplied(status, 1000, 4100, value, '/qa/vehicle.yaml'), false);
  status.status.age_s = 4;
  assert.equal(vehicleStatusFresh(status, 1000, 1100), false);
  assert.equal(vehicleIdentityMatches({ instance_id: 'bridge-a' }, status.identity), true);
  assert.equal(vehicleIdentityMatches({ instance_id: 'bridge-b' }, status.identity), false);
});
test('CAD reference metadata is validated separately and never seeds vehicle dimensions or mount pose', () => {
  const reference = { source: 'cad_candidate', path: 'qa/frame.3mf', sha256: 'a'.repeat(64),
    axis_extent_m: { x: .1, y: .2, z: .01 }, caveat: 'Synthetic CAD reference, not measured.' };
  assert.equal(vehicleCadReference(reference), reference);
  assert.equal(vehicleCadReference({ ...reference, sha256: 'invalid' }), null);
  assert.equal(vehicleCadReference({ ...reference, axis_extent_m: { x: NaN, y: .2, z: .01 } }), null);
  assert.equal(vehicleCadReference(undefined), null);
  assert.deepEqual(vehicleDraftProfile(vehicleDraft(profile())).profile, profile());
});
