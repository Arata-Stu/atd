import assert from 'node:assert/strict';
import test from 'node:test';
import { sameTeleopProfile, teleopConnectionMatches, teleopFresh, teleopPreview, teleopProfileIssue, teleopReasonText, teleopRolesAssigned } from '../src/teleop_profile.ts';

const profile = () => ({ version: 1, axes: { steering: { axis: 'left_x', subtract_axis: '', scale: -1 }, throttle: { axis: 'r2', subtract_axis: 'l2', scale: .5 } }, buttons: { enable: 'r1', record_start: 'cross', record_stop: 'circle' } });
const sample = () => ({ axes: [.4, 0, 0, 0, .2, .8, 0, 0], buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], stale: false });
test('Preview applies named axes, subtraction and inversion without sending an operation', () => {
  const value = teleopPreview(profile(), sample(), true);
  assert.equal(value.steering, -.4);
  assert.ok(Math.abs(value.throttle - .3) < 1e-6);
  assert.equal(value.enable, true);
  const saturated = profile(); saturated.axes.steering = { axis: 'left_x', subtract_axis: 'left_y', scale: 1 };
  const input = sample(); input.axes[0] = 1; input.axes[1] = -1;
  assert.equal(teleopPreview(saturated, input, true).steering, 1);
});
test('Missing, disabled, stale and malformed input cannot produce a preview', () => {
  assert.equal(teleopPreview(profile(), sample(), false), null);
  for (const input of [null, { ...sample(), stale: true }, { ...sample(), stale: undefined }, { ...sample(), axes: [0] }, { ...sample(), buttons: [1] }, { ...sample(), axes: [NaN, 0, 0, 0, 0, 0, 0, 0] }, { ...sample(), axes: [2, 0, 0, 0, 0, 0, 0, 0] }]) assert.equal(teleopPreview(profile(), input, true), null);
});
test('Normalized trigger axes reject negative values while signed sticks remain valid', () => {
  for (const index of [4, 5]) {
    const input = sample(); input.axes[index] = -.01;
    assert.equal(teleopPreview(profile(), input, true), null);
  }
  const input = sample(); input.axes[0] = -.4;
  assert.equal(teleopPreview(profile(), input, true).steering, .4);
});
test('Unassigned controls remain unassigned and do not inherit a driving policy', () => {
  const value = profile(); value.axes.steering.axis = ''; value.axes.throttle.axis = ''; value.axes.throttle.subtract_axis = ''; value.buttons.enable = '';
  assert.equal(teleopRolesAssigned(value), false);
  assert.deepEqual(teleopPreview(value, sample(), true), { steering: null, throttle: null, enable: false });
  assert.equal(teleopProfileIssue(value), '');
});
test('Invalid gains, unavailable names and duplicate button roles are rejected', () => {
  for (const scale of [NaN, Infinity, -1.1, 1.1]) { const value = profile(); value.axes.steering.scale = scale; assert.match(teleopProfileIssue(value), /倍率/); }
  const invalid = profile(); invalid.axes.steering.axis = 'unknown'; assert.match(teleopProfileIssue(invalid), /標準軸/);
  const duplicate = profile(); duplicate.buttons.record_stop = duplicate.buttons.record_start; assert.match(teleopProfileIssue(duplicate), /異なるボタン/);
});
test('Saved and applied profiles compare by content rather than object key order', () => {
  const saved = profile(), active = { buttons: { record_stop: 'circle', record_start: 'cross', enable: 'r1' }, axes: { throttle: { scale: .5, subtract_axis: 'l2', axis: 'r2' }, steering: { scale: -1, subtract_axis: '', axis: 'left_x' } }, version: 1 };
  assert.equal(sameTeleopProfile(saved, active), true);
  active.axes.throttle.scale = .7;
  assert.equal(sameTeleopProfile(saved, active), false);
  assert.equal(sameTeleopProfile(saved, null), false);
});
test('Configuration is locked for a stale status or a different bridge instance', () => {
  const value = { connected: true, identity: { instance_id: 'old' }, status: { stale: false } };
  assert.equal(teleopFresh(value, 1000, 2000), true);
  assert.equal(teleopFresh(value, 1000, 4001), false);
  assert.equal(teleopFresh({ ...value, status: { stale: true } }, 1000, 2000), false);
  assert.equal(teleopFresh({ ...value, connected: false }, 1000, 2000), false);
  assert.equal(teleopConnectionMatches({ instance_id: 'old' }, value.identity), true);
  assert.equal(teleopConnectionMatches({ instance_id: 'new' }, value.identity), false);
  assert.equal(teleopConnectionMatches(null, value.identity), false);
});
test('Jetson reasons provide Japanese actions without exposing unknown status tokens', () => {
  assert.match(teleopReasonText('release_all_controls'), /すべて離して/);
  assert.match(teleopReasonText('profile_missing_or_invalid'), /保存.*再読込/);
  assert.match(teleopReasonText('ready'), /運転指令に変換/);
  for (const reason of ['future_internal_token', 'toString', '', null, {}]) assert.equal(teleopReasonText(reason), '状態を確認できません。Jetsonの接続と入力を確認してください。');
});
