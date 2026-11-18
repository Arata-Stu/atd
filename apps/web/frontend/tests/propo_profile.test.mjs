import assert from 'node:assert/strict';
import test from 'node:test';
import { PROPO_CHANNELS, picoConnectionText, picoFaultText, picoPathText, propoAvailableChannels, propoConnectionMatches, propoDisplayCount, propoInputFresh, propoPosition, propoProfileIssue, propoPulseUs, propoRcValid, propoReasonText, propoResponseFresh, propoRoleInputCurrent, propoSourceMatches, propoTelemetryFresh, propoVisibleChannelCount, samePropoProfile } from '../src/propo_profile.ts';

const binding = (channel = 0) => ({ channel, low_threshold: 700, high_threshold: 1300, inverted: false });
const profile = () => ({ version: 1, debounce_ms: 100, roles: { recording: binding(), auxiliary_1: binding(), auxiliary_2: binding() } });
const status = () => ({ connected: true, identity: { instance_id: 'bridge-a' }, input_current: true,
  board: { connected: true, streaming: true, instance_id: 'pico-a', serial: 'serial-a', age_s: 0, stale: false, telemetry_timeout_s: .3,
    telemetry: { instance_id: 'pico-a', channels: Array(16).fill(992), age_s: .1, stale: false, rc_valid: true, sbus_fresh: true, sbus_flags: 0 } },
  input: { configuring: false, profile_valid: true, profile_path: '/profile.yaml', profile: profile(), ready: true,
    reason: 'ready', input_instance_id: 'pico-a', serial: 'serial-a', age_s: 0, input_age_s: .1, stale: false } });

test('Propo roles default to unassigned and cannot overwrite fixed driving channels', () => {
  assert.equal(propoProfileIssue(profile()), '');
  for (const channel of [1, 2, 3, 17, -1, 4.5, NaN]) {
    const value = profile(); value.roles.recording.channel = channel;
    assert.match(propoProfileIssue(value), /CH4 ～ CH16/);
  }
});
test('RC roles can use CH4 through CH16 while options follow server channel metadata', () => {
  assert.deepEqual(PROPO_CHANNELS, Array.from({ length: 13 }, (_, index) => index + 4));
  for (const channel of PROPO_CHANNELS) {
    const value = profile(); value.roles.recording.channel = channel;
    assert.equal(propoProfileIssue(value), '');
  }
  const available = propoAvailableChannels([3, 10, 4, 16, 10, 17, '5', NaN]);
  assert.deepEqual(available, [4, 10, 16]);
  for (const malformed of [null, undefined, '4,5,6', { channel: 4 }]) assert.deepEqual(propoAvailableChannels(malformed), []);
  const value = profile(); value.roles.recording.channel = 10;
  assert.equal(propoProfileIssue(value, available), '');
  assert.match(propoProfileIssue(value, [4, 5, 6]), /接続先で利用できる/);
});
test('PWM equivalent uses the reference firmware endpoints, center and integer rounding', () => {
  for (const [raw, pulse] of [[0, 1000], [171, 1000], [172, 1000], [173, 1001], [582, 1250],
      [991, 1499], [992, 1500], [993, 1501], [1401, 1750], [1810, 1999], [1811, 2000], [2047, 2000]]) {
    assert.equal(propoPulseUs(raw), pulse, `raw ${raw}`);
  }
  for (const raw of [null, undefined, '', '992', -1, 2048, .5, NaN, Infinity, -Infinity]) assert.equal(propoPulseUs(raw), null);
  let previous = 1000;
  for (let raw = 0; raw <= 2047; raw++) {
    const pulse = propoPulseUs(raw);
    assert.ok(Number.isInteger(pulse) && pulse >= previous && pulse >= 1000 && pulse <= 2000);
    previous = pulse;
  }
});
test('Display count defaults to six and expands to include draft, saved and applied assignments', () => {
  for (const [stored, expected] of [[null, 6], [undefined, 6], ['6', 6], ['10', 10], ['16', 16], [10, 10], [16, 16], ['12', 6], [Infinity, 6]]) {
    assert.equal(propoDisplayCount(stored), expected);
  }
  const draft = profile(), saved = profile(), active = profile();
  assert.equal(propoVisibleChannelCount(6, draft, saved, active), 6);
  draft.roles.recording.channel = 10;
  assert.equal(propoVisibleChannelCount(6, draft, saved, active), 10);
  draft.roles.recording.channel = 4; saved.roles.auxiliary_1.channel = 11;
  assert.equal(propoVisibleChannelCount(6, draft, saved, active), 16);
  saved.roles.auxiliary_1.channel = 0; active.roles.auxiliary_2.channel = 16;
  assert.equal(propoVisibleChannelCount(10, draft, saved, active), 16);
  assert.equal(propoVisibleChannelCount(16, draft), 16);
  const before = structuredClone(draft);
  propoVisibleChannelCount(6, draft);
  assert.deepEqual(draft, before);
  assert.equal(samePropoProfile(draft, before), true);
});
test('Roles can swap CH4 to CH6, but one channel cannot have multiple Jetson roles', () => {
  const value = profile(); value.roles.recording.channel = 4; value.roles.auxiliary_1.channel = 6;
  assert.equal(propoProfileIssue(value), '');
  value.roles.recording.channel = 6;
  assert.match(propoProfileIssue(value), /同じチャンネル/);
  value.roles.auxiliary_1.channel = 4;
  assert.equal(propoProfileIssue(value), '');
});
test('Thresholds and debounce reject non-finite, fractional, out of range and invalid shapes', () => {
  for (const debounce_ms of [-1, 1001, .5, Infinity, NaN]) {
    const value = profile(); value.debounce_ms = debounce_ms;
    assert.match(propoProfileIssue(value), /確認時間/);
  }
  for (const [low_threshold, high_threshold] of [[-1, 1300], [700, 2048], [1300, 700], [700, 700], [NaN, 1300], [700.5, 1300]]) {
    const value = profile(); Object.assign(value.roles.recording, { low_threshold, high_threshold });
    assert.match(propoProfileIssue(value), /閾値/);
  }
  const invalid = profile(); invalid.roles.recording.inverted = 1;
  assert.match(propoProfileIssue(invalid), /位置反転/);
  assert.match(propoProfileIssue({ ...profile(), roles: { unknown: binding() } }), /形式/);
  assert.match(propoProfileIssue({ ...profile(), extra: true }), /形式/);
  const extraBinding = profile(); extraBinding.roles.recording.extra = true;
  assert.match(propoProfileIssue(extraBinding), /形式/);
});
test('Position preview uses inclusive thresholds and reverses logical sides without synthesizing actions', () => {
  const value = binding(4);
  assert.equal(propoPosition(value, 700), 'low');
  assert.equal(propoPosition(value, 1000), 'middle');
  assert.equal(propoPosition(value, 1300), 'high');
  value.inverted = true;
  assert.equal(propoPosition(value, 700), 'high');
  assert.equal(propoPosition(value, 1300), 'low');
  assert.equal(propoPosition(value, 1000), 'middle');
  for (const raw of [null, -1, 2048, NaN]) assert.equal(propoPosition(value, raw), null);
  assert.equal(propoPosition(binding(), null), 'unassigned');
});
test('Profile comparison detects assignments, thresholds, inversion and debounce changes', () => {
  const first = profile(), second = profile();
  assert.equal(samePropoProfile(first, second), true);
  for (const patch of [{ channel: 4 }, { low_threshold: 600 }, { high_threshold: 1500 }, { inverted: true }]) {
    const changed = profile(); Object.assign(changed.roles.recording, patch);
    assert.equal(samePropoProfile(first, changed), false);
  }
  second.debounce_ms = 120;
  assert.equal(samePropoProfile(first, second), false);
  assert.equal(samePropoProfile(first, null), false);
});
test('USB, bridge and source sample freshness are separate and stale source age is not renewed', () => {
  const value = status();
  assert.equal(propoResponseFresh(value, 1000, 1100), true);
  assert.equal(propoInputFresh(value, 1000, 1100), true);
  assert.equal(propoTelemetryFresh(value, 1000, 1100), true);
  value.board.telemetry.age_s = 4;
  assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
  value.board.telemetry.age_s = .1;
  assert.equal(propoTelemetryFresh(value, 1000, 4100), false);
  value.board.streaming = false;
  assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
  value.board.streaming = true; value.connected = false;
  assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
});
test('Source timeout expires displayed input without expiring the periodic configuration status', () => {
  const value = status();
  assert.equal(propoRoleInputCurrent(value, 1000, 1100), true);
  assert.equal(propoTelemetryFresh(value, 1000, 1250), false);
  assert.equal(propoInputFresh(value, 1000, 1250), true);
  assert.equal(propoRoleInputCurrent(value, 1000, 1250), false);
  value.input.input_age_s = .4;
  assert.equal(propoRoleInputCurrent(value, 1000, 1000), false);
  for (const timeout of [0, Infinity, -1, 6, NaN]) {
    value.board.telemetry_timeout_s = timeout;
    assert.equal(propoTelemetryFresh(value, 1000, 1000), false);
  }
});
test('Loss and failsafe invalidate RC actions while raw telemetry remains inspectable', () => {
  for (const flags of [4, 8, 12]) {
    const value = status(); value.board.telemetry.sbus_flags = flags;
    assert.equal(propoTelemetryFresh(value, 1000, 1100), true);
    assert.equal(propoRcValid(value, 1000, 1100), false);
  }
  const value = status();
  assert.equal(propoRcValid(value, 1000, 1100), true);
  value.board.telemetry.rc_valid = false;
  assert.equal(propoRcValid(value, 1000, 1100), false);
  value.board.telemetry.rc_valid = true; value.board.telemetry.sbus_fresh = false;
  assert.equal(propoRcValid(value, 1000, 1100), false);
  value.board.telemetry.sbus_fresh = true;
  for (const flags of [-16, 256, NaN]) {
    value.board.telemetry.sbus_flags = flags;
    assert.equal(propoRcValid(value, 1000, 1100), false);
  }
});
test('Changing bridge or USB source prevents using an old target or old interpreted role data', () => {
  const value = status();
  assert.equal(propoConnectionMatches({ instance_id: 'bridge-a' }, value.identity), true);
  assert.equal(propoConnectionMatches({ instance_id: 'old-bridge' }, value.identity), false);
  assert.equal(propoSourceMatches(value), true);
  value.input.input_instance_id = 'old-pico';
  assert.equal(propoSourceMatches(value), false);
  value.input.input_instance_id = 'pico-a'; value.input.serial = 'other-device';
  assert.equal(propoSourceMatches(value), false);
  value.input.serial = 'serial-a'; value.board.telemetry.instance_id = 'old-pico';
  assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
  value.board.telemetry.instance_id = 'pico-a'; value.board.telemetry.serial = 'other-device';
  assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
});
test('Malformed channel arrays never become current live input', () => {
  for (const channels of [null, undefined, [1, 2], Array(16).fill(-1), Array(16).fill(2048), Array(16).fill(.5), Array(16).fill(NaN)]) {
    const value = status(); value.board.telemetry.channels = channels;
    assert.equal(propoTelemetryFresh(value, 1000, 1100), false);
  }
});
test('Pico and role status explain known states without displaying unknown internal tokens', () => {
  assert.match(picoConnectionText('multiple_devices'), /候補が複数/);
  assert.match(picoConnectionText('serial_error'), /他のプロセス/);
  assert.match(picoConnectionText('awaiting_stream'), /USB は接続済み/);
  assert.match(picoConnectionText('waiting_for_device'), /USB 接続を待って/);
  for (const reason of ['receiving', 'unknown_internal_token', 'toString', null]) assert.equal(picoConnectionText(reason), '');
  assert.equal(picoPathText(1), 'プロポ操作');
  assert.equal(picoPathText(2), 'Jetson 操作');
  assert.equal(picoPathText(99), '未確認');
  assert.equal(picoFaultText(0), 'なし');
  assert.equal(picoFaultText(5), '電源・受信喪失');
  assert.equal(picoFaultText(256), '未定義の状態');
  assert.equal(picoFaultText(undefined), '未確認');
  assert.match(propoReasonText('baseline_pending'), /再接続だけでは記録を開始しません/);
  assert.match(propoReasonText('configuring'), /操作要求を止めています/);
  for (const reason of ['unknown_internal_token', 'toString', null]) assert.equal(propoReasonText(reason), '入力状態を確認できません。接続と入力の起動を確認してください。');
});
