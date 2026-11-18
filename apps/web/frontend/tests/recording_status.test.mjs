import assert from 'node:assert/strict';
import test from 'node:test';
import { RECORDING_PHASES, recordingAvailability, recordingDuration, recordingPhaseText, recordingReasonText, recordingRequestText, recordingResultText, recordingSampleAge, recordingSize, recordingSourceText, recordingStatusText } from '../src/recording_status.ts';

const envelope = () => ({ connected: true, identity: { instance_id: 'qa-bridge' }, freshness_s: 2.5,
  status: { schema_version: 1, instance_id: 'qa-recorder', state: 'recording', stamp: 1234.5, age_s: .2, source_age_s: .1, stale: false } });

test('Every recorder phase is distinct from disconnected, missing and stale observations', () => {
  for (const state of RECORDING_PHASES) {
    const value = envelope(); value.status.state = state;
    assert.equal(recordingAvailability(value, 1000, 1200), 'fresh');
    assert.notEqual(recordingStatusText('fresh', state), '未確認');
  }
  assert.notEqual(recordingPhaseText('starting'), recordingPhaseText('recording'));
  assert.notEqual(recordingPhaseText('stopping'), recordingPhaseText('idle'));
  assert.notEqual(recordingPhaseText('error'), recordingStatusText('disconnected', 'error'));
  assert.equal(recordingAvailability(null, 0, 1200), 'loading');
  assert.equal(recordingAvailability({ ...envelope(), connected: false, status: null }, 1000, 1200), 'disconnected');
  assert.equal(recordingAvailability({ ...envelope(), status: null }, 1000, 1200), 'missing');
});
test('Browser elapsed time ages the original sample without rewriting its stamp or counting source age twice', () => {
  const value = envelope(), original = structuredClone(value);
  assert.equal(recordingSampleAge(value.status, 1000, 1300), .5);
  assert.equal(recordingAvailability(value, 1000, 3300), 'fresh');
  assert.equal(recordingAvailability(value, 1000, 3400), 'stale');
  assert.deepEqual(value, original);
  assert.equal(value.status.stamp, 1234.5);
});
test('A fresh HTTP response does not renew an old recorder publication', () => {
  const value = envelope(); value.status.age_s = 4;
  assert.equal(recordingAvailability(value, 1000, 1000), 'stale');
  value.status.age_s = 0; value.status.stale = true;
  assert.equal(recordingAvailability(value, 1000, 1000), 'stale');
  value.status.stale = false;
  assert.equal(recordingAvailability(value, 1000, 1000, true), 'unavailable');
});
test('Malformed clocks, ages, phases and freshness metadata cannot confirm recording', () => {
  for (const change of [{ age_s: -1 }, { age_s: NaN }, { age_s: null }, { stamp: Infinity }, { stamp: -1 }, { stamp: undefined }, { state: 'unknown' }, { state: 'toString' }, { stale: undefined }, { schema_version: 2 }, { instance_id: '' }]) {
    const value = envelope(); Object.assign(value.status, change);
    assert.equal(recordingAvailability(value, 1000, 1200), 'invalid');
  }
  for (const freshness_s of [undefined, 0, -1, Infinity, '2.5']) {
    const value = envelope(); value.freshness_s = freshness_s;
    assert.equal(recordingAvailability(value, 1000, 1200), 'invalid');
  }
  assert.equal(recordingSampleAge(envelope().status, 1000, 900), null);
  assert.equal(recordingAvailability(envelope(), 0, 1000), 'invalid');
  assert.equal(recordingPhaseText('toString'), '未確認');
});
test('Request acceptance, recorder state and a confirmed saved result are separate observations', () => {
  assert.equal(recordingRequestText(null), '操作要求はまだありません');
  assert.equal(recordingRequestText({ action: 'start', result: 'accepted' }), '開始要求 · 受付済み');
  assert.equal(recordingRequestText({ action: 'stop', result: 'ignored' }), '停止要求 · 実行せず');
  assert.equal(recordingRequestText({ action: 'start', result: 'rejected' }), '開始要求 · 拒否');
  assert.equal(recordingResultText(null), '終了結果はまだありません');
  assert.equal(recordingResultText({ success: true }), '保存確定');
  assert.equal(recordingResultText({ success: false }), '正常終了を確認できず');
  assert.equal(recordingResultText({ success: undefined }), '終了結果は未確認');
  assert.equal(recordingSourceText('propo'), 'プロポ');
  assert.equal(recordingSourceText('teleop'), 'Bluetooth コントローラ');
  assert.equal(recordingSourceText('unknown'), '入力元未確認');
  assert.match(recordingReasonText('start_accepted'), /開始要求を受け付け/);
  assert.match(recordingReasonText('recording_completed'), /保存内容を確認/);
  assert.match(recordingReasonText('missing_required_topics'), /必要なトピック/);
  assert.match(recordingReasonText('duplicate_request'), /重複/);
  assert.equal(recordingReasonText('unknown_internal_token'), '詳しい理由は受信情報を確認してください');
});
test('Duration and size distinguish known zero from missing or invalid observations', () => {
  assert.equal(recordingDuration(0), '00:00:00');
  assert.equal(recordingDuration(3661.9), '01:01:01');
  assert.equal(recordingDuration(360000), '100:00:00');
  assert.equal(recordingSize(0), '0 B');
  assert.equal(recordingSize(1024), '1.0 KiB');
  assert.equal(recordingSize(2 * 1024 ** 3), '2.0 GiB');
  for (const invalid of [null, undefined, -1, NaN, Infinity, '0']) {
    assert.equal(recordingDuration(invalid), '—');
    assert.equal(recordingSize(invalid), '—');
  }
  assert.equal(recordingSize(.5), '—');
});
