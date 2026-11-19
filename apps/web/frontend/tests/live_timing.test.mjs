import assert from 'node:assert/strict';
import test from 'node:test';
import { captureTiming } from '../src/live_timing.ts';

const sample = {now_s: 1_800_000_000.25, session_id: 'run', publishers: 1,
  current: {board: {stamp_s: 1_800_000_000.21875}}};

test('同一応答の Jetson 内経過と Notebook 往復を使い、受領時の経過範囲を外側に丸める', () => {
  assert.deepEqual(captureTiming(sample, 83.5), {roundTripMs: 84, boardAgeMs: [31, 115]});
  assert.deepEqual(captureTiming({...sample, now_s: sample.current.board.stamp_s}, 0),
    {roundTripMs: 0, boardAgeMs: [0, 0]});
});

test('接続失敗・未計測のときは往復も経過も欠測にする', () => {
  for (const elapsed of [null, NaN, Infinity, -1]) {
    assert.deepEqual(captureTiming(sample, elapsed), {roundTripMs: null, boardAgeMs: null});
  }
});

test('応答が成功しても基板の時刻を照合できないときは往復のみを表示する', () => {
  for (const observation of [null, {...sample, current: null}, {...sample, current: {board: null}},
    {...sample, now_s: sample.current.board.stamp_s - .1}, {...sample, now_s: NaN},
    {...sample, session_id: null}, {...sample, publishers: 2}]) {
    assert.deepEqual(captureTiming(observation, 54), {roundTripMs: 54, boardAgeMs: null});
  }
});
