import assert from 'node:assert/strict';
import test from 'node:test';
import { gazeboHoldStatus } from '../src/gazebo_hold.ts';

function snapshot(state, vehicle_hold = null) {
  return {pause_supported: true, current: {state}, vehicle_hold};
}
const stationary = {held: true, speed_mps: 0, stamp_s: 3, remaining_s: .25};

test('pause is available before departure, while driving, and during an observation gap', () => {
  for (const state of ['waiting', 'driving', 'waiting_observation']) {
    const view = gazeboHoldStatus(snapshot(state), 100, 110);
    assert.ok(view.canPause);
    assert.ok(!view.canResume);
  }
});

test('resume requires both the controller pause and fresh stationary feedback from the plant', () => {
  const confirmed = gazeboHoldStatus(snapshot('paused', stationary), 100, 349);
  assert.ok(confirmed.canResume);
  assert.match(confirmed.text, /一時停止中/);
  assert.ok(!confirmed.canPause);
  for (const hold of [null, {...stationary, held: false}, {...stationary, speed_mps: .2}, {...stationary, remaining_s: 0}]) {
    const view = gazeboHoldStatus(snapshot('paused', hold), 100, 100);
    assert.ok(!view.canResume);
    assert.match(view.text, /確認待ち/);
  }
  assert.ok(!gazeboHoldStatus(snapshot('paused', stationary), 100, 350).canResume);
  const waiting = gazeboHoldStatus(snapshot('waiting_observation', stationary), 100, 101);
  assert.ok(!waiting.canResume && waiting.canPause);
  assert.match(waiting.text, /走行指令を待機/);
});

test('ended sessions, missing controller samples and older bridges cannot accept commands', () => {
  for (const data of [undefined, {pause_supported: true, current: null},
    snapshot('stopped', stationary), {...snapshot('driving'), pause_supported: false}, {current: {state: 'paused'}, vehicle_hold: stationary}]) {
    const view = gazeboHoldStatus(data, 100, 105);
    assert.ok(!view.canPause && !view.canResume);
  }
});
