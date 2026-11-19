import test from 'node:test';
import assert from 'node:assert/strict';
import {draftStep, EXAMPLE_STEPS, stepCommand, trialPlan} from '../src/esc_trial.ts';

const rows = () => EXAMPLE_STEPS.map((s, i) => draftStep(i, s));

test('編集手順を順序通り送り、制動・後退の強度は正で送信し負の指令を表示する', () => {
  const {steps, issue} = trialPlan(rows());
  assert.equal(issue, '');
  assert.deepEqual(steps, EXAMPLE_STEPS);
  assert.deepEqual(steps.map(stepCommand), ['+0.150', '-0.150', '制動を維持', '0.000', '-0.120', '0.000']);
});

test('動作を変更した際に以前の強度をニュートラル／停止確認のリクエストへ持ち込まない', () => {
  const draft = rows().slice(0, 1);
  for (const action of ['neutral', 'wait_stopped']) {
    assert.deepEqual(trialPlan([{...draft[0], action, level: 'invalid'}]).steps, [{action, duration_s: 1}]);
  }
});

test('未入力、有限範囲外、空・過剰な手順を確定しない', () => {
  const [row] = rows();
  for (const field of [{duration: ''}, {duration: '.019'}, {duration: '301'}, {level: 'NaN'}, {level: ''}, {level: '0'}, {level: '.001'}, {level: '1.01'}]) {
    assert.ok(trialPlan([{...row, ...field}]).issue);
  }
  assert.ok(trialPlan([]).issue);
  assert.ok(trialPlan(Array(129).fill(row)).issue);
  assert.equal(trialPlan([{...row, level: '1', duration: '0.02'}]).issue, '');
});
