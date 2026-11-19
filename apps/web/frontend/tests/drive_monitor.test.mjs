import assert from 'node:assert/strict';
import test from 'node:test';
import {driveReadout} from '../src/drive_monitor.ts';

const board = {source:'cdc0', ch3:'propo', rx_mode_us:1100, active_path:1,
  control_source:'propo', pwm_enabled:true, servo_us:1250, esc_us:1550};
const joy = {steering:0, throttle:-.5};

test('physical CH3 selection and active route may differ while the Pico switches', () => {
  const row = driveReadout({board:{...board, ch3:'jetson', rx_mode_us:1900}, joy});
  assert.equal(row.ch3, 'Jetson 側'); assert.equal(row.route, 'プロポ');
  assert.equal(row.steering, 0); assert.equal(row.throttle, -.5);
  assert.equal(row.servo, 1250); assert.equal(row.esc, 1550);
  assert.equal(driveReadout({board:{...board, active_path:0, control_source:'standby'}}).route, '待機');
  assert.equal(driveReadout({board:{...board, active_path:2, control_source:'host'},joy}).route, 'HOST（操作元未確認）');
  assert.equal(driveReadout({board:{...board, ch3:'jetson', rx_mode_us:1900, active_path:2,
    control_source:'dualsense'},joy}).route, 'DualSense');
  assert.equal(driveReadout({board:{...board, ch3:'unknown', rx_mode_us:null}}).ch3, '未受信');
  assert.equal(driveReadout({board:{...board, ch3:'unknown', rx_mode_us:0}}).ch3, '未受信');
  assert.equal(driveReadout({board:{...board, ch3:'middle', rx_mode_us:1500}}).ch3, '中間位置');
});

test('CDC1 can report routing even when output feedback and CH3 pulse are missing', () => {
  const row = driveReadout({board:{...board, source:'cdc1', ch3_raw:172}, joy:null});
  assert.equal(row.route, 'プロポ'); assert.equal(row.ch3Raw, 172); assert.equal(row.ch3Pulse, null);
  assert.equal(row.ch3, '未確認'); assert.equal(row.servo, null); assert.equal(row.esc, null);
  assert.equal(row.steering, null); assert.equal(row.throttle, null);
  assert.equal(driveReadout({board:{...board, source:'cdc1', active_path:2, control_source:'dualsense'}, joy}).route,
    'HOST（操作元未確認）');
});

test('neutral, old or missing readings, and disabled PWM remain distinguishable', () => {
  const row = driveReadout({board:{...board, servo_us:999, esc_us:1500}, joy:{steering:-1, throttle:0}});
  assert.equal(row.servo, null); assert.equal(row.esc, 1500);
  assert.equal(row.steering, -1); assert.equal(row.throttle, 0);
  const fault = driveReadout({board:{...board, pwm_enabled:false, active_path:3, control_source:'fault'}, joy:null});
  assert.equal(fault.servo, null); assert.equal(fault.esc, null); assert.equal(fault.route, '基板異常');
  assert.equal(driveReadout({board:{...board, active_path:2, control_source:'fault', fault_bits:1}}).route, '基板異常');
  const noBoard = driveReadout({board:null, joy:{steering:1, throttle:NaN}});
  assert.equal(noBoard.route, '未確認'); assert.equal(noBoard.servo, null);
  assert.equal(noBoard.steering, 1); assert.equal(noBoard.throttle, null);
  assert.equal(driveReadout(undefined).route, '未確認');
});
