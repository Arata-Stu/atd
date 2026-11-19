import assert from 'node:assert/strict';
import test from 'node:test';
import {autoReadout, autoSegments} from '../src/auto_drive_view.ts';

function row(t, epoch = 1) {
  return {stamp_s:t, state:'driving', source:'autonomous',
    board:{stamp_s:t-.03, serial:'PICO', ch3:'jetson', rx_mode_us:2000,
      servo_us:1520, esc_us:1480, pwm_enabled:true},
    command:{stamp_s:t-.01, kind:'drive', steering:.06, throttle:.1, servo_us:1530, esc_us:1450},
    speed:{stamp_s:t-.02, tracking_session_id:'loc', epoch, validity_s:.2, target_mps:.5, estimated_mps:.25}};
}

const current = row(10);
const data = {publishers:1, session_id:'run', current, now_s:10};

test('各入力を元時刻で失効させ、CH3や遅れた基板情報から操作元を推測しない', () => {
  const initial = autoReadout({...data, current:{...current, source:'propo'}}, .01);
  assert.equal(initial.board.ch3, 'jetson');
  assert.equal(initial.source, 'propo');
  const oldCommand = {...current.command, stamp_s:9.87};
  const route = autoReadout({...data,current:{...current,command:oldCommand}},.04);
  assert.ok(route.board); assert.equal(route.command,null); assert.equal(route.source,'host');
  const mid = autoReadout(data, .13);
  assert.equal(mid.source, 'unknown'); assert.equal(mid.board,null);
  assert.equal(mid.command.esc_us,1450); assert.equal(mid.speed.estimated_mps,.25);
  assert.equal(autoReadout(data,.16).command,null);
  assert.equal(autoReadout(data,.19).speed,null);
  assert.ok(autoReadout(data,.19).state);
  assert.equal(autoReadout(data,.501).state,null);
  assert.equal(autoReadout(data,Infinity).refreshIn,null);
});

test('セッション未確認・複数配信者・未来の観測を現在値にしない', () => {
  for (const obs of [{...data,session_id:null},{...data,publishers:2},{...data,now_s:9}]) {
    const frame=autoReadout(obs,0);
    assert.equal(frame.source,'unknown'); assert.equal(frame.state,null); assert.equal(frame.board,null);
  }
});

test('送信と基板報告を元時刻で並べ、重複や解除・PWM 無効を線にしない', () => {
  const first=row(10), second=row(10.05), third=row(10.1), fourth=row(10.14);
  const history=[first,first,second,{...third,command:{...third.command,kind:'disable'},board:{...third.board,pwm_enabled:false}},fourth];
  assert.deepEqual(autoSegments(history,'command_esc',10.2),[
    [[first.command.stamp_s,1450],[second.command.stamp_s,1450]],[[fourth.command.stamp_s,1450]],
  ]);
  assert.deepEqual(autoSegments(history,'board_esc',10.2),[
    [[first.board.stamp_s,1480],[second.board.stamp_s,1480]],[[fourth.board.stamp_s,1480]],
  ]);
});

test('通信断、欠測、追跡世代、操作元、基板の交代を超えて線を結ばない', () => {
  const rows=[row(1),row(30),row(30.02),{...row(30.05),speed:null},row(30.08),row(30.1,2),row(31,2)];
  assert.deepEqual(autoSegments(rows,'target_mps',31).map(a=>a.length),[2,1,1,1]);
  const changed={...row(30.04),source:'propo'}, swapped=row(30.06); swapped.board.serial='PICO-B';
  assert.deepEqual(autoSegments([row(30),row(30.02),changed,swapped],'board_servo',30.1).map(a=>a.length),[2,1,1]);
});
