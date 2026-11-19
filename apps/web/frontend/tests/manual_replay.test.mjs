import assert from 'node:assert/strict';
import test from 'node:test';
import {adjacentChanges, atTime, ch3State, escPwm, motionState, numericPath, servoPwm, stateBands,
  symmetricLimit, windowAt} from '../src/manual_replay.ts';

test('bag time covers consecutive windows, exact final boundaries and very short recordings', () => {
  assert.deepEqual(windowAt(59.99, 75.888, 10), {start_s:50,end_s:60});
  assert.deepEqual(windowAt(60, 75.888, 10), {start_s:60,end_s:70});
  assert.deepEqual(windowAt(60, 60, 10), {start_s:50,end_s:60});
  assert.deepEqual(windowAt(.002, .002, 1), {start_s:0,end_s:.002});
});

test('jump to confirmed transitions across windows including stop before reverse and exact cursor hits', () => {
  const changes = [4,35,58].map(t => ({time_s:t,from:'near_zero',to:'reverse',near_zero_start_s:t-1}));
  assert.equal(adjacentChanges(changes,0).next.time_s,4);
  assert.equal(adjacentChanges(changes,35).current.near_zero_start_s,34);
  assert.equal(adjacentChanges(changes,35).next.time_s,58);
  assert.equal(adjacentChanges(changes,35).previous.time_s,4);
  assert.equal(adjacentChanges(changes,36).previous.time_s,35);
  assert.equal(adjacentChanges(changes,60).next,undefined);
});

test('cursor reads each series independently, including exact switch, expires and bag endpoint', () => {
  const joy = [{start_s:1, end_s:1.2, steering:0}, {start_s:1.3,end_s:1.5,steering:-1}];
  const rc = {start_s:1.1,end_s:1.2,control_source:'propo'}, ds = {start_s:1.2,end_s:1.500000001,control_source:'dualsense'};
  assert.equal(atTime(joy,1)?.steering,0);
  assert.equal(atTime([rc,ds],1),null);
  assert.equal(atTime(joy,1.2),null);
  assert.equal(atTime([rc,ds],1.2),ds);
  assert.equal(atTime(joy,1.5),null);
  assert.equal(atTime([rc,ds],1.5),ds);
});

test('step lines preserve real zero, gaps, nulls, observed extent and instantaneous transitions', () => {
  const rows = [[0,1,-1],[1,2,0],[2,3,null],[3.5,4,1]].map(([start_s,end_s,value])=>({start_s,end_s,value}));
  assert.equal(numericPath(rows,r=>r.value,{start_s:.5,end_s:4.5},-1,1),'M0 96 H125 V50 H375 M750 4 H875');
  assert.equal(numericPath(rows,r=>r.value,{start_s:2,end_s:3},-1,1),'');
});

test('CDC1 and disabled PWM leave gaps, unknown CH3 remains distinct from no RC pulse', () => {
  const row = {source:'cdc0', pwm_enabled:true, ch3:'unknown',rx_mode_us:null,servo_us:1500,esc_us:1200};
  assert.equal(servoPwm(row),1500); assert.equal(escPwm(row),1200);
  assert.equal(ch3State(row),'absent');
  assert.equal(servoPwm({...row,pwm_enabled:false}),null);
  assert.equal(escPwm({...row,source:'cdc1'}),null);
  assert.equal(ch3State({...row,source:'cdc1'}),'unknown');
});

test('signed speed shows ahead, near standstill and reverse, preserving tracking loss', () => {
  const readings = [[0,.1,1], [.1,.2,.05], [.2,.3,-.05], [.6,.7,0], [.7,.8,-.25]]
    .map(([start_s,end_s,speed]) => ({start_s,end_s,speed}));
  assert.deepEqual(stateBands(readings, r => motionState(r.speed)).map(r => [r.start_s,r.end_s,r.state]), [
    [0,.1,'forward'], [.1,.3,'near_stop'], [.6,.7,'near_stop'], [.7,.8,'reverse'],
  ]);
  assert.equal(atTime(readings,.5), null);
  assert.equal(motionState(-.051),'reverse');
});

test('axes include both signs and sensor peaks; speed traces do not join separate tracking segments', () => {
  assert.equal(symmetricLimit([], v=>v,.1), .1);
  assert.equal(symmetricLimit([0,-3.5,1,NaN], v=>v,.5), 5);
  const rows = [[0,.5,-1,1],[.5,1,-.5,2]].map(([start_s,end_s,speed,segment])=>({start_s,end_s,speed,segment}));
  assert.equal(numericPath(rows,r=>r.speed,{start_s:0,end_s:1},-1,1,r=>r.segment), 'M0 96 H500 M500 73 H1000');
});
