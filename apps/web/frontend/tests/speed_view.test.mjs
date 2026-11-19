import assert from 'node:assert/strict';
import test from 'node:test';
import {liveSpeedHistory, liveSpeedIssue, plotSegments, sourceIntervals, speedPoint} from '../src/speed_view.ts';
import {throttleInScale} from '../src/propo_profile.ts';

const source = {map_id:'course',map_sha256:'v',hdmap_sha256:'h',session_id:'run',epoch:1,line:'raceline'};
const sample = {target_speed_mps:1.5,estimated_speed_mps:.8,calculated_raw:1321.5,propo_raw:1230,
  stamp:100,remaining_s:.5,stale:false,valid:true,...source};
const status = {connected:true,identity:{instance_id:'jetson'},planning:{...source,valid:true,stale:false},
  localization:{...source,state:'tracking',stale:false},speed_control:sample};

test('reject expired and unrelated observations before plotting, preserve manual input if FF is unavailable', () => {
  assert.equal(liveSpeedIssue(status,false),'');
  for (const change of [{stale:true},{epoch:2},{map_id:'b'},{stamp:NaN},{remaining_s:0},{line:'custom'}])
    assert.ok(liveSpeedIssue({...status,speed_control:{...sample,...change}},false));
  assert.ok(liveSpeedIssue(status,true)); assert.ok(liveSpeedIssue({...status,connected:false},false));
  const missing = {...sample,valid:false,reason:'FF 表が必要です'};
  assert.equal(liveSpeedIssue({...status,speed_control:missing},false),'');
  assert.deepEqual(speedPoint(100,missing),{time_s:100,target_speed_mps:1.5,estimated_speed_mps:.8,calculated_raw:null,
    propo_raw:1230,manual_source:'unknown',joy_throttle:null});
  assert.equal(speedPoint(0,{...sample,propo_raw:2048}).propo_raw,null);
  assert.equal(speedPoint(0,{...sample,propo_raw:992.1}).propo_raw,null);
  assert.equal(speedPoint(0,{...sample,propo_raw:0}).propo_raw,0);
  assert.equal(speedPoint(0,{...sample,calculated_raw:1321.5,propo_raw:null}).calculated_raw,1321.5);
});

test('Rev6 default comparison puts calculated CH2, Propo and Joy on the same two scales', () => {
  for (const [raw, normalized, pulse] of [
    [0, 1, 1000], [172, 1, 1000], [582, .5, 1250], [992, 0, 1500],
    [1402, -.5, 1750], [1811, -1, 2000], [2047, -1, 2000],
  ]) {
    for (const [metric, value] of [['calculated_raw',raw], ['propo_raw',raw], ['joy_throttle',normalized]]) {
      assert.equal(throttleInScale(metric, value, 'pulse'), pulse);
      assert.equal(throttleInScale(metric, value, 'normalized'), normalized);
    }
  }
  // JPB1 quantizes the magnitude to thousandths and truncates the step to whole µs.
  assert.equal(throttleInScale('joy_throttle', -.333, 'pulse'), 1666);
  assert.equal(throttleInScale('joy_throttle', .333, 'pulse'), 1334);
  assert.equal(throttleInScale('joy_throttle', -.001, 'pulse'), 1500);
  assert.equal(throttleInScale('joy_throttle', -.001, 'normalized'), -.001);
  assert.equal(throttleInScale('calculated_raw', 992.5, 'pulse'), 1501);
  assert.equal(throttleInScale('propo_raw', 992.5, 'pulse'), null);
  for (const value of [null, NaN, Infinity, -1.01, 2047.01]) {
    for (const metric of ['joy_throttle','propo_raw','calculated_raw'])
      assert.equal(throttleInScale(metric, value, 'pulse'), null);
  }
});

test('common scale plots preserve missing readings and keep the unconverted inputs', () => {
  const rows = [.5, null, -.5].map((joy_throttle, i) =>
    speedPoint(i / 10, {...sample, joy_throttle, manual_source:'host_joy'}));
  const convert = v => throttleInScale('joy_throttle', v, 'pulse');
  assert.deepEqual(plotSegments(rows, 'joy_throttle', 0, 1, convert), [[[0,1250]], [[.2,1750]]]);
  assert.deepEqual(rows.map(r=>r.joy_throttle), [.5,null,-.5]);
  assert.deepEqual(rows.map(r=>r.manual_source), Array(3).fill('host_joy'));
});

test('Joy is a separate signed observation and never infers the active path or a raw throttle', () => {
  for (const value of [-1,0,.4,1]) {
    const row=speedPoint(1,{...sample,valid:false,propo_raw:null,manual_source:'unknown',joy_throttle:value});
    assert.equal(row.joy_throttle,value); assert.equal(row.manual_source,'unknown');
    assert.equal(row.propo_raw,null); assert.equal(row.calculated_raw,null);
  }
  for (const value of [1.01,-1.01,NaN,Infinity,null,undefined])
    assert.equal(speedPoint(1,{...sample,joy_throttle:value}).joy_throttle,null);
  const both=speedPoint(1,{...sample,manual_source:'propo',joy_throttle:-.75,propo_raw:1249});
  assert.equal(both.propo_raw,1249); assert.equal(both.joy_throttle,-.75); assert.equal(both.manual_source,'propo');
});

test('source history conveys switching, missing samples, tracking boundaries and each input in its own units', () => {
  const at=(time_s,manual_source,joy_throttle,segment=1)=>({...speedPoint(time_s,{...sample,manual_source,joy_throttle}),segment});
  const history=[at(1,'propo',null),at(1.1,'propo',null),at(1.2,'host_joy',-.8),at(1.3,'host_joy',.2),
    at(1.4,'host',null),at(2.1,'host',null),at(2.2,'unknown',0,2)];
  const received=liveSpeedHistory({...sample,stamp:2.3,manual_source:'host_joy',joy_throttle:1,history});
  assert.deepEqual(received.map(p=>p.joy_throttle),[null,null,-.8,.2,null,null,0,1]);
  assert.deepEqual(sourceIntervals(history,1,2.2).map(r=>[r.source,r.from,r.to]),[
    ['propo',1,1.1],['host_joy',1.2,1.3],['host',1.4,1.4],['host',2.1,2.1],['unknown',2.2,2.2]]);
  assert.deepEqual(plotSegments(history,'joy_throttle',1.1,2.2),[[[1.2,-.8],[1.3,.2]],[[2.2,0]]]);
});

test('full Jetson observation window preserves a short manual pulse between UI polls', () => {
  const t = 106;
  const at=(time_s,raw)=>({...speedPoint(time_s,sample),propo_raw:raw});
  const pulse=[at(99,1000),at(105.91,1230),at(105.92,1500),at(105.93,1230),at(t,1230),at(t+.04,1555)];
  const history=liveSpeedHistory({...sample,stamp:t,history:pulse});
  assert.deepEqual(history.map(p=>p.propo_raw),[1230,1500,1230,1230]);
  assert.deepEqual(history.map(p=>p.time_s),[105.91,105.92,105.93,106]);
  assert.deepEqual(liveSpeedHistory(sample),[speedPoint(100,sample)]);
});

test('chart breaks gaps, missing values and tracking segment changes while respecting seek time', () => {
  const at=(t,segment=1,actual=1230)=>({...speedPoint(t,{...sample,propo_raw:actual}),segment});
  const rows=[at(0),at(.2),at(.3,1,null),at(.4),at(.45,2),at(1.2,2),at(2,2)];
  assert.deepEqual(plotSegments(rows,'propo_raw',0,1.2),[[[0,1230],[.2,1230]],[[.4,1230]],[[.45,1230]],[[1.2,1230]]]);
  assert.equal(plotSegments(rows,'calculated_raw',0,.45)[0].length,4);
  assert.deepEqual(plotSegments(rows,'propo_raw',3,8),[]);
});
