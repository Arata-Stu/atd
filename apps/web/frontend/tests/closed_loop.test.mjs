import test from 'node:test';
import assert from 'node:assert/strict';
import { around, chartsFor, controlLineAt, controlObservations, referenceAt, timeBounds, valueAt, windowRows } from '../src/closed_loop.ts';

test('選択時刻の参照ラインを保存した地図から描き、直前の操舵に使ったラインと区別する',()=>{
  const data = {run:{line:'raceline'},reference:[[0,0],[1,0]],reference_speeds_mps:[1,1]};
  assert.equal(referenceAt(data,9).name,'raceline');
  data.line_changes = [{t:1,line:'custom'},{t:2,line:'centerline'},{t:2,line:'raceline'}];
  data.additional_references = {custom:{points:[[0,1],[1,1]],speeds_mps:[]}};
  assert.deepEqual(referenceAt(data,.99).points,data.reference);
  assert.deepEqual(referenceAt(data,1).points,data.additional_references.custom.points);
  assert.deepEqual(referenceAt(data,2).points,data.reference);
  const samples = [{t:.99,control_image_s:.98,control_line:'raceline'},
    {t:1.1,control_image_s:1.09,control_line:'custom'}];
  assert.equal(controlLineAt(samples,1),'raceline');
  assert.equal(controlLineAt(samples,1.1),'custom');
  assert.equal(controlLineAt(samples,2),null);
  assert.notEqual(controlObservations(samples)[0].segment,controlObservations(samples)[1].segment);
});

test('短い区間を補間し、終了後・欠測・追跡区間の切り替わり・方位角の折返しを結ばない',()=>{
  const rows = [{t:10, v:1, segment:1},{t:10.125,v:3,segment:1},{t:10.25,v:4,segment:2},
    {t:11,v:7,segment:2},{t:11.125,v:null,segment:2}];
  const get = p => p.v;
  assert.equal(valueAt(rows,10.0625,get),2);
  assert.equal(valueAt(rows,10.0625,get,'step'),1);
  assert.equal(valueAt(rows,10.25,get),4);
  for (const t of [0,10.2,10.375,11.0625,12]) assert.equal(valueAt(rows,t,get),null);
  assert.equal(valueAt([{t:0,v:-179},{t:.1,v:179}],.05,get,'degrees'),null);
});

test('一周未満の境界接触も独立した時刻として保持し、前後3秒を保存範囲に収める',()=>{
  const data = {samples:[{t:9},{t:11}],estimates:[{t:10}],vslam:[],contact_s:10.875};
  assert.deepEqual(timeBounds(data),[9,11]);
  assert.deepEqual(around(data.contact_s,[0,11]),[7.875,11]);
  assert.deepEqual(around(1,[0,2]),[0,2]);
  assert.deepEqual(windowRows(data.samples,[10,10.875]),[]);
  assert.deepEqual(windowRows(data.samples,[10,10.875],true),data.samples);
  assert.equal(timeBounds({samples:[],estimates:[],vslam:[],contact_s:null}),null);
});

test('制御状態を元画像時刻に一度だけ登録し、GT時刻と指令生成時刻を取り違えない',()=>{
  const control = {t:5,control_image_s:4.9,tracking_lateral_error_m:.1,image_to_control_ms:30,
    image_to_apply_ms:null,command_age_ms:100,steering_rad:.2,requested_steering_rad:.3};
  const samples = [control,{...control,t:5.05},{...control,t:5.1,control_image_s:null},
    {...control,t:5.2,control_image_s:5.1,tracking_lateral_error_m:-.15}];
  const rows = controlObservations(samples);
  assert.deepEqual(rows.map(r=>[r.t,r.segment,r.lateral,r.delay]),[[4.9,1,.1,30],[5.1,2,-.15,30]]);
  const charts = chartsFor({samples,estimates:[],vslam:[]});
  const s = charts[0].series[1];
  assert.equal(valueAt(s.rows,4.9,s.get),.1);
  assert.equal(valueAt(s.rows,5,s.get),null);
  const timing = charts.find(c=>c.label==='画像からの遅れ').series;
  assert.equal(valueAt(timing[1].rows,4.9,timing[1].get),30);
  assert.equal(valueAt(timing[2].rows,5,timing[2].get),null);
  assert.equal(valueAt(timing[3].rows,5,timing[3].get),100);
});

test('地図補正・画像から生成した指令・採用中の操舵は各記録の時刻を使う',()=>{
  const ekf=[{t:2,segment:1,correction_left_m:.2,correction_forward_m:0,
    correction_yaw_deg:3,steering_rad:.3}];
  const charts=chartsFor({ekf,samples:[],estimates:[],vslam:[]});
  const shift=charts.find(c=>c.label==='地図補正の移動量').series[1];
  assert.equal(valueAt(shift.rows,2,shift.get),.2);
  const steering=charts.find(c=>c.label==='操舵角').series;
  assert.equal(valueAt(steering[1].rows,2,steering[1].get),null);
  assert.ok(Math.abs(valueAt(steering[2].rows,2,steering[2].get)-.3*180/Math.PI)<1e-9);
});
