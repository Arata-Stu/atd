import assert from 'node:assert/strict';
import test from 'node:test';
import { errorAt, recordedLaps, selectWindow, seriesPath } from '../src/replay_errors.ts';

const point = (lap, station_m, segment = 0) => ({lap, station_m, distance_m:(lap-1)*100+station_m,
  segment, time_s:((lap-1)*100+station_m)/5});
const points = [point(1,0), point(1,18), point(1,56), point(1,89), point(1,99), point(2,0), point(2,14), point(2,89), point(2,100)];
const run = {lap_length_m:100, reference:points, samples:points};
const home = {start_m:85, end_m:20};
test('home straight is continuous across timing line, both edge passages remain selectable', () => {
  assert.deepEqual(recordedLaps(run),[1,2]);
  const cross = selectWindow(run,home,1);
  assert.deepEqual(cross.reference.map(row => row.at),[-11,-1,0,14]);
  assert.deepEqual(cross.samples.map(row => row.point.distance_m),[89,99,100,114]);
  assert.deepEqual(selectWindow(run,home,0).samples.map(row => row.at),[0,18]);
  assert.deepEqual(selectWindow(run,home,2).samples.map(row => row.at),[-11,0]);
  assert.deepEqual(selectWindow(run,{start_m:30,end_m:80},1).samples.map(row => row.at),[56]);
});
test('missing intervals including the untracked tail stay empty, no graph stroke bridges them', () => {
  const runWithGap = {...run, samples:[point(1,18),point(1,25), point(1,89,1)]};
  const scope = selectWindow(runWithGap,undefined,1);
  assert.deepEqual(scope.gaps,[[0,18],[25,89],[89,99]]);
  for (const at of [0,5,60,97]) assert.equal(errorAt(scope,at),undefined);
  for (const at of [18,20,89]) assert.ok(errorAt(scope,at));
  const d = seriesPath(scope.samples, p => [p.at,p.at], (a,b) => a.segment !== b.segment);
  assert.equal((d.match(/M/g)||[]).length,2);
});
test('without station or lap metadata every sample remains visible in accumulated distance', () => {
  const rows = points.map(p => ({...p,lap:null,station_m:null}));
  const legacy = {lap_length_m:null, reference:rows,samples:[]};
  assert.deepEqual(recordedLaps(legacy),[]);
  const scope = selectWindow(legacy,undefined,null);
  assert.equal(scope.reference.length,points.length);
  assert.equal(scope.max,200);
  assert.equal(errorAt(scope,0),undefined);
  assert.deepEqual(scope.gaps,[[0,200]]);
});
