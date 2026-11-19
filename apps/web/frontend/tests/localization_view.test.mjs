import assert from 'node:assert/strict';
import test from 'node:test';
import { mapIssue, streamId } from '../src/localization_view.ts';

const source = {map_id:'course', map_sha256:'v', hdmap_sha256:'h', session_id:'node-1', epoch:1, state:'tracking'};
const ref = {document:{id:'course', frame:'map'}, map_sha256:'v', hdmap_sha256:'h'};

test('position overlay requires the deployed visual map and exact HDMap/Raceline', () => {
  assert.equal(mapIssue(source, ref), '');
  for (const change of [{map_id:'other'}, {map_sha256:'different'}, {hdmap_sha256:'older'}])
    assert.notEqual(mapIssue({...source, ...change}, ref), '');
  assert.notEqual(mapIssue(source, {...ref, map_sha256:null}), '');
});
test('tracking histories break on disconnect, loss, stale data, new map or localization epoch', () => {
  const live = {connected:true, localization:source, identity:{instance_id:'bridge'}};
  const id = streamId(live);
  assert.ok(id);
  for (const change of [{state:'lost'}, {state:'waiting_hint'}, {stale:true}])
    assert.equal(streamId({...live, localization:{...source,...change}}), '');
  assert.equal(streamId({...live, connected:false}), '');
  for (const change of [{epoch:2}, {session_id:'node-2'}, {hdmap_sha256:'changed'}, {map_sha256:'changed'}])
    assert.notEqual(streamId({...live, localization:{...source,...change}}), id);
});
