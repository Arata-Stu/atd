import test from 'node:test';
import assert from 'node:assert/strict';
import { validationOverlay, resultMapIssue, seconds } from '../src/localization_validation.ts';

test('trajectory overlay keeps independent segments and marks only measured boundary points', () => {
  const run = {segments:[{points:[[1,2],[2,3]]}, {points:[[5,6]]}], gaps:[
    {before:{point:[2,3]},after:{point:[5,6]}}, {before:{point:[5,6]},after:null}
  ]};
  const overlay = validationOverlay(run);
  assert.deepEqual(overlay.segments, [[[1,2],[2,3]], [[5,6]]]);
  assert.deepEqual(overlay.markers.map(x => [x.gapIndex, x.side, x.point]), [
    [0, 'before', [2,3]], [0, 'after', [5,6]], [1, 'before', [5,6]],
  ]);
  assert.deepEqual(validationOverlay({segments:[],gaps:[]}), {segments:[],markers:[]});
});

test('mismatched historical map or unresolved signature cannot overlay the results', () => {
  const matched = {document:{id:'temple', frame:'map'}, map_id:'temple', map_issue:null};
  assert.equal(resultMapIssue(matched), '');
  assert.ok(resultMapIssue({...matched, document:{id:'campus',frame:'map'}}));
  assert.ok(resultMapIssue({...matched, document:{id:'temple',frame:'odom'}}));
  assert.equal(resultMapIssue({...matched, map_issue:'HDMap hash differs'}), 'HDMap hash differs');
  assert.ok(resultMapIssue({...matched, document:null}));
  assert.equal(seconds(null),'—');
  assert.equal(seconds(0),'0.00 s');
});
