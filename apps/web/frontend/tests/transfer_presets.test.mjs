import assert from 'node:assert/strict';
import test from 'node:test';
import { addTransferPreset, BUILTIN_TRANSFER_PRESETS, loadTransferPresets, saveTransferPresets, transferNetwork } from '../src/transferPresets.ts';

test('only the exact USB endpoint skips the transfer warning', () => {
  assert.deepEqual(BUILTIN_TRANSFER_PRESETS.map(p => p.host), ['192.168.55.1', '10.42.0.1']);
  assert.deepEqual(transferNetwork('192.168.55.1', []), { isUsb: true, label: 'USB' });
  assert.deepEqual(transferNetwork('10.42.0.1', []), { isUsb: false, label: 'Hotspot' });
  assert.deepEqual(transferNetwork('JETSON.local', [{ name: '研究室', host: 'jetson.local' }]), { isUsb: false, label: '研究室' });
  for (const host of ['192.168.55.10', '192.168.55.1.example', 'usb.local', '']) {
    assert.equal(transferNetwork(host, [{ name: 'USB', host }]).isUsb, false);
  }
});

test('adding a custom preset keeps both built-in hosts and names reserved', () => {
  const preset = addTransferPreset([], ' 研究室 Wi-Fi ', ' jetson.local ');
  assert.deepEqual(preset, [{ name: '研究室 Wi-Fi', host: 'jetson.local' }]);
  for (const [name, host] of [
    ['usb', '192.168.1.5'], ['有線', '192.168.55.1'], ['名前', 'JETSON.local'],
    ['研究室 Wi-Fi', '10.1.1.2'], ['', '10.1.1.2'], ['wifi', 'atd@192.168.1.5'],
  ]) {
    assert.throws(() => addTransferPreset(preset, name, host));
  }
  assert.equal(preset.length, 1);
});

test('custom presets survive reload and invalid storage entries leave valid ones usable', () => {
  const entries = new Map();
  const storage = { getItem: key => entries.get(key) ?? null, setItem: (key, value) => entries.set(key, value) };
  const presets = addTransferPreset([], '自宅 Wi-Fi', '192.168.1.20');
  assert.deepEqual(loadTransferPresets(storage), []);
  saveTransferPresets(presets, storage);
  assert.deepEqual(loadTransferPresets(storage), presets);
  const key = [...entries.keys()][0];
  entries.set(key, JSON.stringify([null, 'wrong', {name: 'USB', host: 'elsewhere'}, ...presets, ...presets]));
  assert.deepEqual(loadTransferPresets(storage), presets);
  entries.set(key, 'broken JSON');
  assert.deepEqual(loadTransferPresets(storage), []);
  assert.deepEqual(loadTransferPresets({getItem() { throw new Error(); }}), []);
  assert.throws(() => saveTransferPresets([], {setItem() { throw new Error(); }}), /保存できません/);
});
