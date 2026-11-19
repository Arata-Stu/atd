export interface TransferPreset {
  name: string;
  host: string;
}

const USB_HOST = '192.168.55.1';
const PRESETS_KEY = 'atd.transfer.host-presets.v1';
export const BUILTIN_TRANSFER_PRESETS: readonly TransferPreset[] = [
  { name: 'USB', host: USB_HOST },
  { name: 'Hotspot', host: '10.42.0.1' },
];

export function addTransferPreset(custom: TransferPreset[], name: string, host: string): TransferPreset[] {
  const preset = { name: name.trim(), host: host.trim() };
  if (!preset.name) throw new Error('プリセット名を入力してください。');
  if (!/^[A-Za-z0-9][A-Za-z0-9.-]{0,252}$/.test(preset.host)) {
    throw new Error('ホスト名または IPv4 アドレスを入力してください。');
  }
  const existing = [...BUILTIN_TRANSFER_PRESETS, ...custom];
  if (existing.some(item => item.name.toLowerCase() === preset.name.toLowerCase())) {
    throw new Error('同じ名前のプリセットがあります。');
  }
  if (existing.some(item => item.host.toLowerCase() === preset.host.toLowerCase())) {
    throw new Error('この接続先は既に登録されています。');
  }
  return [...custom, preset];
}

export function loadTransferPresets(storage?: Pick<Storage, 'getItem'>): TransferPreset[] {
  let stored: unknown;
  try {
    stored = JSON.parse((storage ?? window.localStorage).getItem(PRESETS_KEY) ?? '[]');
  } catch {
    return [];
  }
  let custom: TransferPreset[] = [];
  if (!Array.isArray(stored)) return custom;
  for (const item of stored) {
    if (typeof item?.name !== 'string' || typeof item?.host !== 'string') continue;
    try {
      custom = addTransferPreset(custom, item.name, item.host);
    } catch {
      // 登録内容が壊れた場合も、他の有効な接続先を残す。
    }
  }
  return custom;
}

export function saveTransferPresets(custom: TransferPreset[], storage?: Pick<Storage, 'setItem'>): void {
  try {
    (storage ?? window.localStorage).setItem(PRESETS_KEY, JSON.stringify(custom));
  } catch {
    throw new Error('このブラウザにプリセットを保存できません。ストレージ設定を確認してください。');
  }
}

export function transferNetwork(host: string, custom: TransferPreset[]): { isUsb: boolean; label: string } {
  const destination = host.toLowerCase();
  const preset = [...BUILTIN_TRANSFER_PRESETS, ...custom].find(item => item.host.toLowerCase() === destination);
  return { isUsb: destination === USB_HOST, label: preset?.name ?? '手入力' };
}
