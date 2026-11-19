import { useEffect, useState } from 'react';
import { api, bytes } from './api';
import { receivedBag } from './calibration';
import { addTransferPreset, BUILTIN_TRANSFER_PRESETS, loadTransferPresets, saveTransferPresets, transferNetwork } from './transferPresets';
import type { Job, MapDocument } from './types';
import { Field, Icon, Modal } from './ui';

interface Connection { host: string; user: string; port: number; record_root: string; map_root: string; bridge_port: number }
interface RemoteEntry { name: string; relative: string; is_dir: boolean; is_bag: boolean; bytes: number }
const connectionLabels = { host: 'ホスト名 / IP', user: 'SSHユーザー', port: 'SSHポート', bridge_port: '観測ポート' };

export function TransferWorkspace({ notify, onJob, jobs, onCalibration }: {
  notify: (message: string, error?: boolean) => void; onJob: (id?: string) => void;
  jobs: Job[]; onCalibration: (bag: string) => void;
}) {
  const lastReceived = receivedBag(jobs.find(job => job.action === 'pull'));
  const [connection, setConnection] = useState<Connection | null>(null);
  const [loadError, setLoadError] = useState('');
  const [saved, setSaved] = useState(false), [busy, setBusy] = useState(false);
  const [customPresets, setCustomPresets] = useState(() => loadTransferPresets());
  const [presetDraft, setPresetDraft] = useState<{ name: string; host: string } | null>(null);
  const [presetError, setPresetError] = useState('');
  const [maps, setMaps] = useState<MapDocument[]>([]), [entries, setEntries] = useState<RemoteEntry[]>([]);
  const [path, setPath] = useState(''), [map, setMap] = useState(''), [browsed, setBrowsed] = useState(false);
  useEffect(() => {
    const abort = new AbortController();
    api<Connection>('connection', undefined, undefined, abort.signal).then(config => {
      setConnection(config.host ? config : { ...config, host: BUILTIN_TRANSFER_PRESETS[0].host });
      setSaved(Boolean(config.host));
    }).catch(e => { if (!abort.signal.aborted) setLoadError(e.message); });
    api<{ items: MapDocument[] }>('maps', undefined, undefined, abort.signal)
      .then(result => setMaps(result.items)).catch(e => { if (!abort.signal.aborted) notify(e.message, true); });
    return () => abort.abort();
  }, [notify]);

  async function run(action: () => Promise<void>) {
    setBusy(true);
    try { await action(); } catch (e) { notify((e as Error).message, true); } finally { setBusy(false); }
  }

  if (!connection) return <div className="content-page">{loadError
    ? <p role="alert">接続設定を読み込めませんでした。{loadError} <button className="button" onClick={() => window.location.reload()}>再読み込み</button></p>
    : '接続設定を読み込み中…'}</div>;
  const configured = [connection.host, connection.user, connection.record_root, connection.map_root].every(value => value.trim());
  const canTransfer = saved && configured && !busy;
  const presets = [...BUILTIN_TRANSFER_PRESETS, ...customPresets];
  const selectedMap = maps.find(item => item.id === map);
  const source = selectedMap?.source as Record<string, unknown> | undefined;

  function changeConnection(next: Connection) {
    setConnection(next);
    setSaved(false);
    setEntries([]);
    setBrowsed(false);
    setPath('');
  }
  async function browse(relative: string) {
    const result = await api<{ items: RemoteEntry[] }>('transfer/browse', { relative, expected_connection: connection });
    setEntries(result.items); setPath(relative); setBrowsed(true);
  }
  function transfer(direction: 'pull' | 'push', target: string) {
    if (!connection || !canTransfer) return;
    const network = transferNetwork(connection.host, customPresets);
    const action = direction === 'pull' ? `rosbag「${target}」を受信` : `地図「${target}」を送信`;
    if (!network.isUsb && !window.confirm(
      `現在の接続先は ${network.label}（${connection.user}@${connection.host}）です。\nUSB 用の宛先以外で${action}しますか？`
    )) return;
    void run(async () => {
      const body = direction === 'pull' ? { relative: target } : { id: target };
      const job = await api<Job>(`transfer/${direction}`, { ...body, expected_connection: connection });
      onJob(job.id);
      notify(direction === 'pull' ? `${target} の受信を受け付けました` : '地図の送信を受け付けました');
    });
  }
  return <div className="content-page transfer-page">
    <section className="connection-card">
      <div className="connection-heading"><span className="card-symbol"><Icon name="settings"/></span><div><h3>Jetson の接続先</h3><p>SSH 鍵と known_hosts を使用します。</p></div><span className={'save-state ' + (saved && configured ? 'ok' : '')}>{!configured ? '未設定' : saved ? '保存済み' : '未保存'}</span></div>
      <form onSubmit={e => {
        e.preventDefault();
        void run(async () => {
          const result = await api<Connection>('connection', connection, 'PUT');
          setConnection(result); setSaved(true); notify('接続設定を保存しました');
        });
      }}><fieldset className="transfer-settings" disabled={busy}>
        <div className="transfer-presets" role="group" aria-label="IP プリセット">
          {presets.map(preset => <button key={preset.host} type="button" className="transfer-preset"
            aria-pressed={connection.host === preset.host}
            onClick={() => { if (connection.host !== preset.host) changeConnection({ ...connection, host: preset.host }); }}>
            <strong>{preset.name}</strong><span>{preset.host}</span>
          </button>)}
          <button type="button" className="button small" onClick={() => { setPresetError(''); setPresetDraft({ name: '', host: connection.host }); }}><Icon name="plus" size={15}/>プリセットを追加</button>
        </div>
        <div className="connection-fields">{(['host', 'user', 'port', 'bridge_port'] as const).map(key =>
          <Field key={key} label={connectionLabels[key]}><input required value={connection[key]}
            type={key.includes('port') ? 'number' : 'text'} min={key.includes('port') ? 1 : undefined} max={key.includes('port') ? 65535 : undefined}
            onChange={e => changeConnection({ ...connection, [key]: key.includes('port') ? Number(e.target.value) : e.target.value })}/></Field>
        )}</div>
        <div className="field-grid">{(['record_root', 'map_root'] as const).map(key =>
          <Field key={key} label={key === 'record_root' ? 'Jetsonホストの記録ディレクトリ' : 'Jetsonホストの地図ディレクトリ'}>
            <input required value={connection[key]} onChange={e => changeConnection({ ...connection, [key]: e.target.value })}/>
          </Field>
        )}</div>
        <div className="connection-bottom"><span>ホスト上の絶対パスを指定してください。</span><button className="button" disabled={!configured}><Icon name="save" size={16}/>接続設定を保存</button></div>
      </fieldset></form>
    </section>
    {presetDraft && <Modal title="IP プリセットを追加" onClose={() => setPresetDraft(null)}>
      <form onSubmit={e => {
        e.preventDefault();
        try {
          const next = addTransferPreset(loadTransferPresets(), presetDraft.name, presetDraft.host);
          saveTransferPresets(next); setCustomPresets(next);
          const host = presetDraft.host.trim();
          if (connection.host !== host) changeConnection({ ...connection, host });
          setPresetDraft(null);
        } catch (error) { setPresetError((error as Error).message); }
      }}>
        <Field label="プリセット名"><input autoFocus required placeholder="研究室 Wi-Fi" value={presetDraft.name} onChange={e => setPresetDraft({ ...presetDraft, name: e.target.value })}/></Field>
        <Field label="ホスト名 / IP"><input required value={presetDraft.host} onChange={e => setPresetDraft({ ...presetDraft, host: e.target.value })}/></Field>
        {presetError && <p role="alert" className="transfer-error">{presetError}</p>}
        <div className="modal-actions"><button type="button" className="button" onClick={() => setPresetDraft(null)}>キャンセル</button><button className="button primary">追加して選択</button></div>
      </form>
    </Modal>}
    <div className="transfer-columns">
      <section className="content-card">
        <div className="card-heading"><div className="transfer-direction"><span className="direction-icon"><Icon name="download"/></span><div><span className="eyebrow">JETSON → NOTEBOOK</span><h3>rosbag を受信</h3></div></div><button className="button" disabled={!canTransfer} onClick={() => void run(() => browse(path))}><Icon name="refresh" size={16}/>一覧を取得</button></div>
        <div className="remote-path"><Icon name="folder" size={17}/><span>{path || '記録ディレクトリ'}</span>{path && <button className="text-button" disabled={!canTransfer} onClick={() => void run(() => browse(path.split('/').slice(0, -1).join('/')))}>上の階層へ</button>}</div>
        <div className="remote-list">{entries.map(entry => <div className="remote-entry" key={entry.relative}>
          <span className={'entry-icon ' + (entry.is_bag ? 'bag' : '')}><Icon name={entry.is_bag ? 'list' : 'folder'}/></span>
          <button className="entry-name" disabled={!canTransfer || !entry.is_dir} onClick={() => void run(() => browse(entry.relative))}><strong>{entry.name}</strong><small>{entry.is_bag ? 'ROS BAG' : entry.is_dir ? 'フォルダ' : bytes(entry.bytes)}</small></button>
          {entry.is_bag && <button className="button small" disabled={!canTransfer} onClick={() => transfer('pull', entry.relative)}><Icon name="download" size={15}/>受信</button>}
        </div>)}{!entries.length && <div className="card-empty"><Icon name="folder" size={32}/><h4>{browsed ? 'この階層にデータはありません' : 'Jetson の記録を探す'}</h4><p>{browsed ? '上の階層または保存先を確認してください。' : '接続設定を保存し、「一覧を取得」を押してください。'}</p></div>}</div>
        <div className="card-footnote">記録を停止したbagのディレクトリ全体を転送します。{lastReceived && <div className="calibration-pull-link"><span>受信済み · {lastReceived}</span><button className="button small" onClick={() => onCalibration(lastReceived)}>この bag を車両校正へ<Icon name="arrow" size={14}/></button></div>}</div>
      </section>
      <section className="content-card">
        <div className="card-heading"><div className="transfer-direction"><span className="direction-icon upload"><Icon name="upload"/></span><div><span className="eyebrow">NOTEBOOK → JETSON</span><h3>地図を送信</h3></div></div></div>
        <div className="send-content"><Field label="送信する地図"><select value={map} onChange={e => setMap(e.target.value)}><option value="">保存済みの地図を選択</option>{maps.map(item => <option key={item.id} value={item.id}>{item.title || item.id} · rev {item.revision}</option>)}</select></Field>
          <div className="transfer-route"><span>このPC<strong>地図・HDMap・ライン</strong></span><Icon name="arrow"/><span>Jetson<strong>{connection.host || '未設定'}</strong></span></div>
          <p className="small-note">{map && !source?.cuvslam_ready && source?.kind !== 'nvidia_offline' && <strong>この地図はHDMapのみです。自己位置推定用のNVIDIA地図は含みません。<br/></strong>}地図を検証してから新しいディレクトリに配置します。転送した地図の走行への適用は別の操作です。</p>
          <button className="button primary full" disabled={!canTransfer || !map} onClick={() => transfer('push', map)}><Icon name="upload" size={17}/>地図を送信する</button>
        </div><div className="card-footnote">転送の進捗とログは右下の「ジョブ」から確認できます。</div>
      </section>
    </div>
  </div>;
}
