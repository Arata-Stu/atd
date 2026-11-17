import { useCallback, useEffect, useRef, useState } from 'react';
import { api, bytes } from './api';
import type { Bag, Cloud, EditTarget, MapDocument } from './types';
import { MapCanvas } from './MapCanvas';
import { Field, Icon, Modal, Tag } from './workspace_ui';

type Notice = (message: string, error?: boolean) => void;
const DEFAULT_SETTINGS = { spacing: .1, width: .19, margin: .05, max_speed: 1.5, lateral_accel: 2.5, accel: 1.5, decel: 2.5 };
export function MapWorkspace({ notify, onDirty, onJob }: { notify: Notice; onDirty: (value: boolean) => void; onJob: () => void }) {
  const [maps, setMaps] = useState<MapDocument[]>([]), [bags, setBags] = useState<Bag[]>([]);
  const [doc, setDoc] = useState<MapDocument | null>(null), [cloud, setCloud] = useState<Cloud | null>(null);
  const [dirty, setDirty] = useState(false), [busy, setBusy] = useState(false), [target, setTarget] = useState<EditTarget>('left');
  const [mode, setMode] = useState<'select' | 'add' | 'pan'>('select');
  const [visible, setVisible] = useState<Record<string, boolean>>({ centerline: true, raceline: true, custom: true });
  const [settings, setSettings] = useState(DEFAULT_SETTINGS), [kind, setKind] = useState('centerline');
  const [newId, setNewId] = useState(''), [showCreate, setShowCreate] = useState(false), [showBuild, setShowBuild] = useState(false), [showVgl, setShowVgl] = useState(false);
  const [build, setBuild] = useState({ id: '', bag: '', left_image: '/camera/camera/infra1/image_rect_raw', right_image: '/camera/camera/infra2/image_rect_raw', left_info: '/camera/camera/infra1/camera_info', right_info: '/camera/camera/infra2/camera_info', base_frame: 'base_link', left_frame: 'camera_infra1_optical_frame', right_frame: 'camera_infra2_optical_frame' });
  const [modelDir, setModelDir] = useState('');
  const undo = useRef<MapDocument[]>([]), redo = useRef<MapDocument[]>([]);
  const [history, setHistory] = useState([0, 0]);
  const refresh = useCallback(async () => {
    const [catalog, records] = await Promise.all([api<{ items: MapDocument[] }>('maps'), api<{ items: Bag[] }>('records')]);
    setMaps(catalog.items); setBags(records.items);
  }, []);
  useEffect(() => { refresh().catch(e => notify(e.message, true)); }, [refresh, notify]);
  useEffect(() => { onDirty(dirty); }, [dirty, onDirty]);
  async function run(action: () => Promise<void>) { setBusy(true); try { await action(); } catch (e) { notify((e as Error).message, true); } finally { setBusy(false); } }
  async function load(id: string) {
    if (dirty && !window.confirm('未保存の編集を破棄して地図を開きますか？')) return;
    await run(async () => {
      const document = await api<MapDocument>('maps/' + encodeURIComponent(id));
      const points = await api<Cloud>('maps/' + encodeURIComponent(id) + '/cloud').catch(() => null);
      setDoc(document); setCloud(points); setDirty(false); undo.current = []; redo.current = []; setHistory([0, 0]);
    });
  }
  function change(next: MapDocument) { if (!doc) return; undo.current.push(structuredClone(doc)); if (undo.current.length > 60) undo.current.shift(); redo.current = []; setDoc(next); setDirty(true); setHistory([undo.current.length, 0]); }
  function historyStep(forward: boolean) {
    if (!doc) return; const from = forward ? redo.current : undo.current, to = forward ? undo.current : redo.current;
    const next = from.pop(); if (!next) return; to.push(doc); setDoc({ ...next, revision: doc.revision }); setDirty(true); setHistory([undo.current.length, redo.current.length]);
  }
  async function save(): Promise<MapDocument> {
    if (!doc) throw Error('地図を選択してください');
    if (!dirty) return doc;
    const saved = await api<MapDocument>('maps/' + encodeURIComponent(doc.id), doc, 'PUT'); setDoc(saved); setDirty(false); await refresh(); return saved;
  }
  return <div className="workspace-grid">
    <aside className="catalog-panel">
      <div className="panel-title"><span>マイライブラリ</span><button className="icon-button" aria-label="地図一覧を更新" onClick={() => void run(refresh)}><Icon name="refresh" size={16}/></button></div>
      <button className="button primary full" onClick={() => setShowBuild(true)}><Icon name="plus" size={17}/>rosbag から地図を作成</button>
      <div className="catalog-heading"><span>保存済みの地図</span><span>{maps.length}</span></div>
      <div className="map-list">{maps.map(map => <button key={map.id} className={'map-card ' + (doc?.id === map.id ? 'selected' : '')} disabled={busy} onClick={() => void load(map.id)}><span className="map-card-icon"><Icon name="map" size={21}/></span><span><strong>{map.title || map.id}</strong><small>{map.id}</small><small>{map.left.length + map.right.length} 境界点 · rev {map.revision}</small></span><Icon name="chevron" size={15}/></button>)}{!maps.length && <p className="panel-empty">保存された地図がありません。<br/>地図の作成、または空のHDMapから編集を始められます。</p>}</div>
      <button className="button quiet full" onClick={() => setShowCreate(true)}><Icon name="plus" size={16}/>空のHDMapを作成</button>
      <div className="sidebar-note"><Icon name="info" size={16}/><p>地図作成と編集はこのPCで行い、完成した地図をJetsonへ転送します。</p></div>
      <div className="storage-label"><Icon name="folder" size={15}/><span>maps / hdmap / lines</span></div>
    </aside>
    <section className="map-editor">
      <header className="editor-header"><div><span className="eyebrow">MAP WORKSPACE</span><h2>{doc?.title || '地図と走行ライン'}{doc && dirty && <span className="unsaved">未保存</span>}</h2></div><div className="button-row"><button className="icon-button" aria-label="元に戻す" title="元に戻す" disabled={!history[0] || busy} onClick={() => historyStep(false)}><Icon name="undo" size={18}/></button><button className="icon-button" aria-label="やり直す" title="やり直す" disabled={!history[1] || busy} onClick={() => historyStep(true)}><Icon name="redo" size={18}/></button><span className="divider"/><button className="button" disabled={!doc || !dirty || busy} onClick={() => void run(async () => { await save(); notify('地図を保存しました'); })}><Icon name="save" size={16}/>保存</button></div></header>
      <div className="editor-tools"><div className="segmented">{(['left', 'right', 'custom'] as EditTarget[]).map(value => <button key={value} className={target === value ? 'active ' + value : ''} onClick={() => setTarget(value)}><span className={'line-dot ' + value}/>{{ left: '左境界', right: '右境界', custom: 'Custom' }[value]}</button>)}</div><div className="segmented compact">{(['select', 'add', 'pan'] as const).map(value => <button key={value} className={mode === value ? 'active' : ''} onClick={() => setMode(value)}>{{ select: '点を編集', add: '点を追加', pan: '移動' }[value]}</button>)}</div></div>
      <MapCanvas doc={doc} cloud={cloud} target={target} mode={mode} visible={visible} onChange={(key, points) => doc && change({ ...doc, [key]: points, lines: key === 'custom' ? Object.fromEntries(Object.entries(doc.lines).filter(([kind]) => kind !== 'custom')) : {} })}/>
      <footer className="editor-footer"><div className="legend">{['centerline', 'raceline', 'custom'].map(line => <label key={line}><input type="checkbox" checked={visible[line]} onChange={e => setVisible({ ...visible, [line]: e.target.checked })}/><i className={line}/>{line === 'centerline' ? 'Center' : line === 'raceline' ? 'Race' : 'Custom'}</label>)}</div><span>右ドラッグで移動 · ホイールで拡大</span></footer>
    </section>
    <aside className="properties-panel">
      <div className="panel-title"><span>地図の設定</span><Icon name="settings" size={17}/></div>
      {doc ? <><div className="property-section"><div className="section-top"><h3>HDMap</h3><Tag>rev {doc.revision}</Tag></div><Field label="地図の名前"><input value={doc.title} onChange={e => change({ ...doc, title: e.target.value })} disabled={busy}/></Field><label className="check-row"><input type="checkbox" checked={doc.closed} onChange={e => change({ ...doc, closed: e.target.checked, lines: {} })} disabled={busy}/>閉路として編集</label><div className="geometry-stats"><span>左境界<strong>{doc.left.length}</strong></span><span>右境界<strong>{doc.right.length}</strong></span><span>Custom<strong>{doc.custom.length}</strong></span></div>{cloud && cloud.points.length > 0 ? <p className="small-note">点群 {cloud.points.length.toLocaleString()} 点{cloud.truncated ? '（表示用に間引き）' : ''}</p> : <p className="small-note">表示用点群は未取得です。map座標の点群JSONを読み込めます。</p>}<label className="button full file-button"><Icon name="upload" size={15}/>点群JSONを読み込む<input type="file" accept="application/json,.json" disabled={busy} onChange={e => { const file = e.target.files?.[0]; e.target.value = ''; if (!file) return; void run(async () => { if (file.size > 80 * 1024 * 1024) throw Error('80 MB以下の点群JSONを指定してください'); const input = JSON.parse(await file.text()); if (input.frame !== 'map' || !Array.isArray(input.points)) throw Error('frame: map と points: [[x,y,z], …] を持つJSONを指定してください'); const current = await save(); const imported = await api<MapDocument>(`maps/${encodeURIComponent(current.id)}/cloud`, { revision: current.revision, frame: 'map', points: input.points }); setDoc(imported); setCloud(await api<Cloud>(`maps/${encodeURIComponent(current.id)}/cloud`)); await refresh(); notify('map座標の点群を読み込みました'); }); }}/></label></div>
      <div className="property-section"><div className="section-top"><h3>走行ライン生成</h3><span className="step-number">02</span></div><Field label="生成するライン"><select value={kind} onChange={e => setKind(e.target.value)}><option value="centerline">Centerline · 中央線</option><option value="raceline">Raceline · 曲率を低減</option><option value="custom">Customline · 編集した点列</option></select></Field><div className="field-grid">{(Object.keys(settings) as (keyof typeof settings)[]).map(key => <Field key={key} label={{ spacing: '点の間隔 [m]', width: '車幅 [m]', margin: '余裕 [m]', max_speed: '速度上限 [m/s]', lateral_accel: '横加速度 [m/s²]', accel: '加速 [m/s²]', decel: '減速 [m/s²]' }[key]}><input type="number" min="0" step="0.01" value={settings[key]} onChange={e => setSettings({ ...settings, [key]: Number(e.target.value) })}/></Field>)}</div><button className="button primary full" disabled={busy} onClick={() => void run(async () => { const saved = await save(); await api(`maps/${encodeURIComponent(saved.id)}/generate`, { kind, revision: saved.revision, settings }); onJob(); notify('ライン生成を開始しました。完了後に地図を読み直してください'); })}><Icon name="play" size={16}/>{dirty ? '保存してライン生成' : 'ラインを生成'}</button><p className="small-note">生成された速度は目標値です。実車の速度・制動性能は別に確認します。</p></div>
      <div className="property-section"><h3>NVIDIA 地図</h3><p className="small-note">cuVSLAM地図に対応するVGL地図を生成します。</p><button className="button full" disabled={busy} onClick={() => setShowVgl(true)}>VGL地図を生成<Icon name="arrow" size={16}/></button></div>
      </> : <div className="properties-empty"><Icon name="settings" size={28}/><p>地図を選択すると<br/>編集・生成設定が表示されます。</p><div className="workflow-step"><b>01</b><span>rosbagから地図を作成</span></div><div className="workflow-step"><b>02</b><span>境界・ラインを編集</span></div><div className="workflow-step"><b>03</b><span>Jetsonへ転送</span></div></div>}
    </aside>
    {showCreate && <Modal title="空のHDMapを作成" subtitle="編集用の地図IDを指定します。" onClose={() => setShowCreate(false)}><form onSubmit={e => { e.preventDefault(); void run(async () => { const created = await api<MapDocument>('maps', { id: newId, title: newId }); await refresh(); setShowCreate(false); await load(created.id); }); }}><Field label="地図ID" hint="英数字、ハイフン、アンダースコア"><input required pattern="[A-Za-z0-9][A-Za-z0-9_-]*" value={newId} onChange={e => setNewId(e.target.value)} placeholder="practice_course"/></Field><div className="modal-actions"><button className="button primary" disabled={busy}>作成する<Icon name="arrow" size={16}/></button></div></form></Modal>}
    {showBuild && <Modal title="rosbag から地図を作成" subtitle="記録済みのbagを選び、NVIDIAのオフライン処理を実行します。" onClose={() => setShowBuild(false)}><form onSubmit={e => { e.preventDefault(); void run(async () => { await api('maps/build', build); setShowBuild(false); onJob(); notify('地図作成を開始しました'); }); }}><Field label="記録データ"><select required value={build.bag} onChange={e => setBuild({ ...build, bag: e.target.value })}><option value="">rosbagを選択</option>{bags.map(bag => <option key={bag.id} value={bag.id}>{bag.id} · {bytes(bag.bytes)}</option>)}</select></Field><Field label="新しい地図ID"><input required pattern="[A-Za-z0-9][A-Za-z0-9_-]*" value={build.id} onChange={e => setBuild({ ...build, id: e.target.value })} placeholder="practice_course"/></Field><details><summary>画像トピック・frameの詳細</summary><div className="field-grid">{Object.entries(build).filter(([key]) => key !== 'id' && key !== 'bag').map(([key, value]) => <Field key={key} label={key}><input value={value} onChange={e => setBuild({ ...build, [key]: e.target.value })}/></Field>)}</div></details>{!bags.length && <p className="inline-note">bagがありません。「データ転送」からJetsonの記録を受信してください。</p>}<div className="modal-actions"><button className="button primary" disabled={busy || !bags.length}><Icon name="play" size={16}/>地図作成を開始</button></div></form></Modal>}
    {showVgl && doc && <Modal title="VGL地図を生成" subtitle={`${doc.title} のcuVSLAM地図を使用します。`} onClose={() => setShowVgl(false)}><form onSubmit={e => { e.preventDefault(); void run(async () => { await api(`maps/${encodeURIComponent(doc.id)}/vgl`, { model_dir: modelDir }); setShowVgl(false); onJob(); notify('VGL地図生成を開始しました'); }); }}><Field label="VGL export bundle（結果保存先からの相対パス）"><input required value={modelDir} onChange={e => setModelDir(e.target.value)} placeholder="models/vgl/asset/exported/export_id"/></Field><div className="modal-actions"><button className="button primary" disabled={busy}><Icon name="play" size={16}/>生成を開始</button></div></form></Modal>}
  </div>;
}
