import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react';
import { api, bytes } from './api';
import type { Bag, CaptureOptions, Cloud, EditTarget, GenerationSettings, Job, LineKind, MapDocument, RacelineLimitKey } from './types';
import { MapCanvas } from './MapCanvas';
import { Field, Icon, Modal, Tag } from './workspace_ui';
import { CaptureModal, PreparationPanel } from './MapPreparation';
import { VglExportsModal } from './VglExportsModal';
import { LineSettingsPanel } from './LineSettingsPanel';
import type { GazeboMapEntry, MapScope } from './MapLibraryDialog';
import { ACTIVE_MAP_JOBS, captureCloudNotes, completionUpdate, DEFAULT_GENERATION_SETTINGS, editingIssue, generationIssue, nvidiaMap, restoredSettings } from './map_workflow';
import type { TrackedMapJob } from './map_workflow';

const MapLibraryDialog = lazy(() => import('./MapLibraryDialog'));
type Notice = (message: string, error?: boolean) => void;
interface Props { notify: Notice; onDirty: (value: boolean) => void; onJob: () => void; jobs: Job[]; onTransfer: () => void }
const BUILD_DEFAULTS = { id: '', bag: '', left_image: '/camera/camera/infra1/image_rect_raw', right_image: '/camera/camera/infra2/image_rect_raw', left_info: '/camera/camera/infra1/camera_info', right_info: '/camera/camera/infra2/camera_info', base_frame: 'base_link', left_frame: 'camera_infra1_optical_frame', right_frame: 'camera_infra2_optical_frame' };

export function MapWorkspace({ notify, onDirty, onJob, jobs, onTransfer }: Props) {
  const [entry] = useState(() => new URLSearchParams(window.location.hash.split('?')[1]));
  const [maps, setMaps] = useState<MapDocument[]>([]), [bags, setBags] = useState<Bag[]>([]);
  const [doc, setDoc] = useState<MapDocument | null>(null), [cloud, setCloud] = useState<Cloud | null>(null);
  const [dirty, setDirty] = useState(false), [busy, setBusy] = useState(false), [target, setTarget] = useState<EditTarget>('left');
  const [mode, setMode] = useState<'select' | 'add' | 'pan'>('select'), [fitKey, setFitKey] = useState(0);
  const [visible, setVisible] = useState<Record<string, boolean>>({ centerline: true, raceline: true, custom: true, cloud: true });
  const [settings, setSettings] = useState<GenerationSettings>(DEFAULT_GENERATION_SETTINGS);
  const [kind, setKind] = useState<LineKind>(() => entry.get('line') === 'raceline' ? 'raceline' : entry.get('line') === 'custom' ? 'custom' : 'centerline');
  const [sample, setSample] = useState(entry.get('sample') === 'odaiba' ? 'odaiba' : '');
  const [newId, setNewId] = useState(sample), [showCreate, setShowCreate] = useState(!!sample), [showBuild, setShowBuild] = useState(false), [showCapture, setShowCapture] = useState(false);
  const [build, setBuild] = useState(BUILD_DEFAULTS);
  const [gazeboMaps, setGazeboMaps] = useState<GazeboMapEntry[]>([]), [gazeboKey, setGazeboKey] = useState('');
  const [libraryAction, setLibraryAction] = useState<{ operation: 'duplicate' | 'delete'; item: { id: string; revision: number } | GazeboMapEntry } | null>(null);
  const [vglDialog, setVglDialog] = useState<'library' | 'prepare' | null>(null);
  const [capture, setCapture] = useState<CaptureOptions | null>(null), [captureLoading, setCaptureLoading] = useState(false), [captureError, setCaptureError] = useState('');
  const [captureBag, setCaptureBag] = useState(''), [captureModel, setCaptureModel] = useState(''), [captureRefresh, setCaptureRefresh] = useState(0);
  const [deferred, setDeferred] = useState<string | null>(null), [localJobs, setLocalJobs] = useState<Record<string, string>>({});
  const undo = useRef<MapDocument[]>([]), redo = useRef<MapDocument[]>([]), [history, setHistory] = useState([0, 0]);
  const cloudRef = useRef(cloud), docRef = useRef(doc), dirtyRef = useRef(dirty), kindRef = useRef(kind), loadSequence = useRef(0);
  const previousJobs = useRef(new Map<string, string>()), trackedJobs = useRef(new Map<string, TrackedMapJob>());
  cloudRef.current = cloud; docRef.current = doc; dirtyRef.current = dirty; kindRef.current = kind;

  const refresh = useCallback(async () => {
    const [catalog, records] = await Promise.all([api<{ items: MapDocument[]; gazebo_maps?: GazeboMapEntry[] }>('maps'), api<{ items: Bag[] }>('records')]);
    setMaps(catalog.items); setGazeboMaps(catalog.gazebo_maps ?? []); setBags(records.items);
  }, []);
  const reload = useCallback(async (id: string, force = false, selection = docRef.current?.id ?? null) => {
    const sequence = ++loadSequence.current;
    const [document, points] = await Promise.all([api<MapDocument>('maps/' + encodeURIComponent(id)), api<Cloud>('maps/' + encodeURIComponent(id) + '/cloud')]);
    if (sequence !== loadSequence.current) return;
    if (!force && (dirtyRef.current || (docRef.current?.id ?? null) !== selection)) {
      if (docRef.current?.id === id) setDeferred(id);
      return;
    }
    const refit = docRef.current?.id !== id || !cloudRef.current?.points.length && points.points.length > 0;
    docRef.current = document; dirtyRef.current = false;
    setDoc(document); setCloud(points); setDirty(false); setDeferred(null);
    setSettings(restoredSettings(document, kindRef.current));
    undo.current = []; redo.current = []; setHistory([0, 0]);
    if (refit) setFitKey(value => value + 1);
  }, []);
  useEffect(() => { void refresh().catch(e => notify(e.message, true)); }, [refresh, notify]);
  useEffect(() => {
    const id = entry.get('map');
    if (id) void reload(id, true).catch(e => notify(e.message, true));
    return () => { ++loadSequence.current; };
  }, [entry, reload, notify]);
  useEffect(() => {
    if (doc?.id) window.history.replaceState(null, '', '#map?' + new URLSearchParams({ map: doc.id, line: kind }));
  }, [doc?.id, kind]);
  useEffect(() => { onDirty(dirty); }, [dirty, onDirty]);
  useEffect(() => { setCaptureBag(''); setCaptureModel(''); setCapture(null); setCaptureError(''); setShowCapture(false); }, [doc?.id]);
  useEffect(() => {
    if (!doc || !nvidiaMap(doc)) { setCapture(null); setCaptureLoading(false); return; }
    let cancelled = false; setCaptureLoading(true); setCaptureError('');
    const id = doc.id;
    const timer = setTimeout(async () => {
      const query = new URLSearchParams(); if (captureBag) query.set('bag', captureBag); if (captureModel) query.set('model_dir', captureModel);
      try { const result = await api<CaptureOptions>(`maps/${encodeURIComponent(id)}/capture${query.size ? '?' + query : ''}`); if (!cancelled) setCapture(result); }
      catch (e) { if (!cancelled) { setCapture(null); setCaptureError((e as Error).message); } }
      finally { if (!cancelled) setCaptureLoading(false); }
    }, 350);
    return () => { cancelled = true; clearTimeout(timer); };
  }, [doc?.id, doc?.revision, captureBag, captureModel, captureRefresh]);
  useEffect(() => {
    const terminal = new Set<string>();
    for (const job of jobs) {
      const prior = previousJobs.current.get(job.id), tracked = trackedJobs.current.get(job.id);
      previousJobs.current.set(job.id, job.status);
      if (!ACTIVE_MAP_JOBS.has(job.status)) terminal.add(job.id);
      const selectionAtCompletion = docRef.current?.id ?? null;
      const update = completionUpdate(job, prior, tracked, selectionAtCompletion, dirtyRef.current);
      if (update) {
        trackedJobs.current.delete(job.id);
        void refresh().then(async () => {
          if (update.action === 'defer') { setDeferred(update.mapId); notify('処理が完了しました。未保存の編集を保持しています'); }
          else if (update.action === 'refresh' || update.action === 'open') {
            await reload(update.mapId, false, selectionAtCompletion);
            setCaptureRefresh(value => value + 1);
          }
        }).catch(e => notify(e.message, true));
      } else if (tracked && !ACTIVE_MAP_JOBS.has(job.status)) {
        trackedJobs.current.delete(job.id);
        if (job.status === 'failed' || job.status === 'interrupted' || job.status === 'orphaned') notify(`${job.title || '処理'}: ${job.message || '完了を確認できませんでした'}`, true);
      }
    }
    if (terminal.size) setLocalJobs(previous => Object.keys(previous).some(id => terminal.has(id)) ? Object.fromEntries(Object.entries(previous).filter(([id]) => !terminal.has(id))) : previous);
  }, [jobs, notify, refresh, reload]);

  const nvidia = nvidiaMap(doc), editIssue = editingIssue(doc, cloud);
  const activeJob = jobs.find(job => ACTIVE_MAP_JOBS.has(job.status) && job.resources?.includes('map:' + doc?.id));
  const mapWorking = !!activeJob || !!doc && Object.values(localJobs).includes(doc.id);
  const locked = busy || mapWorking;
  const editable = !!doc && !editIssue && !locked;
  const pointcloudReady = nvidia && !editIssue;
  const cloudNotes = captureCloudNotes(cloud?.provenance);
  async function run(action: () => Promise<void>) { setBusy(true); try { await action(); } catch (e) { notify((e as Error).message, true); } finally { setBusy(false); } }
  async function load(id: string) {
    if (dirtyRef.current && !window.confirm('未保存の編集を破棄して地図を開きますか？')) return;
    await run(() => reload(id, true));
  }
  function change(next: MapDocument) {
    if (!doc || locked || editIssue) return;
    undo.current.push(structuredClone(doc)); if (undo.current.length > 60) undo.current.shift(); redo.current = [];
    dirtyRef.current = true; setDoc(next); setDirty(true); setHistory([undo.current.length, 0]);
  }
  function historyStep(forward: boolean) {
    if (!doc || locked || editIssue) return;
    const from = forward ? redo.current : undo.current, to = forward ? undo.current : redo.current;
    const next = from.pop(); if (!next) return; to.push(doc); dirtyRef.current = true;
    setDoc({ ...next, revision: doc.revision, raceline_calibration: doc.raceline_calibration });
    setDirty(true); setHistory([undo.current.length, redo.current.length]);
  }
  async function save(): Promise<MapDocument> {
    if (!doc) throw Error('地図を選択してください');
    if (!dirty) return doc;
    const saved = await api<MapDocument>('maps/' + encodeURIComponent(doc.id), doc, 'PUT');
    docRef.current = saved; dirtyRef.current = false; setDoc(saved); setDirty(false); await refresh(); return saved;
  }
  async function queue(path: string, body: unknown, mapId: string, catalogOnly = false) {
    const job = await api<Job>(path, body);
    trackedJobs.current.set(job.id, { mapId, selectionId: docRef.current?.id ?? null, catalogOnly });
    setLocalJobs(previous => ({ ...previous, [job.id]: mapId })); onJob();
  }
  function generate(nextKind: LineKind) {
    void run(async () => {
      const issue = generationIssue(doc, nextKind, cloud); if (issue) throw Error(issue);
      const saved = await save();
      const nextSettings = kind === nextKind ? settings : restoredSettings(saved, nextKind);
      setKind(nextKind); setSettings(nextSettings);
      await queue(`maps/${encodeURIComponent(saved.id)}/generate`, { kind: nextKind, revision: saved.revision, settings: nextSettings }, saved.id);
      notify('ライン生成を開始しました。完了すると表示を更新します');
    });
  }
  function clearCalibration() {
    if (!doc || dirty || !doc.raceline_calibration) return;
    const freed = Object.keys(doc.raceline_calibration.limits) as RacelineLimitKey[];
    void run(async () => {
      const saved = await api<MapDocument>(`maps/${encodeURIComponent(doc.id)}/raceline-calibration?revision=${doc.revision}`, undefined, 'DELETE');
      docRef.current = saved; setDoc(saved);
      const restored = restoredSettings(saved, 'raceline');
      setSettings(current => {
        const next = { ...current };
        for (const key of freed) next[key] = restored[key];
        return next;
      });
      await refresh();
      notify('この地図の校正制限を解除しました');
    });
  }
  function startCapture() {
    if (!doc) return;
    void run(async () => {
      if (dirty) throw Error('未保存の編集があります。先に保存してください');
      await queue(`maps/${encodeURIComponent(doc.id)}/capture`, { revision: doc.revision, ...(captureBag ? { bag: captureBag } : {}), ...(captureModel ? { model_dir: captureModel } : {}) }, doc.id);
      setShowCapture(false); notify('点群の取得を開始しました。完了するとHDMap編集へ進めます');
    });
  }
  async function importCloud(file: File) {
    if (file.size > 25 * 1024 * 1024) throw Error('25 MB以下の点群JSONを指定してください');
    const input = JSON.parse(await file.text());
    if (input.frame !== 'map' || !Array.isArray(input.points)) throw Error('frame: map と points: [[x,y,z], …] を持つJSONを指定してください');
    const current = await save();
    await api<MapDocument>(`maps/${encodeURIComponent(current.id)}/cloud`, { revision: current.revision, frame: 'map', points: input.points });
    await reload(current.id, false, current.id); setFitKey(value => value + 1); await refresh(); notify('map座標の点群を読み込みました');
  }
  function manageMap(scope: MapScope, destination: string) {
    if (!libraryAction || busy) return;
    const item = libraryAction.item;
    const path = 'maps/' + encodeURIComponent(item.id);
    const gazebo = 'gazebo_manifest_sha256' in item;
    const selection = gazebo ? { scope, gazebo_manifest_sha256: item.gazebo_manifest_sha256 } : { scope, revision: item.revision };
    void run(async () => {
      if (libraryAction.operation === 'duplicate') {
        const payload = { ...selection, destination };
        await queue(path + '/duplicate', payload, destination, gazebo);
        setLibraryAction(null);
        notify(`${destination} への複製を開始しました`);
        return;
      }
      await api(path, selection, 'DELETE');
      setLibraryAction(null);
      if (gazebo) {
        setGazeboKey('');
      } else if (scope === 'map' && docRef.current?.id === item.id) {
        ++loadSequence.current;
        docRef.current = null; cloudRef.current = null;
        setDoc(null); setCloud(null); setDeferred(null);
        undo.current = []; redo.current = []; setHistory([0, 0]);
        window.history.replaceState(null, '', '#map');
      } else if (scope === 'hdmap' && docRef.current?.id === item.id) {
        await reload(item.id, true);
      }
      await refresh();
      notify(`${item.id} の${scope === 'map' ? '地図全体' : 'HDMap編集内容'}を削除しました`);
    });
  }

  const gazeboSelection = gazeboMaps.find(item => item.id === gazeboKey);
  const gazeboBusy = busy || jobs.some(job => ACTIVE_MAP_JOBS.has(job.status) && job.resources?.includes('map:' + gazeboKey));
  return <div className="workspace-grid">
    <aside className="catalog-panel">
      <div className="panel-title"><span>マイライブラリ</span><button className="icon-button" aria-label="地図一覧を更新" disabled={busy} onClick={() => void run(refresh)}><Icon name="refresh" size={16}/></button></div>
      <button className="button primary full" disabled={busy} onClick={() => setShowBuild(true)}><Icon name="plus" size={17}/>rosbag から地図を作成</button>
      <button className="button full vgl-library-button" onClick={() => setVglDialog('library')}><Icon name="folder" size={16}/>VGLモデル一覧</button>
      <div className="catalog-heading"><span>保存済みの地図</span><span>{maps.length}</span></div>
      <div className="map-list">{maps.map(map => <button key={map.id} className={'map-card ' + (doc?.id === map.id ? 'selected' : '')} disabled={busy} onClick={() => void load(map.id)}><span className="map-card-icon"><Icon name="map" size={21}/></span><span><strong>{map.title || map.id}</strong><small>{map.id}</small><small>{nvidiaMap(map) && map.source?.pointcloud_status !== 'ready' ? '点群の取得待ち' : `${map.left.length + map.right.length} 境界点`} · rev {map.revision}</small></span><Icon name="chevron" size={15}/></button>)}{!maps.length && <p className="panel-empty">保存された地図がありません。<br/>「HDMapを作成」からお台場サンプルを開けます。</p>}</div>
      <button className="button quiet full" disabled={busy} onClick={() => { setSample(''); setNewId(''); setShowCreate(true); }}><Icon name="plus" size={16}/>HDMapを作成</button>
      {doc && <div className="button-row" title={dirty ? '先に編集を保存するか、再読込してください' : doc.id}>
        <button className="button" disabled={locked || dirty} onClick={() => setLibraryAction({ operation: 'duplicate', item: doc })}>地図を複製</button>
        <button className="button" disabled={locked || dirty} onClick={() => setLibraryAction({ operation: 'delete', item: doc })}>地図を削除</button>
      </div>}
      {gazeboMaps.length > 0 && <details><summary>Gazebo 保存地図（{gazeboMaps.length}）</summary>
        <Field label="HDMap のない cuVSLAM 地図"><select value={gazeboSelection?.id ?? ''} onChange={e => setGazeboKey(e.target.value)}>
          <option value="">地図を選択</option>{gazeboMaps.map(item => <option key={item.id} value={item.id}>{item.id}</option>)}
        </select></Field>
        <div className="button-row">
          <button className="button" disabled={!gazeboSelection || gazeboBusy} onClick={() => gazeboSelection && setLibraryAction({ operation: 'duplicate', item: gazeboSelection })}>地図を複製</button>
          <button className="button" disabled={!gazeboSelection || gazeboBusy} onClick={() => gazeboSelection && setLibraryAction({ operation: 'delete', item: gazeboSelection })}>地図を削除</button>
        </div>
      </details>}
      <div className="sidebar-note"><Icon name="info" size={16}/><p>cuVSLAM・VGL地図を準備し、保存地図から点群を取得してHDMapを編集します。</p></div>
      <div className="storage-label"><Icon name="folder" size={15}/><span>maps / hdmap / lines</span></div>
    </aside>
    <section className="map-editor">
      <header className="editor-header"><div><span className="eyebrow">MAP WORKSPACE</span><h2>{doc?.title || '地図と走行ライン'}{doc && dirty && <span className="unsaved">未保存</span>}</h2></div><div className="button-row"><button className="icon-button" aria-label="元に戻す" title="元に戻す" disabled={!history[0] || !editable} onClick={() => historyStep(false)}><Icon name="undo" size={18}/></button><button className="icon-button" aria-label="やり直す" title="やり直す" disabled={!history[1] || !editable} onClick={() => historyStep(true)}><Icon name="redo" size={18}/></button><button className="icon-button" aria-label="選択した地図を再読込" title="選択した地図を再読込" disabled={!doc || busy} onClick={() => doc && void load(doc.id)}><Icon name="refresh" size={16}/></button><span className="divider"/><button className="button" disabled={!doc || !dirty || locked || !!editIssue} onClick={() => void run(async () => { await save(); notify('地図を保存しました'); })}><Icon name="save" size={16}/>保存</button></div></header>
      {mapWorking && <div className="map-operation-note"><Icon name="live" size={15}/><span>{activeJob?.message || '処理を開始しています'} · 完了後に地図を更新します</span></div>}
      {deferred === doc?.id && <div className="map-update-note"><p>新しい処理結果があります。未保存の編集を保持しています。</p><button className="text-button" onClick={() => doc && void load(doc.id)}>保存済み結果を確認</button></div>}
      <div className="editor-tools"><div className="segmented">{(['left', 'right', 'custom'] as EditTarget[]).map(value => <button key={value} disabled={!editable} className={target === value ? 'active ' + value : ''} onClick={() => setTarget(value)}><span className={'line-dot ' + value}/>{{ left: '左境界', right: '右境界', custom: 'Custom' }[value]}</button>)}</div><div className="segmented compact">{(['select', 'add', 'pan'] as const).map(value => <button key={value} disabled={value !== 'pan' && !editable} className={mode === value ? 'active' : ''} onClick={() => setMode(value)}>{{ select: '点を編集', add: '点を追加', pan: '移動' }[value]}</button>)}</div></div>
      <div className="map-canvas-stage"><MapCanvas doc={doc} cloud={cloud} target={target} mode={editable ? mode : 'pan'} visible={visible} fitKey={fitKey} onChange={editable ? (key, points) => doc && change({ ...doc, [key]: points, lines: key === 'custom' ? Object.fromEntries(Object.entries(doc.lines).filter(([line]) => line !== 'custom')) : {} }) : undefined}/>
        {nvidia && editIssue && !mapWorking && <div className="capture-stage"><span className="empty-icon"><Icon name="map" size={27}/></span><h3>点群を取得して、編集を始める。</h3><p>cuVSLAM地図は作成済みです。<br/>{capture?.vgl_ready ? '元bagとVGLモデルは自動で選択します。' : 'VGL地図の準備後に、保存地図から点群を取得します。'}</p><button className="button primary" disabled={locked || captureLoading} onClick={() => setShowCapture(true)}><Icon name="arrow" size={16}/>点群取得の準備を確認</button></div>}
      </div>
      <footer className="editor-footer"><div className="legend">{['centerline', 'raceline', 'custom'].map(line => <label key={line}><input type="checkbox" checked={visible[line]} onChange={e => setVisible({ ...visible, [line]: e.target.checked })}/><i className={line}/>{line === 'centerline' ? 'Center' : line === 'raceline' ? 'Race' : 'Custom'}</label>)}</div><span>右ドラッグで移動 · ホイールで拡大</span></footer>
    </section>
    <aside className="properties-panel">
      <div className="panel-title"><span>地図の設定</span><Icon name="settings" size={17}/></div>
      {doc ? <>
        {nvidia && <PreparationPanel doc={doc} options={capture} loading={captureLoading} error={captureError} pointcloudReady={pointcloudReady} busy={locked} onVgl={() => setVglDialog('prepare')} onCapture={() => setShowCapture(true)} onRefresh={() => setCaptureRefresh(value => value + 1)}/>}
        <div className="property-section"><div className="section-top"><h3>HDMap</h3><Tag>rev {doc.revision}</Tag></div><Field label="地図の名前"><input value={doc.title} onChange={e => change({ ...doc, title: e.target.value })} disabled={!editable}/></Field><label className="check-row"><input type="checkbox" checked={doc.closed} onChange={e => change({ ...doc, closed: e.target.checked, lines: {} })} disabled={!editable}/>閉路として編集</label><div className="geometry-stats"><span>左境界<strong>{doc.left.length}</strong></span><span>右境界<strong>{doc.right.length}</strong></span><span>Custom<strong>{doc.custom.length}</strong></span></div>
          {cloud?.points.length ? <p className="small-note">点群 {cloud.points.length.toLocaleString()} 点{cloud.truncated ? '（表示用に間引き）' : ''}</p> : <p className="small-note">{nvidia ? '保存地図から点群を取得すると編集できます。' : '手動HDMap · 点群なしで編集できます。'}</p>}
          {nvidia && !!cloud?.points.length && <div className="cloud-scope-note"><p className="small-note">{cloudNotes.scope}</p>{cloudNotes.tail && <p className="small-note">{cloudNotes.tail}</p>}</div>}
          {!nvidia && <details><summary>背景点群をJSONから読み込む</summary><label className="button full file-button"><Icon name="upload" size={15}/>点群JSONを選択<input type="file" accept="application/json,.json" disabled={locked} onChange={e => { const file = e.target.files?.[0]; e.target.value = ''; if (file) void run(() => importCloud(file)); }}/></label><p className="small-note">map座標・メートル単位の [x,y,z] 点群を使います。</p></details>}
        </div>
        <LineSettingsPanel doc={doc} kind={kind} settings={settings} busy={locked || !!editIssue} issue={generationIssue(doc, kind, cloud)} onKind={value => { setKind(value); setSettings(restoredSettings(doc, value)); }} onSettings={setSettings} onGenerate={generate} onClearCalibration={clearCalibration} dirty={dirty}/>
      </> : <div className="properties-empty"><Icon name="settings" size={28}/><p>地図を選択すると<br/>準備状態と編集設定が表示されます。</p><div className="workflow-step"><b>01</b><span>cuVSLAM地図を作成</span></div><div className="workflow-step"><b>02</b><span>VGL地図を準備</span></div><div className="workflow-step"><b>03</b><span>保存地図から点群を取得</span></div><div className="workflow-step"><b>04</b><span>HDMap・走行ラインを編集</span></div></div>}
    </aside>
    {showCreate && <Modal title="HDMapを作成" subtitle="このNotebookに保存する新しい地図IDを指定してください。" onClose={() => !busy && setShowCreate(false)}>
      <form onSubmit={e => { e.preventDefault(); void run(async () => {
        if (dirty && !window.confirm('未保存の編集を破棄して新しいHDMapを開きますか？')) return;
        const created = await api<MapDocument>('maps', { id: newId, title: newId, sample: sample || null });
        await refresh(); setShowCreate(false); await reload(created.id, true);
      }); }}>
        <Field label="作成元"><select disabled={busy} value={sample} onChange={e => {
          const value = e.target.value;
          if (!newId || newId === sample) setNewId(value);
          setSample(value);
        }}><option value="">空のHDMap</option><option value="odaiba">お台場 SKY TRACK（1/5 サンプル）</option></select></Field>
        <Field label="地図ID" hint="英数字、ハイフン、アンダースコア"><input required maxLength={96} disabled={busy} pattern="[A-Za-z0-9][A-Za-z0-9_\-]*" value={newId} onChange={e => setNewId(e.target.value)} placeholder="practice_course"/></Field>
        <div className="modal-actions"><button className="button primary" disabled={busy}>作成する<Icon name="arrow" size={16}/></button></div>
      </form></Modal>}
    {showBuild && <Modal title="rosbag から地図を作成" subtitle="最初にcuVSLAM地図を作成します。完了後、VGLと点群取得へ進めます。" onClose={() => setShowBuild(false)}><form onSubmit={e => { e.preventDefault(); void run(async () => { await queue('maps/build', build, build.id); setShowBuild(false); notify('cuVSLAM地図作成を開始しました。完了後に作成した地図を開きます'); }); }}><Field label="記録データ"><select required value={build.bag} onChange={e => setBuild({ ...build, bag: e.target.value })}><option value="">rosbagを選択</option>{bags.map(bag => <option key={bag.id} value={bag.id}>{bag.id} · {bytes(bag.bytes)}</option>)}</select></Field><Field label="新しい地図ID"><input required pattern="[A-Za-z0-9][A-Za-z0-9_-]*" value={build.id} onChange={e => setBuild({ ...build, id: e.target.value })} placeholder="practice_course"/></Field><details><summary>画像トピック・frameの詳細</summary><div className="field-grid">{Object.entries(build).filter(([key]) => key !== 'id' && key !== 'bag').map(([key, value]) => <Field key={key} label={key}><input value={value} onChange={e => setBuild({ ...build, [key]: e.target.value })}/></Field>)}</div></details>{!bags.length && <p className="inline-note">bagがありません。「データ転送」からJetsonの記録を受信してください。</p>}<div className="modal-actions"><button className="button primary" disabled={busy || !bags.length}><Icon name="play" size={16}/>cuVSLAM地図作成を開始</button></div></form></Modal>}
    {vglDialog === 'library' && <VglExportsModal onClose={() => setVglDialog(null)}/>}
    {libraryAction && <Suspense fallback={<p role="status">地図の管理画面を読み込んでいます…</p>}><MapLibraryDialog key={libraryAction.item.id} doc={libraryAction.item} action={libraryAction.operation} busy={busy} onClose={() => setLibraryAction(null)} onSubmit={manageMap}/></Suspense>}
    {vglDialog === 'prepare' && doc && <VglExportsModal key={doc.id} onClose={() => setVglDialog(null)}
      preparation={{ title: doc.title, busy: locked, start: modelDir => void run(async () => {
        await queue(`maps/${encodeURIComponent(doc.id)}/vgl`, { model_dir: modelDir }, doc.id);
        setVglDialog(null); notify('VGL地図の準備を開始しました。完了後に点群を取得できます');
      }) }}/>}
    {showCapture && doc && <CaptureModal doc={doc} options={capture} loading={captureLoading} error={captureError} bags={bags} bag={captureBag} modelDir={captureModel} busy={locked} onBag={setCaptureBag} onModelDir={setCaptureModel} onClose={() => setShowCapture(false)} onRefresh={() => setCaptureRefresh(value => value + 1)} onStart={startCapture} onVgl={() => { setShowCapture(false); setVglDialog('prepare'); }} onTransfer={() => { setShowCapture(false); onTransfer(); }}/>}
  </div>;
}
