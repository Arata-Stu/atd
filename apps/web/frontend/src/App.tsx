import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react';
import { api } from './api';
import type { Job } from './types';
import { JobLog } from './JobLog';
import { Icon } from './ui';
import { ACTIVE, STATUS } from './job_status';
import { calibrationHash, receivedBag } from './calibration';
import { EscTrialWorkspace } from './EscTrialWorkspace';
import { LocalizationWorkspace } from './LocalizationWorkspace';
import { SimulationWorkspace } from './SimulationWorkspace';
import { MapWorkspace } from './MapWorkspace';
import { TransferWorkspace } from './TransferWorkspace';
import { LiveWorkspace } from './LiveWorkspace';

const InputWorkspace = lazy(() => import('./InputWorkspace').then(module => ({ default: module.InputWorkspace })));
const VehicleWorkspace = lazy(() => import('./VehicleWorkspace').then(module => ({ default: module.VehicleWorkspace })));
const CalibrationWorkspace = lazy(() => import('./CalibrationWorkspace').then(module => ({ default: module.CalibrationWorkspace })));
const ClosedLoopWorkspace = lazy(() => import('./ClosedLoopWorkspace'));
const ReplayErrorsWorkspace = lazy(() => import('./ReplayErrorsWorkspace'));
const JobHistory = lazy(() => import('./JobHistory'));

type Page = 'map' | 'transfer' | 'live' | 'input' | 'vehicle' | 'calibration' | 'localization' | 'esc' | 'simulation' | 'replay' | 'closed-loop';
const EVALUATIONS: { id: Page; title: string }[] = [
  { id: 'closed-loop', title: 'Gazebo 閉ループ' },
  { id: 'simulation', title: '2D シミュレーター' },
  { id: 'replay', title: 'VSLAM 軌跡誤差' },
  { id: 'localization', title: '自己位置検証' },
];
const PAGES: { id: Page; icon: string; title: string; subtitle: string }[] = [
  { id: 'map', icon: 'map', title: '地図とライン', subtitle: '地図作成・HDMap編集' },
  { id: 'transfer', icon: 'transfer', title: 'データ転送', subtitle: 'bag受信・地図送信' },
  { id: 'calibration', icon: 'chart', title: '車両校正', subtitle: '速度・加速度・操舵応答' },
  { id: 'closed-loop', icon: 'chart', title: '走行評価', subtitle: 'Gazebo・2D・自己位置' },
  { id: 'live', icon: 'live', title: 'ライブ', subtitle: '自動走行・自己位置' },
  { id: 'input', icon: 'settings', title: 'コントローラ', subtitle: '入力監視・配置調整' },
  { id: 'vehicle', icon: 'settings', title: '車両設定', subtitle: '実測寸法・センサ配置' },
  { id: 'esc', icon: 'controller', title: 'ESC 試験', subtitle: '試験手順・状態監視' },
];
function pageFromHash(hash: string): Page | undefined {
  const id = hash.slice(1).split('?')[0] || 'map';
  return [...PAGES, ...EVALUATIONS].find(item => item.id === id)?.id;
}

export function App() {
  const [page, setPage] = useState<Page>(() => pageFromHash(window.location.hash) ?? 'map'), [connected, setConnected] = useState(false), [notice, setNotice] = useState<{ message: string; error: boolean } | null>(null);
  const [capture, setCapture] = useState(() =>
    new URLSearchParams(window.location.hash.split('?')[1]).get('view') === 'capture');
  const [jobs, setJobs] = useState<Job[]>([]), [drawer, setDrawer] = useState(false), [jobId, setJobId] = useState('');
  const [manageHistory, setManageHistory] = useState(false);
  const dirty = useRef(false);
  const notify = useCallback((message: string, error = false) => setNotice({ message, error }), []);
  const onDirty = useCallback((value: boolean) => { dirty.current = value; }, []);
  const refreshJobs = useCallback(async () => { const result = await api<{ items: Job[] }>('jobs'); setJobs(result.items); setConnected(true); }, []);
  const updateJobList = useCallback(() => { void refreshJobs().catch(e => notify(e.message, true)); }, [refreshJobs, notify]);
  function openJobLog(id: string) {
    setJobId(id);
    setDrawer(true);
    updateJobList();
  }
  useEffect(() => {
    let cancelled = false; let timer: ReturnType<typeof setTimeout>;
    async function poll() { try { await api('health'); if (!cancelled) await refreshJobs(); } catch { if (!cancelled) setConnected(false); } if (!cancelled) timer = setTimeout(poll, 2000); }
    void poll(); return () => { cancelled = true; clearTimeout(timer); };
  }, [refreshJobs]);
  useEffect(() => { const handler = (e: BeforeUnloadEvent) => { if (dirty.current) { e.preventDefault(); e.returnValue = ''; } }; window.addEventListener('beforeunload', handler); return () => window.removeEventListener('beforeunload', handler); }, []);
  useEffect(() => { if (!jobId && jobs.length) setJobId(jobs[0].id); }, [jobs, jobId]);
  useEffect(() => {
    function syncPage(event: HashChangeEvent) {
      const [pageHash, query] = window.location.hash.split('?');
      const next = pageFromHash(pageHash);
      if (!next) return;
      if (dirty.current && !window.confirm('未保存の編集を破棄して画面を切り替えますか？')) {
        window.history.replaceState(null, '', event.oldURL);
        return;
      }
      dirty.current = false;
      setPage(next);
      setDrawer(false);
      setCapture(next === 'live' && new URLSearchParams(query).get('view') === 'capture');
    }
    window.addEventListener('hashchange', syncPage);
    return () => window.removeEventListener('hashchange', syncPage);
  }, []);
  const active = jobs.filter(job => ACTIVE.has(job.status));
  function navigate(next: Page, assetId = '') {
    if (next === page || (dirty.current && !window.confirm('未保存の編集を破棄して画面を切り替えますか？'))) return;
    dirty.current = false;
    let hash = '#' + next;
    if (next === 'calibration') hash = calibrationHash(assetId);
    if (next === 'map' && assetId) hash += '?' + (assetId === 'sample:odaiba'
      ? new URLSearchParams({sample: 'odaiba'}) : new URLSearchParams({map: assetId, line: 'raceline'}));
    window.history.replaceState(null, '', hash);
    setPage(next); setDrawer(false); setCapture(false);
  }
  function showCapture(next: boolean) {
    window.history.replaceState(null, '', next ? '#live?view=capture' : '#live');
    setDrawer(false);
    setCapture(next);
  }
  const selectedJob = jobs.find(job => job.id === jobId);
  const received = receivedBag(selectedJob);
  const evaluation = EVALUATIONS.find(item => item.id === page);
  const current = PAGES.find(item => item.id === (evaluation ? 'closed-loop' : page))!;
  return <div className={'app-shell' + (page === 'live' && capture ? ' live-capture-shell' : '')}><aside className="app-sidebar"><a className="brand" href="#" onClick={e => { e.preventDefault(); navigate('map'); }} aria-label="ATD Studio ホーム"><span className="brand-mark">A<span/></span><span>ATD<small>STUDIO</small></span></a><div className="workspace-label">WORKSPACE</div><nav aria-label="メインメニュー">{PAGES.map(item => <button key={item.id} aria-label={item.title + ' ' + item.subtitle} title={item.title} className={current.id === item.id ? 'active' : ''} onClick={() => { if (item.id !== current.id) navigate(item.id); }}><Icon name={item.icon} size={20}/><span>{item.title}<small>{item.subtitle}</small></span>{current.id === item.id && <i/>}</button>)}</nav><div className="sidebar-bottom"><div className="host-card"><span className="host-icon"><Icon name="folder" size={18}/></span><div><strong>Notebook</strong><small>ローカルワークスペース</small></div><span className="status-dot"/></div><p>BUILD · OBSERVE · REFINE</p></div></aside>
    <main className="main-shell"><header className="app-header"><div className="breadcrumb">ワークスペース<Icon name="chevron" size={14}/><strong>{current.title}</strong></div><div className="header-right"><span className={'connection-indicator ' + (connected ? 'online' : '')}><i/>{connected ? 'ローカル接続' : '接続を確認中'}</span><span className="header-separator"/><span className="local-label">ATD / DEVELOPMENT</span></div></header>
      {evaluation && <nav className="evaluation-nav" aria-label="走行評価の画面">
        {EVALUATIONS.map(item => <a key={item.id} href={'#' + item.id} aria-current={page === item.id ? 'page' : undefined}>{item.title}</a>)}
      </nav>}
      {!connected && <div className="backend-banner"><Icon name="info" size={16}/>ローカルサーバーに接続できません。起動状態を確認してください。</div>}
      <div className="page-container">
        <Suspense fallback={<p role="status">画面を読み込んでいます…</p>}>
          {page === 'map' && <MapWorkspace notify={notify} onDirty={onDirty} onJob={updateJobList} jobs={jobs} onTransfer={() => navigate('transfer')}/>}
          {page === 'transfer' && <TransferWorkspace notify={notify} onJob={updateJobList} jobs={jobs} onCalibration={bag => navigate('calibration', bag)}/>}
          {page === 'localization' && <LocalizationWorkspace jobs={jobs} onLog={openJobLog}/>}
          {page === 'live' && <LiveWorkspace notify={notify} capture={capture} onCapture={showCapture}/>}
          {page === 'input' && <InputWorkspace notify={notify} onDirty={onDirty} jobs={jobs} onLog={openJobLog}/>}
          {page === 'calibration' && <CalibrationWorkspace jobs={jobs} onLog={openJobLog} onTransfer={() => navigate('transfer')} onOpenMap={id => navigate('map', id)}/>}
          {page === 'esc' && <EscTrialWorkspace/>}
          {page === 'closed-loop' && <ClosedLoopWorkspace/>}
          {page === 'replay' && <ReplayErrorsWorkspace/>}
          {page === 'simulation' && <SimulationWorkspace jobs={jobs} onOpenMap={id => navigate('map', id)} onLog={openJobLog}/>}
          {page === 'vehicle' && <VehicleWorkspace notify={notify} onDirty={onDirty}/>}
        </Suspense>
      </div>
      <footer className="app-footer"><span><i className={connected ? 'online' : ''}/>{connected ? 'ワークスペース接続中' : 'ローカルサーバー未接続'}</span><span>ATD Studio <b>v0.1</b></span></footer>
    </main>
    {notice && <div className={'toast ' + (notice.error ? 'error' : '')} role={notice.error ? 'alert' : 'status'}><Icon name={notice.error ? 'info' : 'check'} size={19}/><span>{notice.message}</span><button className="icon-button" aria-label="通知を閉じる" onClick={() => setNotice(null)}><Icon name="close" size={16}/></button></div>}
    <button className={'jobs-trigger ' + (active.length ? 'running' : '')} onClick={() => setDrawer(!drawer)} aria-expanded={drawer}><Icon name="list" size={18}/><span>ジョブ</span><b>{active.length || jobs.length}</b><Icon name={drawer ? 'close' : 'chevron'} size={15}/></button>
    {drawer && <section className="jobs-drawer" aria-label="ジョブとログ"><header><div><span className="eyebrow">ACTIVITY</span><h3>ジョブとログ</h3></div><div className="button-row"><button className="button small" onClick={() => setManageHistory(true)}>履歴を整理</button><button className="icon-button" aria-label="ジョブを閉じる" onClick={() => setDrawer(false)}><Icon name="close"/></button></div></header><div className="jobs-layout"><div className="jobs-list">{jobs.map(job => <button key={job.id} className={jobId === job.id ? 'selected' : ''} onClick={() => setJobId(job.id)}><span className={'job-state ' + job.status}>{job.cancel_requested && ACTIVE.has(job.status) ? '停止要求中' : STATUS[job.status] || job.status}</span><strong>{job.title || job.kind || job.id}</strong><small>{job.message || job.id}</small></button>)}{!jobs.length && <p className="panel-empty">実行した処理はここに表示されます。</p>}</div><JobLog key={jobId} jobId={jobId} title={selectedJob?.title || '実行ログ'}>
        {received && page !== 'calibration' && <button className="button small" onClick={() => navigate('calibration', received)}>この bag を車両校正へ</button>}
        {selectedJob && ACTIVE.has(selectedJob.status) && <button className="button small" disabled={!!selectedJob.cancel_requested} onClick={async () => {
          try { await api(`jobs/${encodeURIComponent(jobId)}/cancel`, {}); await refreshJobs(); notify('停止を要求しました'); }
          catch (e) { notify((e as Error).message, true); }
        }}>{selectedJob.cancel_requested ? '停止要求中' : '停止を要求'}</button>}
      </JobLog></div></section>}
    {manageHistory && <Suspense fallback={<p className="toast" role="status">ジョブ履歴を読み込んでいます…</p>}><JobHistory onClose={() => setManageHistory(false)} onDeleted={(ids, count) => {
      const removed = new Set(ids);
      setJobs(current => current.filter(job => !removed.has(job.id)));
      if (removed.has(jobId)) setJobId('');
      setManageHistory(false);
      notify(`ジョブ履歴を ${count} 件削除しました`);
      void refreshJobs().catch(e => notify(e.message, true));
    }}/></Suspense>}
  </div>;
}
