import { useCallback, useEffect, useRef, useState } from 'react';
import { api } from './api';
import type { Job } from './types';
import { JobLog } from './JobLog';
import { Icon } from './ui';
import { ACTIVE, STATUS } from './job_status';
import { calibrationHash, receivedBag } from './calibration';
import { CalibrationWorkspace } from './CalibrationWorkspace';
import { EscTrialWorkspace } from './EscTrialWorkspace';
import { LocalizationWorkspace } from './LocalizationWorkspace';
import { SimulationWorkspace } from './SimulationWorkspace';
import { MapWorkspace } from './MapWorkspace';
import { TransferWorkspace } from './TransferWorkspace';
import { LiveWorkspace } from './LiveWorkspace';
import { InputWorkspace } from './InputWorkspace';
import { VehicleWorkspace } from './VehicleWorkspace';

type Page = 'map' | 'transfer' | 'live' | 'input' | 'vehicle' | 'calibration' | 'localization' | 'esc' | 'simulation';
const PAGES: { id: Page; icon: string; title: string; subtitle: string }[] = [
  { id: 'map', icon: 'map', title: '地図とライン', subtitle: '地図作成・HDMap編集' },
  { id: 'transfer', icon: 'transfer', title: 'データ転送', subtitle: 'bag受信・地図送信' },
  { id: 'calibration', icon: 'chart', title: '車両校正', subtitle: '速度・加速度・操舵応答' },
  { id: 'localization', icon: 'map', title: '自己位置検証', subtitle: '別走行の軌跡・追跡喪失' },
  { id: 'simulation', icon: 'chart', title: 'シミュレーター', subtitle: '先読み距離と走行軌跡' },
  { id: 'live', icon: 'live', title: 'ライブ', subtitle: '自動走行・自己位置' },
  { id: 'input', icon: 'settings', title: 'コントローラ', subtitle: '入力監視・配置調整' },
  { id: 'vehicle', icon: 'settings', title: '車両設定', subtitle: '実測寸法・センサ配置' },
  { id: 'esc', icon: 'controller', title: 'ESC 試験', subtitle: '試験手順・状態監視' },
];
export function App() {
  const [page, setPage] = useState<Page>(() => PAGES.find(item => '#' + item.id === window.location.hash.split('?')[0])?.id ?? 'map'), [connected, setConnected] = useState(false), [notice, setNotice] = useState<{ message: string; error: boolean } | null>(null);
  const [capture, setCapture] = useState(() =>
    new URLSearchParams(window.location.hash.split('?')[1]).get('view') === 'capture');
  const [jobs, setJobs] = useState<Job[]>([]), [drawer, setDrawer] = useState(false), [jobId, setJobId] = useState('');
  const dirty = useRef(false);
  const notify = useCallback((message: string, error = false) => setNotice({ message, error }), []);
  const onDirty = useCallback((value: boolean) => { dirty.current = value; }, []);
  const refreshJobs = useCallback(async () => { const result = await api<{ items: Job[] }>('jobs'); setJobs(result.items); setConnected(true); }, []);
  const onJob = useCallback((id?: string) => { if (id) setJobId(id); setDrawer(true); void refreshJobs().catch(e => notify(e.message, true)); }, [refreshJobs, notify]);
  useEffect(() => {
    let cancelled = false; let timer: ReturnType<typeof setTimeout>;
    async function poll() { try { await api('health'); if (!cancelled) await refreshJobs(); } catch { if (!cancelled) setConnected(false); } if (!cancelled) timer = setTimeout(poll, 2000); }
    void poll(); return () => { cancelled = true; clearTimeout(timer); };
  }, [refreshJobs]);
  useEffect(() => { const handler = (e: BeforeUnloadEvent) => { if (dirty.current) { e.preventDefault(); e.returnValue = ''; } }; window.addEventListener('beforeunload', handler); return () => window.removeEventListener('beforeunload', handler); }, []);
  useEffect(() => { if (!jobId && jobs.length) setJobId(jobs[0].id); }, [jobs, jobId]);
  useEffect(() => {
    function syncCapture(event: HashChangeEvent) {
      const [pageHash, query] = window.location.hash.split('?');
      const previous = new URL(event.oldURL).hash.split('?')[0];
      if (pageHash !== '#live' && previous !== '#live') return;
      const next = PAGES.find(item => '#' + item.id === (pageHash || '#map'));
      if (!next) return;
      if (dirty.current && !window.confirm('未保存の編集を破棄して画面を切り替えますか？')) {
        window.history.replaceState(null, '', event.oldURL);
        return;
      }
      dirty.current = false;
      setPage(next.id);
      setDrawer(false);
      setCapture(next.id === 'live' && new URLSearchParams(query).get('view') === 'capture');
    }
    window.addEventListener('hashchange', syncCapture);
    return () => window.removeEventListener('hashchange', syncCapture);
  }, []);
  const active = jobs.filter(job => ACTIVE.has(job.status));
  function navigate(next: Page, assetId = '') {
    if (next === page || (dirty.current && !window.confirm('未保存の編集を破棄して画面を切り替えますか？'))) return;
    dirty.current = false;
    let hash = '#' + next;
    if (next === 'calibration') hash = calibrationHash(assetId);
    if (next === 'map' && assetId) hash += '?' + new URLSearchParams({map: assetId, line: 'raceline'});
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
  const current = PAGES.find(item => item.id === page)!;
  return <div className={'app-shell' + (page === 'live' && capture ? ' live-capture-shell' : '')}><aside className="app-sidebar"><a className="brand" href="#" onClick={e => { e.preventDefault(); navigate('map'); }} aria-label="ATD Studio ホーム"><span className="brand-mark">A<span/></span><span>ATD<small>STUDIO</small></span></a><div className="workspace-label">WORKSPACE</div><nav aria-label="メインメニュー">{PAGES.map(item => <button key={item.id} aria-label={item.title + ' ' + item.subtitle} title={item.title} className={page === item.id ? 'active' : ''} onClick={() => navigate(item.id)}><Icon name={item.icon} size={20}/><span>{item.title}<small>{item.subtitle}</small></span>{page === item.id && <i/>}</button>)}</nav><div className="sidebar-bottom"><div className="host-card"><span className="host-icon"><Icon name="folder" size={18}/></span><div><strong>Notebook</strong><small>ローカルワークスペース</small></div><span className="status-dot"/></div><p>BUILD · OBSERVE · REFINE</p></div></aside>
    <main className="main-shell"><header className="app-header"><div className="breadcrumb">ワークスペース<Icon name="chevron" size={14}/><strong>{current.title}</strong></div><div className="header-right"><span className={'connection-indicator ' + (connected ? 'online' : '')}><i/>{connected ? 'ローカル接続' : '接続を確認中'}</span><span className="header-separator"/><span className="local-label">ATD / DEVELOPMENT</span></div></header>
      {!connected && <div className="backend-banner"><Icon name="info" size={16}/>ローカルサーバーに接続できません。起動状態を確認してください。</div>}
      <div className="page-container">{page === 'map' && <MapWorkspace notify={notify} onDirty={onDirty} onJob={onJob} jobs={jobs} onTransfer={() => navigate('transfer')}/>} {page === 'transfer' && <TransferWorkspace notify={notify} onJob={onJob} jobs={jobs} onCalibration={bag => navigate('calibration', bag)}/>} {page === 'localization' && <LocalizationWorkspace jobs={jobs} onLog={onJob}/>} {page === 'live' && <LiveWorkspace notify={notify} capture={capture} onCapture={showCapture}/>} {page === 'input' && <InputWorkspace notify={notify} onDirty={onDirty} jobs={jobs} onLog={onJob}/>} {page === 'calibration' && <CalibrationWorkspace jobs={jobs} onLog={onJob} onTransfer={() => navigate('transfer')} onOpenMap={id => navigate('map', id)}/>} {page === 'esc' && <EscTrialWorkspace/>} {page === 'simulation' && <SimulationWorkspace jobs={jobs} onOpenMap={id => navigate('map', id)} onLog={onJob}/>} {page === 'vehicle' && <VehicleWorkspace notify={notify} onDirty={onDirty}/>}</div>
      <footer className="app-footer"><span><i className={connected ? 'online' : ''}/>{connected ? 'ワークスペース接続中' : 'ローカルサーバー未接続'}</span><span>ATD Studio <b>v0.1</b></span></footer>
    </main>
    {notice && <div className={'toast ' + (notice.error ? 'error' : '')} role={notice.error ? 'alert' : 'status'}><Icon name={notice.error ? 'info' : 'check'} size={19}/><span>{notice.message}</span><button className="icon-button" aria-label="通知を閉じる" onClick={() => setNotice(null)}><Icon name="close" size={16}/></button></div>}
    <button className={'jobs-trigger ' + (active.length ? 'running' : '')} onClick={() => setDrawer(!drawer)} aria-expanded={drawer}><Icon name="list" size={18}/><span>ジョブ</span><b>{active.length || jobs.length}</b><Icon name={drawer ? 'close' : 'chevron'} size={15}/></button>
    {drawer && <section className="jobs-drawer" aria-label="ジョブとログ"><header><div><span className="eyebrow">ACTIVITY</span><h3>ジョブとログ</h3></div><button className="icon-button" aria-label="ジョブを閉じる" onClick={() => setDrawer(false)}><Icon name="close"/></button></header><div className="jobs-layout"><div className="jobs-list">{jobs.map(job => <button key={job.id} className={jobId === job.id ? 'selected' : ''} onClick={() => setJobId(job.id)}><span className={'job-state ' + job.status}>{job.cancel_requested && ACTIVE.has(job.status) ? '停止要求中' : STATUS[job.status] || job.status}</span><strong>{job.title || job.kind || job.id}</strong><small>{job.message || job.id}</small></button>)}{!jobs.length && <p className="panel-empty">実行した処理はここに表示されます。</p>}</div><JobLog key={jobId} jobId={jobId} title={selectedJob?.title || '実行ログ'}>
        {received && page !== 'calibration' && <button className="button small" onClick={() => navigate('calibration', received)}>この bag を車両校正へ</button>}
        {selectedJob && ACTIVE.has(selectedJob.status) && <button className="button small" disabled={!!selectedJob.cancel_requested} onClick={async () => {
          try { await api(`jobs/${encodeURIComponent(jobId)}/cancel`, {}); await refreshJobs(); notify('停止を要求しました'); }
          catch (e) { notify((e as Error).message, true); }
        }}>{selectedJob.cancel_requested ? '停止要求中' : '停止を要求'}</button>}
      </JobLog></div></section>}
  </div>;
}
