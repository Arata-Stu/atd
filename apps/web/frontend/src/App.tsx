import { useCallback, useEffect, useRef, useState } from 'react';
import { api } from './api';
import type { Job } from './types';
import { Icon } from './ui';
import { MapWorkspace } from './MapWorkspace';
import { TransferWorkspace } from './TransferWorkspace';
import { LiveWorkspace } from './LiveWorkspace';
import { InputWorkspace } from './InputWorkspace';
import { VehicleWorkspace } from './VehicleWorkspace';

type Page = 'map' | 'transfer' | 'live' | 'input' | 'vehicle';
const PAGES: { id: Page; icon: string; title: string; subtitle: string }[] = [
  { id: 'map', icon: 'map', title: '地図とライン', subtitle: '地図作成・HDMap編集' },
  { id: 'transfer', icon: 'transfer', title: 'データ転送', subtitle: 'bag受信・地図送信' },
  { id: 'live', icon: 'live', title: 'ライブ', subtitle: '自己位置・パラメータ' },
  { id: 'input', icon: 'settings', title: 'コントローラ', subtitle: '入力監視・配置調整' },
  { id: 'vehicle', icon: 'settings', title: '車両設定', subtitle: '実測寸法・センサ配置' },
];
const ACTIVE = new Set(['starting', 'queued', 'running', 'stopping', 'cancel_requested']);
const STATUS: Record<string, string> = { starting: '開始中', queued: '待機中', running: '実行中', stopping: '停止中', cancel_requested: '停止要求中', succeeded: '完了', completed: '完了', failed: '失敗', cancelled: '中止', canceled: '中止', interrupted: '中断', orphaned: '要確認' };
export function App() {
  const [page, setPage] = useState<Page>('map'), [connected, setConnected] = useState(false), [notice, setNotice] = useState<{ message: string; error: boolean } | null>(null);
  const [jobs, setJobs] = useState<Job[]>([]), [drawer, setDrawer] = useState(false), [jobId, setJobId] = useState(''), [log, setLog] = useState('');
  const dirty = useRef(false), selectedJob = useRef('');
  const notify = useCallback((message: string, error = false) => setNotice({ message, error }), []);
  const onDirty = useCallback((value: boolean) => { dirty.current = value; }, []);
  const refreshJobs = useCallback(async () => { const result = await api<{ items: Job[] }>('jobs'); setJobs(result.items); setConnected(true); }, []);
  const onJob = useCallback(() => { setDrawer(true); void refreshJobs().catch(e => notify(e.message, true)); }, [refreshJobs, notify]);
  useEffect(() => {
    let cancelled = false; let timer: ReturnType<typeof setTimeout>;
    async function poll() { try { await api('health'); if (!cancelled) await refreshJobs(); } catch { if (!cancelled) setConnected(false); } if (!cancelled) timer = setTimeout(poll, 2000); }
    void poll(); return () => { cancelled = true; clearTimeout(timer); };
  }, [refreshJobs]);
  useEffect(() => { const handler = (e: BeforeUnloadEvent) => { if (dirty.current) { e.preventDefault(); e.returnValue = ''; } }; window.addEventListener('beforeunload', handler); return () => window.removeEventListener('beforeunload', handler); }, []);
  useEffect(() => { if (!jobId && jobs.length) setJobId(jobs[0].id); }, [jobs, jobId]);
  useEffect(() => {
    if (!drawer || !jobId) return; let cancelled = false; let timer: ReturnType<typeof setTimeout>; selectedJob.current = jobId;
    async function poll() { try { const result = await api<{ text: string }>(`jobs/${encodeURIComponent(jobId)}/log`); if (!cancelled) setLog(result.text); } catch (e) { if (!cancelled) setLog((e as Error).message); } if (!cancelled) timer = setTimeout(poll, 1800); }
    setLog(''); void poll(); return () => { cancelled = true; clearTimeout(timer); };
  }, [drawer, jobId]);
  const active = jobs.filter(job => ACTIVE.has(job.status));
  function navigate(next: Page) { if (next === page) return; if (dirty.current && !window.confirm('未保存の編集を破棄して画面を切り替えますか？')) return; dirty.current = false; setPage(next); }
  const current = PAGES.find(item => item.id === page)!;
  return <div className="app-shell"><aside className="app-sidebar"><a className="brand" href="#" onClick={e => { e.preventDefault(); navigate('map'); }} aria-label="ATD Studio ホーム"><span className="brand-mark">A<span/></span><span>ATD<small>STUDIO</small></span></a><div className="workspace-label">WORKSPACE</div><nav aria-label="メインメニュー">{PAGES.map(item => <button key={item.id} className={page === item.id ? 'active' : ''} onClick={() => navigate(item.id)}><Icon name={item.icon} size={20}/><span>{item.title}<small>{item.subtitle}</small></span>{page === item.id && <i/>}</button>)}</nav><div className="sidebar-bottom"><div className="host-card"><span className="host-icon"><Icon name="folder" size={18}/></span><div><strong>Notebook</strong><small>ローカルワークスペース</small></div><span className="status-dot"/></div><p>BUILD · OBSERVE · REFINE</p></div></aside>
    <main className="main-shell"><header className="app-header"><div className="breadcrumb">ワークスペース<Icon name="chevron" size={14}/><strong>{current.title}</strong></div><div className="header-right"><span className={'connection-indicator ' + (connected ? 'online' : '')}><i/>{connected ? 'ローカル接続' : '接続を確認中'}</span><span className="header-separator"/><span className="local-label">ATD / DEVELOPMENT</span></div></header>
      {!connected && <div className="backend-banner"><Icon name="info" size={16}/>ローカルサーバーに接続できません。起動状態を確認してください。</div>}
      <div className="page-container">{page === 'map' && <MapWorkspace notify={notify} onDirty={onDirty} onJob={onJob} jobs={jobs} onTransfer={() => navigate('transfer')}/>} {page === 'transfer' && <TransferWorkspace notify={notify} onJob={onJob}/>} {page === 'live' && <LiveWorkspace notify={notify}/>} {page === 'input' && <InputWorkspace notify={notify} onDirty={onDirty}/>} {page === 'vehicle' && <VehicleWorkspace notify={notify} onDirty={onDirty}/>}</div>
      <footer className="app-footer"><span><i className={connected ? 'online' : ''}/>{connected ? 'ワークスペース接続中' : 'ローカルサーバー未接続'}</span><span>ATD Studio <b>v0.1</b></span></footer>
    </main>
    {notice && <div className={'toast ' + (notice.error ? 'error' : '')} role={notice.error ? 'alert' : 'status'}><Icon name={notice.error ? 'info' : 'check'} size={19}/><span>{notice.message}</span><button className="icon-button" aria-label="通知を閉じる" onClick={() => setNotice(null)}><Icon name="close" size={16}/></button></div>}
    <button className={'jobs-trigger ' + (active.length ? 'running' : '')} onClick={() => setDrawer(!drawer)} aria-expanded={drawer}><Icon name="list" size={18}/><span>ジョブ</span><b>{active.length || jobs.length}</b><Icon name={drawer ? 'close' : 'chevron'} size={15}/></button>
    {drawer && <section className="jobs-drawer" aria-label="ジョブとログ"><header><div><span className="eyebrow">ACTIVITY</span><h3>ジョブとログ</h3></div><button className="icon-button" aria-label="ジョブを閉じる" onClick={() => setDrawer(false)}><Icon name="close"/></button></header><div className="jobs-layout"><div className="jobs-list">{jobs.map(job => <button key={job.id} className={jobId === job.id ? 'selected' : ''} onClick={() => setJobId(job.id)}><span className={'job-state ' + job.status}>{job.cancel_requested && ACTIVE.has(job.status) ? '停止要求中' : STATUS[job.status] || job.status}</span><strong>{job.title || job.kind || job.id}</strong><small>{job.message || job.id}</small></button>)}{!jobs.length && <p className="panel-empty">実行した処理はここに表示されます。</p>}</div><div className="job-log"><div><strong>{jobs.find(job => job.id === jobId)?.title || '実行ログ'}</strong>{jobs.find(job => job.id === jobId && ACTIVE.has(job.status)) && <button className="button small" disabled={!!jobs.find(job => job.id === jobId)?.cancel_requested} onClick={async () => { try { await api(`jobs/${encodeURIComponent(jobId)}/cancel`, {}); await refreshJobs(); notify('停止を要求しました'); } catch (e) { notify((e as Error).message, true); } }}>{jobs.find(job => job.id === jobId)?.cancel_requested ? '停止要求中' : '停止を要求'}</button>}</div><pre>{log || 'ログはまだありません。'}</pre></div></div></section>}
  </div>;
}
