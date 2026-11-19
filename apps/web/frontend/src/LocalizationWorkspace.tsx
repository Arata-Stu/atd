import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { ACTIVE, STATUS } from './job_status';
import { LocalizationResult } from './LocalizationResult';
import { OUTCOMES } from './localization_validation';
import type { ValidationOptions, ValidationResult, ValidationRun } from './localization_validation';
import type { Job } from './types';
import { Field, Icon } from './ui';

export function LocalizationWorkspace({jobs, onLog}: {jobs: Job[]; onLog: (id: string) => void}) {
  const [mapId, setMapId] = useState(''), [bag, setBag] = useState(''), [engine, setEngine] = useState(''), [rate, setRate] = useState('1');
  const [options, setOptions] = useState<{map: string; data: ValidationOptions} | null>(null);
  const [optionsError, setOptionsError] = useState(''), [actionError, setActionError] = useState('');
  const [submitted, setSubmitted] = useState<Job | null>(null), [pending, setPending] = useState(false), [stopping, setStopping] = useState('');
  const [runs, setRuns] = useState<ValidationRun[] | null>(null), [runId, setRunId] = useState(''), [historyError, setHistoryError] = useState('');
  const [result, setResult] = useState<ValidationResult | null>(null), [resultError, setResultError] = useState('');
  const [reload, setReload] = useState(0);
  const followJob = useRef('');
  const query = mapId ? '?'+new URLSearchParams({map_id: mapId}) : '';
  const data = options?.map === mapId && !optionsError ? options.data : null;
  const maps = options?.data.maps || [];
  const map = data?.maps.find(m => m.id === mapId);
  const selectedBag = data?.bags.find(b => b.id === bag);
  const validationJobs = jobs.filter(j => j.action === 'localization_validation');
  const submittedJob = validationJobs.find(j => j.id === submitted?.id) || submitted;
  const active = validationJobs.find(j => ACTIVE.has(j.status)) || (submittedJob && ACTIVE.has(submittedJob.status) ? submittedJob : null);
  const job = active || (submittedJob && (!mapId || submittedJob.resources?.includes('map:' + mapId)) ? submittedJob : null) ||
    validationJobs.find(j => !mapId || j.resources?.includes('map:' + mapId));
  const lastFinished = validationJobs.find(j => !ACTIVE.has(j.status));
  const updatedRun = lastFinished?.id || '';
  const choice = runId || runs?.[0]?.id || '';
  const current = result?.id === choice ? result : null;
  const speed = Number(rate);
  const canStart = data?.available && map?.ready && selectedBag?.eligible && Number.isFinite(speed) && speed >= .05 && speed <= 4 &&
    (!engine || data.engine_dirs.includes(engine)) && !pending && !active;

  useEffect(() => {
    let live = true;
    setOptionsError('');
    api<ValidationOptions>('localization/options' + query).then(data => {
      if (live) setOptions({map: mapId, data});
    }).catch(e => { if (live) setOptionsError(e.message); });
    return () => { live = false; };
  }, [mapId, query, reload]);

  useEffect(() => {
    let live = true;
    setHistoryError('');
    api<{items: ValidationRun[]}>('localization/runs' + query).then(({items}) => {
      if (!live) return;
      setRuns(items);
      setRunId(id => items.some(run => run.id === id) ? id : '');
    }).catch(e => { if (live) { setRuns(null); setHistoryError(e.message); } });
    return () => { live = false; };
  }, [query, reload, updatedRun]);

  useEffect(() => {
    if (submittedJob && followJob.current === submittedJob.id && !ACTIVE.has(submittedJob.status)) {
      followJob.current = '';
      if (typeof submittedJob.result?.run_id === 'string') setRunId(submittedJob.result.run_id);
    }
  }, [submittedJob]);

  useEffect(() => {
    let live = true;
    setResult(null); setResultError('');
    if (choice) api<ValidationResult>('localization/runs/' + encodeURIComponent(choice)).then(run => {
      if (live) {
        if (run.id !== choice) setResultError('要求した検証結果と取得した実行 ID が異なります。');
        else setResult(run);
      }
    }).catch(e => { if (live) setResultError(e.message); });
    return () => { live = false; };
  }, [choice, reload]);

  function selectMap(id: string) {
    followJob.current = '';
    setMapId(id); setBag(''); setEngine(''); setRunId(''); setRuns(null); setResult(null); setActionError('');
  }

  async function start() {
    if (!canStart || !map) return;
    setActionError(''); setPending(true);
    try {
      const job = await api<Job>('localization/runs', {map_id: map.id, bag, revision: map.revision, rate: speed,
        ...(engine ? {engine_dir: engine} : {})});
      followJob.current = job.id;
      setSubmitted(job); setStopping('');
    } catch (e) { setActionError((e as Error).message); }
    finally { setPending(false); }
  }

  async function cancel(id: string) {
    setStopping(id); setActionError('');
    try { await api('jobs/' + encodeURIComponent(id) + '/cancel', {}); }
    catch (e) { setStopping(''); setActionError((e as Error).message); }
  }

  return <div className="content-page validation-page">
    <div className="page-intro"><div><h2>自己位置検証</h2><p>保存地図に別の走行 bag を位置合わせし、推定軌跡・追跡の途絶と、走行ラインに沿った前方軌道を確認します。</p></div>
      <button className="button" onClick={() => setReload(n => n + 1)}><Icon name="refresh" size={16}/>一覧を更新</button></div>
    <section className="content-card"><div className="card-heading"><h3>検証する走行</h3></div>
      <form className="validation-form" onSubmit={e => { e.preventDefault(); void start(); }}>
        <div className="validation-fields">
          <Field label="保存地図"><select value={mapId} disabled={pending} onChange={e => selectMap(e.target.value)} required>
            <option value="">{maps.length ? '地図を選択してください' : options ? '保存地図がありません' : '地図を読み込み中…'}</option>
            {maps.map(m => <option value={m.id} key={m.id}>{m.id}{m.title && m.title !== m.id ? ' · ' + m.title : ''}</option>)}
          </select></Field>
          <Field label="検証用 bag"><select value={bag} onChange={e => { setBag(e.target.value); setActionError(''); }} disabled={!map?.ready || pending} required>
            <option value="">{!mapId ? '先に地図を選択してください' : !data ? '読み込み中…' : data.bags.some(b => b.eligible) ? '別の走行 bag を選択してください' : '利用できる別走行 bag がありません'}</option>
            {data?.bags.map(b => <option key={b.id} value={b.id} disabled={!b.eligible}>{b.id}{!b.eligible && b.issue ? ' · ' + b.issue : ''}</option>)}
          </select></Field>
          <button className="button primary" disabled={!canStart}><Icon name="play" size={16}/>{pending ? '受け付け中…' : '検証を開始'}</button>
        </div>
        {optionsError && <p className="inline-note" role="alert">{optionsError}</p>}
        {data && !data.available && <p className="inline-note">{data.unavailable_reason || 'Notebook で検証を実行できません。'}</p>}
        {map && <p className="small-note">地図作成 bag · {map.source_bag || '不明'}</p>}
        {map?.issue && <p className="inline-note">{map.issue}</p>}
        {map?.ready && <details className="validation-options"><summary>再生設定</summary>
          <div className="validation-option-grid"><Field label="再生倍率"><input type="number" min=".05" max="4" step="any" required value={rate} onChange={e => setRate(e.target.value)}/></Field>
            <Field label="Notebook 用 VGL engine"><select value={engine} onChange={e => setEngine(e.target.value)}><option value="">自動選択</option>
              {data!.engine_dirs.map(path => <option key={path} value={path}>{path}</option>)}</select></Field></div>
        </details>}
      </form>
      {job && <div className="validation-job"><div><strong role="status">{job.cancel_requested && ACTIVE.has(job.status) ? '停止要求中' : STATUS[job.status] || job.status}</strong>
        <span>{job.title}</span>{job.message && <small>{job.message}</small>}</div>
        <div className="button-row"><button className="button small" onClick={() => onLog(job.id)}><Icon name="list" size={14}/>実行ログ</button>
          {ACTIVE.has(job.status) && <button className="button small" disabled={stopping === job.id || job.cancel_requested}
            onClick={() => void cancel(job.id)}>停止を要求</button>}</div></div>}
      {actionError && <p className="inline-note" role="alert">{actionError}</p>}
    </section>
    <div className="validation-history">
      <Field label="検証履歴"><select value={choice} disabled={!runs?.length} onChange={e => { followJob.current = ''; setRunId(e.target.value); }}>
        {!runs?.length && <option value="">{historyError ? '履歴を読み込めません' : runs ? '保存された検証結果がありません' : '履歴を読み込み中…'}</option>}
        {runs?.map(run => <option key={run.id} value={run.id}>{run.saved_at ? new Date(run.saved_at * 1000).toLocaleString('ja-JP') + ' · ' : ''}{run.map_id} · {run.bag} · {OUTCOMES[run.outcome] || run.outcome} · {run.id}</option>)}
      </select></Field>
    </div>
    {historyError && <p className="inline-note" role="alert">{historyError}</p>}
    {resultError && <p className="inline-note" role="alert">{resultError}</p>}
    {current ? <LocalizationResult key={current.id} result={current} jobs={jobs} onLog={onLog}/> : choice && !resultError && <p role="status">検証結果を読み込み中…</p>}
  </div>;
}
