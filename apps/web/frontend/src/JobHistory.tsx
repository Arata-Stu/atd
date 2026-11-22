import { useEffect, useState } from 'react';
import { api } from './api';
import { historyBoundary, STATUS } from './job_status';
import { Field, Modal } from './ui';
import './job_history.css';

type Entry = { id: string; title: string; status: string; started: number };
type History = { items: Entry[]; protected: number };
type Mode = 'selected' | 'range' | 'all';
const modes: Record<Mode, string> = { selected: '個別選択', range: '日時範囲', all: '全件' };
const pageSize = 100;

export default function JobHistory({ onClose, onDeleted }: {
  onClose: () => void; onDeleted: (ids: string[], count: number) => void;
}) {
  const [history, setHistory] = useState<History | null>(null);
  const [mode, setMode] = useState<Mode>('selected');
  const [shown, setShown] = useState(pageSize);
  const [selection, setSelection] = useState<Set<string>>(new Set());
  const [fromDate, setFromDate] = useState(''), [fromTime, setFromTime] = useState('');
  const [toDate, setToDate] = useState(''), [toTime, setToTime] = useState('');
  const [busy, setBusy] = useState(false), [error, setError] = useState('');

  useEffect(() => {
    const request = new AbortController();
    api<History>('jobs/history', undefined, undefined, request.signal)
      .then(result => { if (!request.signal.aborted) setHistory(result); })
      .catch(e => { if (!request.signal.aborted) setError(e.message); });
    return () => request.abort();
  }, []);

  const start = historyBoundary(fromDate, fromTime, false), end = historyBoundary(toDate, toTime, true);
  const invalidRange = !!(fromDate || toDate) && !(start < end);
  const items = history?.items ?? [];
  const targets = items.filter(job => {
    if (mode === 'selected') return selection.has(job.id);
    if (mode === 'all') return true;
    return (fromDate || toDate) && job.started >= start && job.started < end;
  });
  const preview = mode === 'selected' ? items : targets;
  function toggle(id: string) {
    setSelection(previous => {
      const next = new Set(previous);
      if (next.has(id)) next.delete(id); else next.add(id);
      return next;
    });
  }
  async function remove() {
    if (!targets.length || busy || !window.confirm(`ジョブ履歴と実行ログを ${targets.length} 件削除しますか？`)) return;
    setBusy(true);
    setError('');
    const ids = targets.map(job => job.id);
    try {
      const result = await api<{deleted: number}>('jobs/history/delete', { ids });
      onDeleted(ids, result.deleted);
    } catch (e) { setError((e as Error).message); setBusy(false); }
  }
  return <Modal title="ジョブ履歴の整理" subtitle="終了したジョブの履歴とログを削除します。地図や解析結果は保持されます。" onClose={() => { if (!busy) onClose(); }}>
    <div className="segmented" aria-label="削除対象の指定方法">
      {(Object.keys(modes) as Mode[]).map(key => <button key={key} disabled={busy} className={mode === key ? 'active' : ''} aria-pressed={mode === key} onClick={() => { setMode(key); setShown(pageSize); }}>{modes[key]}</button>)}
    </div>
    {mode === 'range' && <fieldset className="job-history-range" disabled={busy}>
      <legend>ジョブの開始日時（Notebook の現地時刻）</legend>
      <Field label="この日から"><input type="date" value={fromDate} onChange={e => setFromDate(e.target.value)}/></Field>
      <Field label="開始時刻（任意）"><input type="time" disabled={!fromDate} value={fromDate ? fromTime : ''} onChange={e => setFromTime(e.target.value)}/></Field>
      <Field label="この日まで"><input type="date" value={toDate} onChange={e => setToDate(e.target.value)}/></Field>
      <Field label="終了時刻（任意）"><input type="time" disabled={!toDate} value={toDate ? toTime : ''} onChange={e => setToTime(e.target.value)}/></Field>
    </fieldset>}
    {invalidRange && mode === 'range' && <p role="alert">終了日時は開始日時以降を選んでください。</p>}
    {history && <p className="job-history-count">削除対象 <strong>{targets.length} 件</strong>{history.protected > 0 && <span> ／ 実行中・要確認 {history.protected} 件を除外</span>}</p>}
    <div className="job-history-list">
      {preview.slice(0, shown).map(job => <label key={job.id} className="job-history-entry">
        {mode === 'selected' && <input type="checkbox" checked={selection.has(job.id)} disabled={busy} onChange={() => toggle(job.id)}/>}
        <span><strong>{job.title}</strong><small>{new Date(job.started * 1000).toLocaleString('ja-JP', {hour12: false})} · {STATUS[job.status] || job.status} · {job.id}</small></span>
      </label>)}
      {preview.length > shown && <button className="button small" onClick={() => setShown(shown + pageSize)}>次の {Math.min(pageSize, preview.length - shown)} 件を表示</button>}
      {!history && !error && <p role="status">履歴を読み込んでいます…</p>}
      {history && !items.length && <p>削除できる履歴はありません。</p>}
    </div>
    {error && <p role="alert">{error}</p>}
    <div className="modal-actions"><button className="button danger" disabled={busy || !targets.length} onClick={remove}>{busy ? '削除中…' : `${targets.length} 件を削除`}</button></div>
  </Modal>;
}
