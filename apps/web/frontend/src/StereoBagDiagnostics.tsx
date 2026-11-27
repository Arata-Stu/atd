import { useState } from 'react';
import { api } from './api';
import { ACTIVE, STATUS } from './job_status';
import type { Job } from './types';

interface Gap { after_s: number; interval_ms: number; estimated_missing: number }
interface Stream {
  topic: string; status: 'absent' | 'unsupported' | 'ready';
  valid: number; invalid: number; estimated_missing: number | null; gap_count: number;
  duplicates: number; backwards: number; longest_ms: number | null; gaps: Gap[];
}
interface Report {
  fps: number; streams: Stream[];
  pairs: { paired: number; left_only: number; right_only: number; tolerance_ms: number } | null;
}

export function StereoBagDiagnostics({bag, jobs, onLog}: {bag: string; jobs: Job[]; onLog: (id: string) => void}) {
  const [fps, setFps] = useState('90');
  const [submitted, setSubmitted] = useState<Job | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const latest = jobs.find(item => item.action === 'stereo_bag_diagnostics' && item.resources?.includes('record:' + bag));
  const job = (submitted && jobs.find(item => item.id === submitted.id)) ?? submitted ?? latest;
  const report = job?.status === 'succeeded' ? job.result as unknown as Report : null;
  const active = Boolean(job && ACTIVE.has(job.status));

  async function run() {
    setBusy(true);
    setError('');
    try { setSubmitted(await api<Job>('stereo-bag-diagnostics', {bag, fps: Number(fps)})); }
    catch (e) { setError((e as Error).message); }
    finally { setBusy(false); }
  }

  return <section className="stereo-diagnostics" aria-label="ステレオ欠落診断">
    <h4>ステレオ欠落診断</h4>
    <form className="drive-bag-choice" onSubmit={e => { e.preventDefault(); void run(); }}>
      <label htmlFor="stereo-fps">録画時のIR fps</label>
      <input id="stereo-fps" type="number" min="1" max="120" step="1" required
        value={fps} onChange={e => setFps(e.target.value)} disabled={busy || active}/>
      <button className="button small" disabled={busy || active}>{active ? '解析中…' : '欠落を調べる'}</button>
    </form>
    {error && <p role="alert">{error}</p>}
    {job && <p role="status">{STATUS[job.status] ?? job.status} {job.message}
      {' '}<button className="button small" onClick={() => onLog(job.id)}>ログ</button></p>}
    {report && <>
      <p className="small-note">{report.fps} fpsを基準に画像の時刻差から推定します。時刻補正や撮影周期の変化もギャップに反映されます。録画区間の両端は欠落推定の対象外です。</p>
      <div className="stereo-diagnostics-streams">
        {report.streams.map((stream, side) => <div key={stream.topic}>
          <h5>{side === 0 ? '左 IR' : '右 IR'}</h5>
          {stream.status === 'absent' ? <p>画像topicがbagにありません。</p> :
            stream.status === 'unsupported' ? <p>画像topicの型または保存形式に対応していません。</p> :
            <>
              <p>有効 {stream.valid.toLocaleString()} 件 · 無効時刻 {stream.invalid} 件</p>
              <p>推定欠落 {stream.estimated_missing?.toLocaleString() ?? '判定不可'} · ギャップ {stream.gap_count} 箇所
                {' · '}最大間隔 {stream.longest_ms?.toFixed(2) ?? '—'} ms</p>
              <p>重複時刻 {stream.duplicates} 件 · 時刻の逆行 {stream.backwards} 件</p>
              {stream.gaps.length > 0 && <details>
                <summary>ギャップの位置（先頭 {stream.gaps.length} 件）</summary>
                <table><thead><tr><th>最初のIR画像から</th><th>間隔</th><th>推定欠落</th></tr></thead>
                  <tbody>{stream.gaps.map((gap, index) => <tr key={index}><td>{gap.after_s.toFixed(3)} s の直後</td>
                    <td>{gap.interval_ms.toFixed(2)} ms</td><td>{gap.estimated_missing}</td></tr>)}</tbody>
                </table>
              </details>}
            </>}
        </div>)}
      </div>
      {report.pairs ? <p>ステレオペア {report.pairs.paired.toLocaleString()} 組 · 左のみ {report.pairs.left_only} 件
        {' · '}右のみ {report.pairs.right_only} 件（時刻差 ±{report.pairs.tolerance_ms.toFixed(2)} ms、録画端を含む）</p> :
        <p>左右どちらかに有効な時刻がなく、ペアを照合できません。</p>}
    </>}
  </section>;
}
