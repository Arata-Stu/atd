import { useEffect, useState } from 'react';
import { api } from './api';
import { DeleteAnalysisButton } from './DeleteAnalysisButton';
import { modeLabel, type Mode, type SimSettings } from './simulation';

export interface Circuit {
  id: string; name: string; lines: string[];
}
interface Attempt { trial: number; outcome: string; lap_times_s: number[]; reason: string }
interface Score {
  mode: string; trials: number; finishes: number; finish_rate: number; collisions: number;
  mean_lap_time_s: number | null; best_lap_time_s: number | null; mean_trial_rms_lateral_error_m?: number; runs: Attempt[];
}
interface Report {
  id: string;
  options: { line: string; source: string; laps: number; trials: number; settings: SimSettings };
  courses: { course_id: string; course_sha256: string; results: Score[] }[];
}
const controllerName = (mode: string) => Object.hasOwn(modeLabel, mode) ? modeLabel[mode as Mode] : mode;
const seconds = (s: number | null) => s == null ? '—' : s.toFixed(2);
const outcomes: Record<string, string> = {
  finished: '完走', collision: '境界に接触', tracking_lost: '追跡喪失', controller_error: '制御計算中断', timeout: '時間上限',
};

export function BenchmarkResults({ latest, circuits }: { latest: string; circuits: Circuit[] }) {
  const [reports, setReports] = useState<{ id: string; saved: number }[]>([]);
  const [key, setKey] = useState('');
  const [loaded, setLoaded] = useState(false);
  const [report, setReport] = useState<Report>(), [error, setError] = useState('');
  const [referenceKey, setReferenceKey] = useState(''), [reference, setReference] = useState<Report>();
  const [detail, setDetail] = useState('');
  useEffect(() => {
    const controller = new AbortController();
    api<{items: typeof reports}>('simulation/benchmarks', undefined, undefined, controller.signal)
      .then(data => {
        if (controller.signal.aborted) return;
        setReports(data.items); setLoaded(true);
        if (latest && data.items.some(item => item.id === latest)) setKey(latest);
        else setKey(current => data.items.some(item => item.id === current) ? current : data.items[0]?.id ?? '');
      }).catch(e => { if (!controller.signal.aborted) setError(e.message); });
    return () => controller.abort();
  }, [latest]);
  useEffect(() => {
    const controller = new AbortController();
    setReport(undefined); setDetail('');
    if (key) {
      setError('');
      api<Report>('simulation/benchmarks/' + key, undefined, undefined, controller.signal)
        .then(data => { if (!controller.signal.aborted) setReport(data); })
        .catch(e => { if (!controller.signal.aborted) setError(e.message); });
    }
    return () => controller.abort();
  }, [key]);
  useEffect(() => {
    setReference(undefined);
    if (!referenceKey || referenceKey === key) return;
    const controller = new AbortController();
    api<Report>('simulation/benchmarks/' + referenceKey, undefined, undefined, controller.signal)
      .then(data => { if (!controller.signal.aborted) setReference(data); })
      .catch(e => { if (!controller.signal.aborted) setError(e.message); });
    return () => controller.abort();
  }, [referenceKey, key]);
  const selected = report?.courses.find(c => c.course_id === detail);
  const title = (id: string) => circuits.find(c => c.id === id)?.name ?? id;
  const options = <>
    {reports.map(r => <option key={r.id} value={r.id}>{new Date(r.saved * 1000).toLocaleString('ja-JP')} · {r.id.slice(0,8)}</option>)}
  </>;
  function delta(current: number | null, other: number | null | undefined, digits=0, factor=1) {
    if (current == null || other == null) return null;
    const value = (current-other)*factor;
    return <small className="bench-delta">({value > 0 ? '+' : ''}{value.toFixed(digits)})</small>;
  }
  return <section className="sim-card"><div className="sim-section-title"><h2>コース別の評価結果</h2>
    {report && <span><a href={'/api/simulation/benchmarks/' + key + '/summary.csv'} download>CSV を保存</a> · <a href={'/api/simulation/benchmarks/' + key} download={'benchmark-' + key + '.json'}>全試行を JSON で保存</a></span>}
  </div>
    {loaded && !reports.length && <p className="sim-note">保存済みの評価結果はありません。</p>}
    {reports.length > 0 && <div className="bench-actions"><label className="bench-report-picker">評価履歴<select aria-label="評価履歴" value={key} onChange={e => setKey(e.target.value)}>{options}</select></label>
      {reports.length > 1 && <label className="bench-report-picker">差分の比較元<select aria-label="差分の比較元" value={referenceKey} onChange={e => setReferenceKey(e.target.value)}>
        <option value="">差分なし</option>{options}</select></label>}
      {reports.some(item => item.id === key) && <DeleteAnalysisButton key={key} path={'analysis-results/benchmark/' + encodeURIComponent(key)}
        detail={`ベンチマーク「${key}」の全コース・全試行の結果を削除しますか？`} onDeleted={() => {
          const remaining = reports.filter(item => item.id !== key);
          setReports(remaining); setKey(remaining[0]?.id ?? '');
          if (referenceKey === key) setReferenceKey('');
        }}/>}</div>}
    {error && <p role="alert" className="sim-error">{error}</p>}
    {report && <>
      <p className="sim-note">{report.courses.length} コース · {report.options.source === 'sample' ? '同梱 HDMap' : '保存済み HDMap'} · {report.options.line} · 各方式 {report.options.trials} 試行 × {report.options.laps} 周</p>
      <details className="sim-model"><summary>この評価で使った制御・遅延設定</summary>
        <p>固定 {report.options.settings.fixed_lookahead_m} m · 速度連動 {report.options.settings.min_lookahead_m} ～ {report.options.settings.max_lookahead_m} m、先読み {report.options.settings.lookahead_time_s} s</p>
        <p>操舵／スロットルのむだ時間 {report.options.settings.steering_delay_ms} / {report.options.settings.throttle_delay_ms} ms · 操舵／速度の時定数 {report.options.settings.steering_tau_s} / {report.options.settings.speed_tau_s} s</p>
      </details>
      <p className="sim-note">完走率は規定周回を終えた試行の割合です。衝突は車体がコース境界に接触した試行数、ラップタイムは完了した周の値です。</p>
      {reference && <p className="sim-note">括弧内は比較元からの差です。境界・ライン・周回数・初期条件が一致した行で表示します。完走率の差はパーセントポイントです。</p>}
      <div className="sim-table-wrap"><table><thead><tr><th>コース</th><th>方式</th><th>完走率</th><th>衝突</th><th>平均ラップ (s)</th><th>最速ラップ (s)</th><th>平均横ずれ RMS (cm)</th></tr></thead>
        <tbody>{report.courses.flatMap(course => course.results.map((s, i) => {
          const other = reference?.options.laps === report.options.laps && reference.options.trials === report.options.trials
            ? reference.courses.find(c => c.course_id === course.course_id && c.course_sha256 === course.course_sha256)?.results.find(r => r.mode === s.mode)
            : undefined;
          return <tr key={course.course_id + s.mode}>
          {i === 0 && <th rowSpan={course.results.length}><button className="bench-link" onClick={() => setDetail(course.course_id)}>{title(course.course_id)}</button></th>}
          <td>{controllerName(s.mode)}</td><td>{(100*s.finish_rate).toFixed(0)}% ({s.finishes}/{s.trials}){delta(s.finish_rate,other?.finish_rate,1,100)}</td>
          <td>{s.collisions}{delta(s.collisions,other?.collisions)}</td><td>{seconds(s.mean_lap_time_s)}{delta(s.mean_lap_time_s,other?.mean_lap_time_s,2)}</td>
          <td>{seconds(s.best_lap_time_s)}{delta(s.best_lap_time_s,other?.best_lap_time_s,2)}</td>
          <td>{s.mean_trial_rms_lateral_error_m == null ? '—' : (s.mean_trial_rms_lateral_error_m*100).toFixed(2)}
            {delta(s.mean_trial_rms_lateral_error_m ?? null,other?.mean_trial_rms_lateral_error_m,2,100)}</td>
        </tr>;}))}</tbody></table></div>
      {selected && <div className="sim-table-wrap" aria-live="polite"><h3>{title(selected.course_id)} の試行</h3><table><thead><tr><th>方式</th><th>試行</th><th>結果</th><th>各周 (s)</th></tr></thead>
        <tbody>{selected.results.flatMap(s => s.runs.map((r,i) => <tr key={s.mode + i}><th>{controllerName(s.mode)}</th><td>{i+1}</td><td>{outcomes[r.outcome] ?? r.outcome}{r.reason && ' · ' + r.reason}</td><td>{r.lap_times_s.map(seconds).join(' / ') || '—'}</td></tr>))}</tbody></table></div>}
    </>}
  </section>;
}
