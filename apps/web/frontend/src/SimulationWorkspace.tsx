import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { api } from './api';
import { Icon } from './ui';
import { BenchmarkResults, type Circuit } from './BenchmarkResults';
import { ACTIVE, STATUS } from './job_status';
import type { Job } from './types';
import { CourseOverlay, TimePlots } from './SimulationPlots';
import { speedScale, type SpeedScale } from './raceline_speed';
import { SpeedLegend } from './SpeedLegend';
import { defaultSettings, modeColor, modeLabel, sampleAt } from './simulation';
import type { Comparison, SimSettings } from './simulation';
import './simulation.css';

type Control = { key: keyof SimSettings; label: string; unit: string; min: number; max: number };
const distances: Control[] = [
  { key: 'fixed_lookahead_m', label: '固定距離', unit: 'm', min: .15, max: 2 },
  { key: 'min_lookahead_m', label: '速度連動の最短距離', unit: 'm', min: .15, max: 2 },
  { key: 'max_lookahead_m', label: '速度連動の最長距離', unit: 'm', min: .15, max: 2 },
  { key: 'lookahead_time_s', label: '先読み時間', unit: 's', min: 0, max: 2 },
];
const responses: Control[] = [
  { key: 'steering_delay_ms', label: '操舵のむだ時間', unit: 'ms', min: 0, max: 1000 },
  { key: 'throttle_delay_ms', label: 'スロットルのむだ時間', unit: 'ms', min: 0, max: 1000 },
  { key: 'steering_tau_s', label: '操舵の時定数', unit: 's', min: .02, max: 2 },
  { key: 'speed_tau_s', label: '速度の時定数', unit: 's', min: .02, max: 2 },
];

function Inputs({ controls }: { controls: Control[] }) {
  return <div className="sim-fields">{controls.map(({key, label, unit, min, max}) => <label key={key}>
    <span>{label} <small>({unit})</small></span>
    <input type="number" name={key} defaultValue={defaultSettings[key]} required min={min} max={max} step="any"/>
  </label>)}</div>;
}

function Legend({ reference = false, lineScale }: { reference?: boolean; lineScale?: SpeedScale | null }) {
  return <div className="sim-legend" aria-label="凡例">
    {lineScale ? <SpeedLegend scale={lineScale}/> : reference && <span><i className="sim-swatch reference"/>基準ライン</span>}
    <span><i className="sim-swatch fixed"/>固定</span>
    <span><i className={'sim-swatch ' + (reference ? 'adaptive dashed' : 'adaptive')}/>速度連動</span>
  </div>;
}

function Results({ result }: { result: Comparison }) {
  const [time, setTime] = useState(0);
  const { course } = result;
  const lineScale = useMemo(() => course.line === 'raceline' ? speedScale(course.speeds_mps, course.points.length) : null, [course]);
  const end = Math.max(result.model.dt_s, ...result.runs.map(run => run.samples.at(-1)?.t_s ?? 0));
  const step = result.model.dt_s;
  const select = (t: number) => setTime(Math.max(0, Math.min(end, Number((Math.round(t/step)*step).toFixed(6)))));
  return <>
    <div className="sim-grid">
      <section className="sim-card sim-track">
        <div className="sim-section-title"><h2>走行軌跡</h2><span>{result.course.name} · {result.course.length_m.toFixed(1)} m</span></div>
        <Legend reference lineScale={lineScale}/>
        <CourseOverlay result={result} time={time} lineScale={lineScale}/>
        <div className="sim-track-foot"><span>矢印は車両の向き、○ は先読み点{result.course.left && '、× は接触時の車両位置'}</span><span>+X 右 / +Y 上</span></div>
        <label className="sim-scrubber"><span>比較時刻 <output>{time.toFixed(2)} <small>s</small></output></span>
          <input type="range" min="0" max={end} step={step} value={time} onChange={e => select(e.target.valueAsNumber)} aria-label="比較時刻（秒）"/>
        </label>
        <div className="sim-table-wrap"><table className="sim-readouts"><caption>選択時刻の車両状態</caption>
          <thead><tr><th scope="col">方式</th><th scope="col">先読み m</th><th scope="col">横ずれ m</th><th scope="col">速度 m/s</th><th scope="col">操舵 °</th></tr></thead>
          <tbody>{result.runs.map(run => {
            const p = sampleAt(run.samples, time);
            return <tr key={run.mode}><th scope="row" style={{color:modeColor[run.mode]}}>{modeLabel[run.mode]}</th>
              {p ? <><td>{p.lookahead_m.toFixed(2)}</td><td>{p.lateral_error_m.toFixed(3)}</td><td>{p.speed_mps.toFixed(2)}</td><td>{p.steering_deg.toFixed(1)}</td></> :
                <td colSpan={4}>{run.completed ? '走行終了' : '走行打ち切り'}</td>}</tr>;
          })}</tbody>
        </table></div>
      </section>
      <section className="sim-card"><div className="sim-section-title"><h2>走行時系列</h2><span>経過時間 (s)</span></div>
        <Legend/><TimePlots result={result} time={time} end={end} select={select}/>
      </section>
    </div>
    <section className="sim-card">
      <div className="sim-section-title"><h2>走行結果</h2><span>目標 {result.model.laps} 周</span></div>
      <div className="sim-table-wrap"><table><thead><tr><th scope="col">先読み方式</th><th scope="col">完了周回</th><th scope="col">走行時間 (s)</th>
        <th scope="col">横ずれ RMS (m)</th><th scope="col">最大 |横ずれ| (m)</th><th scope="col">操舵飽和 (s)</th></tr></thead>
        <tbody>{result.runs.map(run => <tr key={run.mode}><th scope="row" style={{color:modeColor[run.mode]}}>{modeLabel[run.mode]}</th>
          <td>{run.metrics.laps_completed} 周 · {run.completed ? '完走' : '打ち切り'}</td><td>{run.metrics.elapsed_s.toFixed(2)}</td>
          <td>{run.metrics.rms_lateral_error_m.toFixed(3)}</td><td>{run.metrics.max_abs_lateral_error_m.toFixed(3)}</td>
          <td>{run.metrics.steering_saturation_s.toFixed(2)}</td></tr>)}</tbody></table></div>
      {result.runs.filter(run => !run.completed).map(run => <p key={run.mode} className="sim-warning">{modeLabel[run.mode]}の終了理由　{run.reason}</p>)}
    </section>
  </>;
}

export function SimulationWorkspace({ jobs, onOpenMap, onLog }: { jobs: Job[]; onOpenMap: (id: string) => void; onLog: (id: string) => void }) {
  const [result, setResult] = useState<Comparison | null>(null);
  const [error, setError] = useState(''), [pending, setPending] = useState(true), [edited, setEdited] = useState(false);
  const [catalog, setCatalog] = useState<Circuit[] | null>(null), [catalogError, setCatalogError] = useState('');
  const [key, setKey] = useState(''), [line, setLine] = useState('raceline');
  const [batchView, setBatchView] = useState(() =>
    new URLSearchParams(window.location.hash.split('?')[1]).get('view') === 'benchmark');
  const [submitting, setSubmitting] = useState(false);
  const [scope, setScope] = useState('all'), [laps, setLaps] = useState(2), [trials, setTrials] = useState(3), [trial, setTrial] = useState(1);
  const [submitted, setSubmitted] = useState<Job>();
  const circuits = catalog ?? [];
  const circuit = circuits.find(c => c.id === key);
  const allCourses = batchView && scope === 'all';
  const usableLines = allCourses ? ['raceline', 'centerline'].filter(kind => circuits.some(c => c.lines.includes(kind))) : circuit?.lines ?? [];
  const selectedLine = usableLines.includes(line) ? line : usableLines[0];
  const courseIds = allCourses ? circuits.filter(c => c.lines.includes(selectedLine)).map(c => c.id) : circuit ? [circuit.id] : [];
  const benchJobs = jobs.filter(job => job.action === 'simulation_benchmark');
  const job = benchJobs.find(j => ACTIVE.has(j.status)) ?? benchJobs.find(j => j.id === submitted?.id) ?? submitted ?? benchJobs[0];
  const latest = benchJobs.find(j => j.status === 'succeeded')?.id ?? '';
  const active = !!job && ACTIVE.has(job.status);
  useEffect(() => {
    const controller = new AbortController();
    api<{items: Circuit[]}>('simulation/courses', undefined, undefined, controller.signal)
      .then(data => setCatalog(data.items)).catch(e => { if (!controller.signal.aborted) setCatalogError(e.message); });
    return () => controller.abort();
  }, []);
  const [revision, setRevision] = useState(0);
  const request = useRef<AbortController | null>(null);
  const compare = useCallback(async (settings: SimSettings, course = '', selectedLine = 'raceline', nLaps=2, start=1) => {
    request.current?.abort();
    const controller = new AbortController();
    request.current = controller;
    setPending(true); setError('');
    try {
      const path = course ? 'simulation/courses/' + encodeURIComponent(course) + '/compare' : 'simulation/compare';
      const payload = course ? {settings, line:selectedLine, laps:nLaps, trial:start} : settings;
      const data = await api<Comparison>(path, payload, 'POST', controller.signal);
      if (controller.signal.aborted) return;
      setResult(data); setRevision(i => i+1); setEdited(false);
    } catch (e) { if (!controller.signal.aborted) setError((e as Error).message); }
    finally { if (!controller.signal.aborted) setPending(false); }
  }, []);
  useEffect(() => {
    const timer = window.setTimeout(() => void compare(defaultSettings), 0);
    return () => { window.clearTimeout(timer); request.current?.abort(); };
  }, [compare]);

  function changeView(value: boolean) {
    setBatchView(value); setError('');
    window.history.replaceState(null, '', value ? '#simulation?view=benchmark' : '#simulation');
  }
  return <div className="sim-workspace"><div className="sim-content">
    <div className="sim-heading"><h1>{batchView ? 'コース別ベンチマーク' : '先読み距離の比較'}</h1>
      <div className="bench-actions" aria-label="シミュレーターの表示">
        <button className="button" aria-pressed={!batchView} onClick={() => changeView(false)}>走行比較</button>
        <button className="button" aria-pressed={batchView} onClick={() => changeView(true)}>コース別評価</button>
      </div></div>
    <form className="sim-card sim-settings" onChange={() => setEdited(true)} onSubmit={async event => {
      event.preventDefault();
      const values = new FormData(event.currentTarget);
      const settings = { ...defaultSettings };
      for (const c of [...distances, ...responses]) settings[c.key] = Number(values.get(c.key));
      if (settings.max_lookahead_m < settings.min_lookahead_m) { setError('最長距離は最短距離以上にしてください。'); return; }
      if (!batchView) { void compare(settings, key, selectedLine, laps, trial); return; }
      if (!courseIds.length) { setError('評価対象のコースを選択してください。'); return; }
      setSubmitting(true); setError('');
      try {
        const launched = await api<Job>('simulation/benchmarks', {course_ids: courseIds, line: selectedLine, laps, trials, settings});
        setSubmitted(launched);
      } catch (e) { setError((e as Error).message); } finally { setSubmitting(false); }
    }}>
      <div className="sim-section-title"><h2>比較条件</h2><button className="button primary" type="submit" disabled={pending || submitting || (batchView && (active || !courseIds.length))}>
        <Icon name="play" size={15}/>{batchView ? (active ? '評価中…' : '評価を実行') : pending ? '計算中…' : '比較を実行'}</button></div>
      <fieldset disabled={pending || submitting}><div className="sim-fields bench-selection">
        {batchView && <label>評価対象<select aria-label="評価対象" value={scope} onChange={e => setScope(e.target.value)}>
          <option value="all">選択ラインのある全コース</option><option value="one">選択したコース</option></select></label>}
        {(!batchView || scope === 'one') && <label>コース<select aria-label="コース" value={key} onChange={e => setKey(e.target.value)}>
          {!batchView ? <option value="">ヘアピンと高速コーナー</option> : !key && <option value="" disabled>コースを選択</option>}{circuits.map(c => <option key={c.id} value={c.id}>{c.name}</option>)}</select></label>}
        {usableLines.length > 0 && <label>走行ライン<select aria-label="走行ライン" value={selectedLine} onChange={e => setLine(e.target.value)}>
          {usableLines.map(kind => <option key={kind} value={kind}>{kind === 'raceline' ? 'Raceline' : 'Centerline'}</option>)}</select></label>}
        {(batchView || key) && <label>規定周回数<select aria-label="規定周回数" value={laps} onChange={e => setLaps(Number(e.target.value))}>{[1,2,3,4,5].map(n => <option key={n} value={n}>{n} 周</option>)}</select></label>}
        {batchView && <label>初期条件の数<select aria-label="初期条件の数" value={trials} onChange={e => setTrials(Number(e.target.value))}>{[1,2,3].map(n => <option key={n} value={n}>{n} 試行 / 方式</option>)}</select></label>}
        {!batchView && key && <label>開始姿勢<select aria-label="開始姿勢" value={trial} onChange={e => setTrial(Number(e.target.value))}><option value={1}>① ライン上</option><option value={2}>② 左 6 cm・+3°</option><option value={3}>③ 右 6 cm・−3°</option></select></label>}
      </div>
      {circuit && (!batchView || scope === 'one') && <button className="bench-link" type="button" onClick={() => onOpenMap(circuit.id)}>{circuit.sample ? '編集用にサンプルをコピー' : 'HDMap を開く'}</button>}
      <Inputs controls={distances}/>
        <p className="sim-note">速度連動の先読み距離は「最短距離 ＋ 速度 × 先読み時間」で求め、最長距離で制限します。</p>
        <details className="sim-model"><summary>車両モデル・遅延設定</summary>
          <p>両方式に同じ車両モデルを使います。むだ時間は指令から応答開始までの待ち時間、時定数はその後の追従の速さを表します。初期値は仮定値です。</p>
          <Inputs controls={responses}/>
          {result && <p>運動学的自転車モデル · 模擬 FF 表 · ホイールベース {result.model.wheelbase_m.toFixed(3)} m · 操舵上限 ±{result.model.steering_limit_deg.toFixed(1)}° · 計算刻み {(1000*result.model.dt_s).toFixed(0)} ms</p>}
        </details>
      </fieldset>
      {batchView && catalog?.length === 0 && <p className="sim-note">「地図とライン」で閉路の HDMap に Centerline または Raceline を生成すると、ここで評価できます。</p>}
      {!batchView && edited && result && <p className="sim-warning" role="status">比較条件を変更しました。現在のグラフは前回の実行結果です。</p>}
      {batchView && job && <p role="status">{STATUS[job.status] ?? job.status} · {job.message} <button type="button" className="bench-link" onClick={() => onLog(job.id)}>評価ログ</button></p>}
      {(error || catalogError) && <p className="sim-error" role="alert">{error || catalogError}</p>}
    </form>
    {batchView ? <BenchmarkResults latest={latest} circuits={circuits}/> : result ? <Results key={revision} result={result}/> : pending && <p role="status" className="sim-note">2方式の走行軌跡を計算しています…</p>}
  </div></div>;
}
