import { useEffect, useRef, useState } from 'react';
import { api, bytes } from './api';
import { StereoBagDiagnostics } from './StereoBagDiagnostics';
import { DriveTimeline } from './DriveTimeline';
import { DeleteAnalysisButton } from './DeleteAnalysisButton';
import { ACTIVE, STATUS } from './job_status';
import { adjacentChanges, STOP_BAND_MPS, windowAt, type DirectionChange, type DriveReplay,
  type DriveWindow, type RecordedDirection } from './manual_replay';
import type { Bag, Job } from './types';
import './drive_monitor.css';

type JobProps = {jobs: Job[]; onLog: (id: string) => void};
const endpoint = 'manual-drive/replay';

export function ManualDriveReplay({jobs, onLog}: JobProps) {
  const [bags, setBags] = useState<Bag[] | null>(null), [bag, select] = useState('');
  const [error, setError] = useState(''), [reload, setReload] = useState(0);
  useEffect(() => {
    const abort = new AbortController();
    setError('');
    api<{items: Bag[]}>('records', undefined, undefined, abort.signal).then(({items}) => {
      if (abort.signal.aborted) return;
      setBags(items);
      select(previous => items.some(b => b.id === previous) ? previous : '');
    }).catch(e => { if (!abort.signal.aborted) setError(e.message); });
    return () => abort.abort();
  }, [reload]);
  return <section className="content-card manual-drive-panel drive-replay" aria-label="記録した手動走行">
    <div className="manual-drive-heading"><h3>手動走行の記録</h3></div>
    <div className="drive-bag-choice"><label htmlFor="drive-bag">Notebook の rosbag</label>
      <select id="drive-bag" value={bag} onChange={e => select(e.target.value)}>
        <option value="">{bags ? (bags.length ? '走行を選択' : '受信済みの bag がありません') : error ? '一覧を取得できません' : '一覧を読み込み中…'}</option>
        {bags?.map(item => <option key={item.id} value={item.id}>{item.id} · {bytes(item.bytes)}</option>)}
      </select><button className="button small" onClick={() => setReload(n => n+1)}>一覧を更新</button>
    </div>
    {error && <p className="drive-note" role="alert">{error}</p>}
    {bags?.length === 0 && <p className="drive-note">「データ転送」で Jetson から走行 bag を受信してください。</p>}
    {bag && <StereoBagDiagnostics key={bag} bag={bag} jobs={jobs} onLog={onLog}/>}
    {bag && <ReplayBag key={bag} bag={bag} jobs={jobs} onLog={onLog}/>}
  </section>;
}

function ReplayBag({bag, jobs, onLog}: {bag: string} & JobProps) {
  const [meta, setMeta] = useState<DriveReplay | null>(null), [error, setError] = useState('');
  const [refresh, setRefresh] = useState(0), [submitting, setSubmitting] = useState(false);
  const [chosenSpeed, chooseSpeed] = useState<string | null>(null), [startingVo, setStartingVo] = useState(false);
  const requestedVo = useRef<string | null>(null);
  const job = jobs.find(item => item.id === meta?.job_id);
  const voJob = jobs.find(item => item.id === (requestedVo.current ?? meta?.vo_job_id));
  const jobStatus = job?.status;
  const running = !!meta?.job_id && (!job || ACTIVE.has(job.status));
  const voRunning = !!(requestedVo.current ?? meta?.vo_job_id) && (!voJob || ACTIVE.has(voJob.status));
  const generatedSpeed = voJob?.status === 'succeeded' && typeof voJob.result?.speed_source === 'string'
    ? voJob.result.speed_source : null;
  function check(data: DriveReplay) {
    if (data.bag !== bag || (data.ready && (data.duration_s === null || !Number.isFinite(data.duration_s) || data.duration_s < 0))) {
      throw new Error('選択した bag の記録時間を確認できません。');
    }
    return data;
  }
  useEffect(() => {
    const abort = new AbortController();
    api<DriveReplay>(endpoint + '?' + new URLSearchParams({bag}), undefined, undefined, abort.signal).then(data => {
      if (abort.signal.aborted) return;
      setMeta(check(data)); setError('');
      if (requestedVo.current && generatedSpeed && data.speed_sources.some(item => item.id === generatedSpeed)) {
        chooseSpeed(generatedSpeed);
        requestedVo.current = null;
      }
    }).catch(e => { if (!abort.signal.aborted) { setMeta(null); setError(e.message); } });
    return () => abort.abort();
  }, [bag, jobStatus, voJob?.status, generatedSpeed, refresh]);
  async function extract() {
    setSubmitting(true); setError('');
    try { setMeta(check(await api<DriveReplay>(endpoint, {bag}))); }
    catch (e) { setError((e as Error).message); }
    finally { setSubmitting(false); }
  }
  async function generateVo() {
    setStartingVo(true); setError('');
    try {
      const result = check(await api<DriveReplay>(endpoint + '/vo', {bag}));
      requestedVo.current = result.vo_job_id;
      setMeta(result);
    } catch (e) { setError((e as Error).message); }
    finally { setStartingVo(false); }
  }
  const source = meta?.speed_sources?.find(item => item.id === chosenSpeed)?.id ?? meta?.default_speed_source ?? null;
  return <>
    {meta?.warnings.map(w => <p className="drive-note" key={w}>{w}</p>)}
    {error && <p className="drive-note" role="alert">{error} <button className="button small" onClick={() => setRefresh(n => n+1)}>再読込</button></p>}
    {!meta && !error && <p className="drive-note" role="status">記録の状態を確認中…</p>}
    {meta && !meta.ready && <div className="drive-extraction">
      <button className="button primary small" disabled={running || submitting} onClick={() => void extract()}>
        {submitting ? '受付中…' : running ? '時系列を読み出し中…' : '時系列を読み出す'}</button>
      {job?.message && <span role="status">{STATUS[job.status] || job.status} · {job.message}</span>}
      {meta.job_id && <button className="button small" onClick={() => onLog(meta.job_id!)}>ログ</button>}
    </div>}
    {meta?.ready && meta.duration_s !== null && <>
      <div className="drive-speed-choice">
        <label htmlFor="drive-speed-source">VSLAM 速度の出典</label>
        {meta.speed_sources.length ? <select id="drive-speed-source" value={source ?? ''} onChange={e => chooseSpeed(e.target.value)}>
          {source === null && <option value="">出典を選択</option>}
          {meta.speed_sources.map(item => <option key={item.id} value={item.id}>{item.label}</option>)}
        </select> : <span>推定速度なし</span>}
        {(meta.can_generate_vo || voRunning) && <button className="button small" disabled={voRunning || startingVo}
          onClick={() => void generateVo()}>{startingVo ? '受付中…' : voRunning ? 'VSLAM 速度を生成中…' : 'ステレオから VSLAM 速度を生成'}</button>}
        <button className="button small" onClick={() => setRefresh(n => n+1)}>出典を更新</button>
        <DeleteAnalysisButton path={endpoint} body={{bag, index_id: meta.index_id}} label="抽出した時系列を削除"
          detail={`「${bag}」から抽出した操作・PWM・IMU の時系列を削除します。元の rosbag から再度読み出せます。`}
          disabled={running || voRunning || submitting || startingVo} onDeleted={() => {
            requestedVo.current = null; chooseSpeed(null); setMeta(null); setRefresh(n => n + 1);
          }}/>
        {voJob && <button className="button small" onClick={() => onLog(voJob.id)}>{STATUS[voJob.status] || voJob.status} · ログ</button>}
      </div>
      {!source && meta.speed_reason && <p className="drive-note">{meta.speed_reason}</p>}
      {meta.vo_reason && <p className="drive-note">{meta.vo_reason}</p>}
      {meta.imu_reason && <p className="drive-note">{meta.imu_reason}</p>}
      <WindowViewer bag={bag} duration={meta.duration_s} maxWidth={meta.max_window_s} speedSource={source} revision={refresh}/>
    </>}
  </>;
}

function WindowViewer({bag, duration, maxWidth, speedSource, revision}: {
  bag: string; duration: number; maxWidth: number; speedSource: string | null; revision: number;
}) {
  const [cursor, setCursor] = useState(0), [width, setWidth] = useState(Math.min(10,maxWidth));
  const [playing, setPlaying] = useState(false), [refresh, setRefresh] = useState(0);
  const [data, setData] = useState<DriveWindow | null>(null), [error, setError] = useState('');
  const {start_s, end_s} = windowAt(cursor, duration, width);
  const current = data?.bag === bag && data.start_s === start_s && data.end_s === end_s && data.speed_source === speedSource ? data : null;
  const seek = (t: number) => { setPlaying(false); setCursor(Math.max(0, Math.min(duration, t))); };
  useEffect(() => {
    setError('');
    const abort = new AbortController();
    const query = new URLSearchParams({bag, start_s:String(start_s), end_s:String(end_s)});
    if (speedSource !== null) query.set('speed_source', speedSource);
    api<DriveWindow>(endpoint + '/window?' + query, undefined, undefined, abort.signal).then(data => {
      if (abort.signal.aborted) return;
      if (data.bag !== bag || data.start_s !== start_s || Math.abs(data.end_s - end_s) > 1e-8
          || data.duration_s !== duration || data.speed_source !== speedSource
          || ![data.joy, data.board, data.propo, data.imu, data.speed].every(Array.isArray)) {
        throw new Error('時系列の bag または表示区間が一致しません。');
      }
      setData({...data, end_s});
    }).catch(e => { if (!abort.signal.aborted) { setData(null); setPlaying(false); setError(e.message); } });
    return () => abort.abort();
  }, [bag, start_s, end_s, duration, speedSource, refresh, revision]);

  useEffect(() => {
    if (!playing || !current) return;
    let last = performance.now();
    const timer = setInterval(() => {
      const now = performance.now(), step = (now-last)/1000;
      last = now;
      setCursor(t => Math.min(duration, t+step));
    }, 75);
    return () => clearInterval(timer);
  }, [playing, current, duration]);
  useEffect(() => { if (cursor >= duration) setPlaying(false); }, [cursor, duration]);

  return <>
    <div className="drive-time-controls">
      <button className="button small" disabled={!duration || (!current && !playing)}
        onClick={() => { if (cursor >= duration) setCursor(0); setPlaying(!playing); }}>{playing ? '一時停止' : '再生'}</button>
      <label htmlFor="drive-seek">bag 内時刻</label><input id="drive-seek" type="range" min={0} max={duration}
        step="any" value={cursor} onChange={e => seek(Number(e.target.value))} aria-valuetext={cursor.toFixed(3) + ' 秒'}/>
      <output htmlFor="drive-seek">{cursor.toFixed(3)} / {duration.toFixed(3)} s</output>
    </div>
    {speedSource !== null && !current?.speed_issue && <MotionTransitions key={speedSource} bag={bag} speedSource={speedSource} revision={revision} cursor={cursor} seek={seek}/>}
    <div className="drive-window-controls">
      <label>表示幅 <select value={width} onChange={e => {setPlaying(false); setWidth(Number(e.target.value));}}>
        {[1,5,10,30].filter(n => n <= maxWidth).map(n => <option value={n} key={n}>{n} 秒</option>)}</select></label>
      <button className="button small" disabled={start_s <= 0} onClick={() => seek(start_s-width)}>前の区間</button>
      <button className="button small" disabled={end_s >= duration} onClick={() => seek(end_s)}>次の区間</button>
      <form onSubmit={e => {e.preventDefault(); const n = Number(new FormData(e.currentTarget).get('time')); if (Number.isFinite(n)) seek(n);}}>
        <label>指定時刻 [s] <input name="time" type="number" min={0} max={duration} step="any" required/></label>
        <button className="button small">移動</button>
      </form>
    </div>
    {error ? <p className="drive-note" role="alert">{error} <button className="button small" onClick={() => setRefresh(n => n+1)}>区間を再読込</button></p>
      : current ? <DriveTimeline data={current} cursor={cursor} seek={seek}/>
      : <p className="drive-note" role="status">選択区間を読み込み中…</p>}
  </>;
}

const eventLabels: Record<RecordedDirection,string> = {forward:'前進', near_zero:'停止付近', reverse:'後退'};
type ChangesResult = {bag: string; speed_source: string; threshold_mps: number; confirm_s: number; transitions: DirectionChange[]; issue: string};
function MotionTransitions({bag, speedSource, revision, cursor, seek}: {
  bag: string; speedSource: string; revision: number; cursor: number; seek: (time: number) => void;
}) {
  const [data, setData] = useState<ChangesResult | null>(null), [error, setError] = useState('');
  const [direction, setDirection] = useState<RecordedDirection>('reverse'), [refresh, setRefresh] = useState(0);
  useEffect(() => {
    const abort = new AbortController();
    setData(null); setError('');
    const params = new URLSearchParams({bag, speed_source:speedSource, threshold_mps:String(STOP_BAND_MPS)});
    api<ChangesResult>(endpoint + '/transitions?' + params, undefined, undefined, abort.signal).then(reply => {
      if (abort.signal.aborted) return;
      if (reply.bag !== bag || reply.speed_source !== speedSource || reply.threshold_mps !== STOP_BAND_MPS
          || !Array.isArray(reply.transitions)) throw new Error('走行と推定速度の切替時刻を照合できません。');
      setData(reply);
    }).catch(e => { if (!abort.signal.aborted) setError(e.message); });
    return () => abort.abort();
  }, [bag, speedSource, refresh, revision]);
  const events = data?.transitions.filter(event => event.to === direction) ?? [];
  const {previous, current, next} = adjacentChanges(events, cursor);
  return <div className="drive-transitions">
    <label>速度の切替へ移動 <select value={direction} onChange={e => setDirection(e.target.value as RecordedDirection)}>
      {(['reverse','near_zero','forward'] as const).map(mode =>
        <option value={mode} key={mode}>{eventLabels[mode]}に移行</option>)}
    </select></label>
    <button className="button small" disabled={!previous} onClick={() => previous && seek(previous.time_s)}>前の切替</button>
    <button className="button small" disabled={!next} onClick={() => next && seek(next.time_s)}>次の切替</button>
    {current && <span>{current.time_s.toFixed(3)} s · {eventLabels[current.from]} → {eventLabels[current.to]}</span>}
    {current?.to === 'reverse' && current.near_zero_start_s != null &&
      <button className="button small" onClick={() => seek(current.near_zero_start_s!)}>直前の停止付近へ</button>}
    {error ? <span role="alert">{error} <button className="button small" onClick={() => setRefresh(n => n+1)}>再読込</button></span>
      : data?.issue ? <span>{data.issue}</span> : <small>{data ? `走行全体で${events.length}件 · 推定状態が${data.confirm_s}秒続いた区間の始点` : '切替時刻を確認中…'}</small>}
  </div>;
}
