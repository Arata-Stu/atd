import { useEffect, useState } from 'react';
import { api } from './api';
import { ACTIVE, STATUS } from './job_status';
import { LINE_CHOICES, LINE_NAMES, planPose, usablePlan } from './planning_view';
import { ControlReadout, ReplayControlConfig } from './ControlReadout';
import { controlConfigIssue, controlQuery } from './control_view';
import type { ControlConfig, ControlFrame } from './control_view';
import type { PlanFrame, PlanPose } from './planning_view';
import type { Job, LineKind } from './types';
import { PlanningMetrics } from './PlanningMetrics';
import { ReplaySpeed, useReplaySpeed } from './ReplaySpeed';
import type { SpeedSettings } from './ReplaySpeed';

interface ReplayMeta {
  id: string; line: LineKind; ready: boolean; duration_s: number; pose_samples: number;
  valid_samples: number | null; job_id: string | null; control_config?: ControlConfig;
}
interface FrameReply extends Omit<PlanFrame, 'vehicle_pose'> {
  id: string; line: LineKind; time_s: number; sample_s: number | null; vehicle: PlanPose | null; control?: ControlFrame;
}
interface Frame extends PlanFrame {sample_s: number | null; control?: ControlFrame}
export function useReplayPlanning(id: string, line: LineKind, issue: string, time: number, jobs: Job[], config: ControlConfig | null) {
  const [meta, setMeta] = useState<ReplayMeta | null>(null), [error, setError] = useState('');
  const [frameData, setFrameData] = useState<{time: number; tuning: string; frame: Frame} | null>(null), [frameError, setFrameError] = useState('');
  const [loading, setLoading] = useState(true), [submitting, setSubmitting] = useState(false), [cancelling, setCancelling] = useState(false);
  const [refresh, setRefresh] = useState(0);
  const base = 'localization/runs/' + encodeURIComponent(id) + '/planning';
  const query = '?line=' + line;
  const tuning = controlQuery(config);
  const job = jobs.find(item => item.id === meta?.job_id);
  const jobState = job?.status;
  useEffect(() => {
    if (issue) return;
    let live = true;
    setError('');
    api<ReplayMeta>(base + query).then(data => {
      if (data.id !== id || data.line !== line) throw new Error('走行計画の検証 ID・ラインが一致しません。');
      if (live) setMeta(data);
    }).catch(e => { if (live) { setMeta(null); setError(e.message); } }).finally(() => { if (live) setLoading(false); });
    return () => { live = false; };
  }, [id, line, base, query, issue, refresh, jobState]);

  useEffect(() => {
    if (!meta?.ready || issue) return;
    const controller = new AbortController();
    setFrameError('');
    const timer = setTimeout(() => {
      api<FrameReply>(base + '/frame' + query + '&time_s=' + time + tuning, undefined, undefined, controller.signal).then(reply => {
        if (controller.signal.aborted) return;
        if (reply.id !== id || reply.line !== line || !Number.isFinite(reply.time_s) || Math.abs(time - reply.time_s) > .00001)
          throw new Error('要求した走行・ライン・時刻と計画が一致しません。');
        const frame = {...reply, vehicle_pose: planPose(reply.vehicle)};
        if (frame.valid && !usablePlan(frame)) throw new Error('観測時の姿勢・前方軌道・速度を確認できません。');
        setFrameData({time, tuning, frame});
      }).catch(e => { if (!controller.signal.aborted) { setFrameData(null); setFrameError(e.message); } });
    }, 50);
    return () => { clearTimeout(timer); controller.abort(); };
  }, [base, id, line, query, time, tuning, meta?.ready, issue, refresh]);

  async function calculate() {
    setError(''); setSubmitting(true); setCancelling(false);
    try {
      const data = await api<ReplayMeta>(base, {line});
      if (data.id !== id || data.line !== line) throw new Error('計算の受付結果が一致しません。');
      setMeta(data);
    } catch (e) { setError((e as Error).message); }
    finally { setSubmitting(false); }
  }
  async function cancel() {
    if (!job) return;
    setCancelling(true); setError('');
    try { await api('jobs/' + encodeURIComponent(job.id) + '/cancel', {}); }
    catch (e) { setError((e as Error).message); setCancelling(false); }
  }
  const frame = !issue && meta?.ready && frameData?.time === time && frameData.tuning === tuning ? frameData.frame : null;
  return {id, jobs, meta, frame, job, error, frameError, loading, submitting, cancelling, calculate, cancel,
    refresh: () => setRefresh(index => index + 1)};
}

export function ReplayPlanControls({line, lines, choose, time, seek, selection, issue, plan, onLog, config, applyConfig, speedSettings, setSpeedSettings}: {
  speedSettings:SpeedSettings | null; setSpeedSettings:(c:SpeedSettings | null) => void;
  line: LineKind; lines: LineKind[]; choose: (k: LineKind) => void; time: number; seek: (t: number) => void;
  selection: {index: number} | null; issue: string; plan: ReturnType<typeof useReplayPlanning>; onLog: (id: string) => void;
  config: ControlConfig | null; applyConfig: (config: ControlConfig | null) => void;
}) {
  const [playing, setPlaying] = useState(false);
  useEffect(() => { if (selection) setPlaying(false); }, [selection]);
  const {meta, job, frame} = plan;
  const ready = !!meta?.ready && !issue;
  const running = !!meta?.job_id && !ready && (!job || ACTIVE.has(job.status));
  const duration = meta?.duration_s ?? 0;
  const speed = useReplaySpeed(plan.id, line, time, ready, plan.jobs, speedSettings);
  const waitForSpeed = !!speed.meta?.ready && !speed.frame;
  useEffect(() => {
    if (!playing || !frame || !ready || waitForSpeed || time >= duration) return;
    const timer = setTimeout(() => seek(Math.min(duration, +(time + .25).toFixed(6))), 200);
    return () => clearTimeout(timer);
  }, [playing, frame, ready, waitForSpeed, time, duration, seek]);
  useEffect(() => { if (time >= duration || plan.frameError || speed.frameFailure) setPlaying(false); }, [time, duration, plan.frameError, speed.frameFailure]);
  const note = issue || plan.error || plan.frameError || (ready ? (frame ? (frame.valid ? '' : frame.reason || '計画が無効です。') : '選択時刻を読み込み中…') :
    plan.loading ? '計算結果を確認中…' : running ? '推定軌跡から前方軌道を計算しています。' :
      job && !ready ? (job.message || '計算をもう一度実行できます。') : '検証時の姿勢と速度を使って、選択したラインの前方軌道を計算します。');
  const defaults = meta?.control_config;
  return <section className="planning-replay" aria-label="別走行 bag の走行計画">
    <div className="planning-header"><h4>走行計画</h4><div className="planning-action">
      <label htmlFor="replay-planning-line">基準ライン</label>
      <select id="replay-planning-line" value={line} disabled={!lines.length} onChange={e => choose(e.target.value as LineKind)}>
        {LINE_CHOICES.map(k => <option key={k} value={k} disabled={!lines.includes(k)}>{LINE_NAMES[k]}{!lines.includes(k) && '（未保存）'}</option>)}
      </select>
      {!ready && <button className="button primary small" disabled={!!issue || plan.loading || !meta || plan.submitting || running}
        onClick={() => void plan.calculate()}>{plan.submitting ? '受付中…' : running ? '計算中…' : '前方軌道を計算'}</button>}
      {job && <button className="button small" onClick={() => onLog(job.id)}>{STATUS[job.status] || job.status} · ログ</button>}
      {job && ACTIVE.has(job.status) && <button className="button small" disabled={plan.cancelling || job.cancel_requested}
        onClick={() => void plan.cancel()}>計算を中止</button>}
      {(plan.error || plan.frameError) && <button className="button small" onClick={plan.refresh}>再読込</button>}
    </div></div>
    {ready && <div className="planning-time">
      <button className="button small" disabled={!duration} onClick={() => { if (!playing && time >= duration) seek(0); setPlaying(!playing); }}>{playing ? '一時停止' : '軌道を再生'}</button>
      <label htmlFor="plan-replay-time">bag 内時刻</label><input id="plan-replay-time" type="range" min="0" max={duration} step=".01"
        value={time} onChange={e => { setPlaying(false); seek(Number(e.target.value)); }} aria-valuetext={time.toFixed(2) + ' 秒'}/>
      <output htmlFor="plan-replay-time">{time.toFixed(2)} / {duration.toFixed(2)} s</output>
    </div>}
    <PlanningMetrics frame={frame} reason={note}/>
    {ready && <p className="planning-sample">有効な計画 {meta?.valid_samples?.toLocaleString() ?? '—'} / {meta?.pose_samples.toLocaleString()} 姿勢
      {typeof frame?.sample_s === 'number' && ' · 観測時刻 ' + frame.sample_s.toFixed(3) + ' s'}</p>}
    {ready && <ControlReadout frame={frame?.control} issue={!frame || !frame.valid ? note : ''}>
      {defaults && !controlConfigIssue(defaults) && <ReplayControlConfig defaults={defaults} config={config}
        apply={value => { setPlaying(false); applyConfig(value); }}/>}
    </ControlReadout>}
    {ready && <ReplaySpeed speed={speed} time={time} config={speedSettings} apply={c => {setPlaying(false); setSpeedSettings(c);}} onLog={onLog}/>}
  </section>;
}
