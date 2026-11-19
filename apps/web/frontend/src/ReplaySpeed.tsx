import { useEffect, useState } from 'react';
import { api } from './api';
import { ACTIVE, STATUS } from './job_status';
import { GainsText, SpeedComparison, TableSelect, TablesNotice, useSpeedTables } from './SpeedComparison';
import { finite, speedPoint } from './speed_view';
import type { SpeedGains, SpeedSample } from './speed_view';
import type { Job, LineKind } from './types';

export interface SpeedSettings extends SpeedGains {table_sha256:string}
type ReplaySample = SpeedSample & {time_s:number; segment:number | null};
interface SpeedMeta {id:string; line:LineKind; table_sha256:string; table_run_id:string; ready:boolean; job_id:string | null;
  duration_s:number; config:SpeedGains; propo_reason:string; joy_reason?:string}
interface Frame {id:string; line:LineKind; table_sha256:string; time_s:number; sample:ReplaySample | null; history:ReplaySample[]}

export function useReplaySpeed(id:string, line:LineKind, time:number, planReady:boolean, jobs:Job[], config:SpeedSettings | null) {
  const [statusData,setStatus] = useState<{key:string; data:SpeedMeta} | null>(null);
  const [frameData,setFrame] = useState<{key:string; data:Frame} | null>(null);
  const [failure,setFailure] = useState(''), [frameFailure,setFrameFailure] = useState('');
  const [submitting,setSubmitting] = useState(false), [refresh,setRefresh] = useState(0);
  const path = 'localization/runs/' + encodeURIComponent(id) + '/speed';
  const query = config ? '?' + new URLSearchParams({line, table_sha256:config.table_sha256,
    kp:String(config.kp),ki:String(config.ki),kd:String(config.kd)}) : '';
  const key = path + query, timeKey = path + '/frame' + query + '&time_s=' + time, meta = statusData?.key === key ? statusData.data : null;
  const job = jobs.find(j => j.id === meta?.job_id);
  const jobStatus = job?.status;
  useEffect(() => {
    if (!planReady || !config) return;
    const abort = new AbortController(); setFailure('');
    const matches = (m:SpeedMeta) => m.id === id && m.line === line && m.table_sha256 === config.table_sha256
      && ['kp','ki','kd'].every(k => m.config?.[k as keyof SpeedGains] === config[k as keyof SpeedGains]);
    api<SpeedMeta>(key, undefined, undefined, abort.signal).then(m => {
      if (abort.signal.aborted) return;
      if (!matches(m)) throw new Error('速度計算の走行・ライン・設定が一致しません。');
      setStatus({key,data:m});
    }).catch(e => { if (!abort.signal.aborted) { setStatus(null); setFailure(e.message); } });
    return () => abort.abort();
  }, [key,id,line,config,planReady,jobStatus,refresh]);
  useEffect(() => {
    if (!meta?.ready || !planReady || !config) return;
    const abort = new AbortController(); setFrameFailure('');
    api<Frame>(timeKey, undefined, undefined, abort.signal).then(f => {
      if (abort.signal.aborted) return;
      if (f.id !== id || f.line !== line || f.table_sha256 !== config.table_sha256 || !finite(f.time_s)
          || Math.abs(f.time_s-time) > .00001 || !Array.isArray(f.history)) throw new Error('速度比較の表示時刻・設定を確認できません。');
      setFrame({key:timeKey,data:f});
    }).catch(e => { if (!abort.signal.aborted) { setFrame(null); setFrameFailure(e.message); } });
    return () => abort.abort();
  }, [timeKey,id,line,time,config,meta?.ready,planReady,refresh]);
  async function calculate(next = config) {
    if (!next || submitting) return;
    const nextKey = path + '?' + new URLSearchParams({line,table_sha256:next.table_sha256,
      kp:String(next.kp),ki:String(next.ki),kd:String(next.kd)});
    setSubmitting(true); setFailure('');
    try {
      const m = await api<SpeedMeta>(path, {line,...next});
      if (m.id !== id || m.line !== line || m.table_sha256 !== next.table_sha256) throw new Error('速度計算の受付結果が一致しません。');
      setStatus({key:nextKey,data:m}); setRefresh(v => v+1);
    } catch (e) { setFailure((e as Error).message); }
    finally { setSubmitting(false); }
  }
  const frame = planReady && meta?.ready && frameData?.key === timeKey ? frameData.data : null;
  return {meta,job,frame,submitting,failure,frameFailure,calculate,refresh:() => setRefresh(n => n+1)};
}

export function ReplaySpeed({speed, time, config, apply, onLog}: {
  speed:ReturnType<typeof useReplaySpeed>; time:number; config:SpeedSettings | null; apply:(c:SpeedSettings | null) => void;
  onLog:(id:string) => void;
}) {
  const tables = useSpeedTables();
  const [choice,setChoice] = useState(config?.table_sha256 || '');
  const [edits,setEdits] = useState<Partial<Record<keyof SpeedGains,string>>>({});
  const gains = config ?? tables.data?.defaults;
  const value = (k:keyof SpeedGains) => edits[k] ?? (gains ? String(gains[k]) : '');
  const read = (k:keyof SpeedGains) => value(k).trim() ? Number(value(k)) : NaN;
  const submit:SpeedSettings = {table_sha256:choice, kp:read('kp'),ki:read('ki'),kd:read('kd')};
  const gainValid = [submit.kp,submit.ki,submit.kd].every(v => finite(v) && v >= 0);
  const changed = !config || choice !== config.table_sha256 || submit.kp !== config.kp || submit.ki !== config.ki || submit.kd !== config.kd;
  const running = !!speed.meta?.job_id && !speed.meta.ready && (!speed.job || ACTIVE.has(speed.job.status));
  const available = tables.data?.items.some(t => t.proposal_sha256 === choice) === true;
  const ready = !!speed.meta?.ready, frame = speed.frame;
  const issue = speed.failure || (!config ? '校正表を選んで速度制御を計算してください。' :
    !speed.meta ? '速度計算の状態を確認中…' : !ready ? (running ? '走行時刻順に速度制御を計算中…' :
      speed.job?.message || 'この設定で走行全体の速度制御を計算できます。') :
      speed.frameFailure || (!frame ? '選択時刻の速度比較を読み込み中…' :
        !frame.sample ? 'この時刻に有効な走行計画がありません。' : ''));
  const history = (frame?.history ?? []).map(p => ({...speedPoint(p.time_s,p),segment:p.segment}));
  const fields: [keyof SpeedGains,string][] = [['kp','Kp [raw/(m/s)]'],['ki','Ki [raw/m]'],['kd','Kd [raw/(m/s²)]']];
  return <div className="replay-speed"><SpeedComparison sample={frame?.sample} points={history} time={time} issue={issue}>
    <TablesNotice tables={tables}/>
    {tables.data && tables.data.items.length > 0 && <form className="speed-form" onSubmit={e => {
      e.preventDefault(); if (available && gainValid && !speed.submitting && (changed || !ready && !running)) { apply(submit); void speed.calculate(submit); }
    }}>
      <TableSelect tables={tables.data.items} value={choice} onChange={setChoice}/>
      <div className="speed-fields">{fields.map(([k,label]) => <label key={k}>{label}<input type="number" step="any" min="0"
        value={value(k)} onChange={e => setEdits({...edits,[k]:e.target.value})}/></label>)}</div>
      <button className="button small" disabled={!available || !gainValid || speed.submitting || (!changed && (ready || running))}>{speed.submitting ? '受付中…' : running && !changed ? '速度制御を計算中…' : 'この設定で速度制御を計算'}</button>
    </form>}
    {config && <div className="planning-action">
      <span className="planning-sample">比較中の設定　{speed.meta?.table_run_id || config.table_sha256.slice(0,8)} · <GainsText gains={config}/></span>
      {speed.job && <button className="button small" onClick={() => onLog(speed.job!.id)}>{STATUS[speed.job.status] || speed.job.status} · ログ</button>}
      {(speed.failure || speed.frameFailure) && <button className="button small" onClick={speed.refresh}>速度比較を再読込</button>}
    </div>}
    {[...new Set([speed.meta?.propo_reason,speed.meta?.joy_reason])].filter(reason => reason && (!frame?.sample ||
      ![frame.sample.propo_reason,frame.sample.joy_reason].includes(reason))).map(reason =>
      <p key={reason} className="planning-note">{reason}</p>)}
  </SpeedComparison></div>;
}
