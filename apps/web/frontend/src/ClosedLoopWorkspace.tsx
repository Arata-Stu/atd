import { useEffect, useMemo, useState } from 'react';
import { api } from './api';
import { around, chartsFor, poseAt, runKey, statuses, timeBounds, valueAt } from './closed_loop';
import type { ClosedLoopDetail, GazeboRun, XY } from './closed_loop';
import { ClosedLoopMap, TimelinePlot } from './ClosedLoopPlots';
import { SpeedLegend } from './SpeedLegend';
import { speedScale } from './raceline_speed';
import { Empty, Field, Icon } from './ui';
import './replay_errors.css';
import './closed_loop.css';

function label(run:GazeboRun) {
  const laps = run.laps_completed === null && run.laps_requested === null ? '' :
    ' · '+(run.laps_completed ?? '—')+'/'+(run.laps_requested ?? '—')+' 周';
  return (statuses[run.status] ?? run.status) + laps;
}

function RunTimeline({data}: {data:ClosedLoopDetail}) {
  const full = useMemo(()=>timeBounds(data),[data]);
  const charts = useMemo(()=>chartsFor(data),[data]);
  const [focus,setFocus] = useState(data.contact_s !== null);
  const [cursor,setCursor] = useState<number|null>(null);
  const [showCurrent,setShowCurrent] = useState(false);
  const lineScale = useMemo(()=>data.run.line === 'raceline' ? speedScale(data.reference_speeds_mps,data.reference.length) : null,[data]);
  const event = data.contact_s ?? full?.[1] ?? 0;
  const span = useMemo(()=>full && focus ? around(event,full) : full,[full,focus,event]);
  if (!span) return <p className="cl-missing" role="status">この試行には時刻付きの走行データがありません。</p>;
  const at = Math.min(span[1],Math.max(span[0],cursor ?? data.contact_s ?? data.anchor_s ?? span[0]));
  const lineAvailable = data.reference.length > 0;
  const currentMap = data.reference_source === 'current_map';
  const showLine = lineAvailable && (!currentMap || showCurrent);
  const isContact = data.contact_s !== null;
  const eventLabel = isContact ? '接触前後 3 秒' : '記録終了前 3 秒';
  function changeWindow(local:boolean) {
    setFocus(local);
    if (local) setCursor(event);
  }
  const gt = poseAt(data.samples,at), est = poseAt(data.estimates,at);
  const speed = valueAt(data.samples,at,p=>p.speed_mps);
  const controlImage = valueAt(data.samples,at,p=>p.control_image_s,'step');
  const xyText = (xy:XY|null) => xy ? xy.map(v=>v.toFixed(3)).join(', ')+' m' : 'この時刻の記録なし';
  return <>
    <div className="cl-timebar">
      <div className="cl-window" aria-label="表示範囲">
        <button className="button small" aria-pressed={!focus} onClick={()=>changeWindow(false)}>全走行</button>
        <button className="button small" aria-pressed={focus} onClick={()=>changeWindow(true)}>{eventLabel}</button>
      </div>
      {isContact && <button className="bench-link cl-jump" onClick={()=>setCursor(event)}>
        × 接触時刻 {event.toFixed(3)} s へ</button>}
    </div>
    <div className="replay-panes cl-panes">
      <section className="replay-card cl-course">
        <header><h3>走行軌跡</h3><span>Gazebo world · 後輪軸</span></header>
        <div className="replay-legend cl-legend">
          <span className="replay-truth">Gazebo GT</span><span className="replay-estimate">推定軌跡</span>
          {showLine && !lineScale && <span className="cl-reference">{data.run.line ?? '選択ライン'}{currentMap && '（現在の地図）'}</span>}
          {data.boundaries.left.length>0 && <span className="cl-wall">舗装境界</span>}
        </div>
        {currentMap && lineAvailable && <label className="cl-toggle"><input type="checkbox" checked={showCurrent} onChange={e=>setShowCurrent(e.target.checked)}/>
          現在の地図のラインを参考表示（走行時との一致は未確認）</label>}
        {!lineAvailable && <p className="cl-missing">この試行に重ねられる選択ラインがありません。</p>}
        {showLine && lineScale && <SpeedLegend scale={lineScale}/>}
        <ClosedLoopMap run={data} span={span} focus={focus} showLine={showLine} lineScale={showLine ? lineScale : null} at={at} select={setCursor}/>
        <p className="cl-guide">+X 右・+Y 上{data.footprint && ' · 矩形は選択時刻の GT 車体外形'}{isContact && ' · × は接触判定時の後輪軸'}</p>
        <label className="replay-slider"><span>シミュレーション時刻 <output>{at.toFixed(3)} s</output></span>
          <input aria-label="シミュレーション時刻（秒）" type="range" min={span[0]} max={span[1]} step="any" value={at}
            disabled={span[0]===span[1]} onChange={e=>setCursor(e.currentTarget.valueAsNumber)}/>
        </label>
        <dl className="cl-state">
          <dt>GT 位置 (x, y)</dt><dd>{xyText(gt)}</dd>
          <dt>推定位置 (x, y)</dt><dd>{xyText(est)}</dd>
          <dt>GT 速度</dt><dd>{speed===null ? '—' : speed.toFixed(2)+' m/s'}</dd>
          <dt>制御が参照した画像時刻</dt><dd>{controlImage===null ? '—' : controlImage.toFixed(3)+' s'}</dd>
        </dl>
        {data.anchor_s !== null && <p className="cl-guide">{data.anchor_s.toFixed(3)} s の後輪軸位置と向きで推定座標を GT に合わせています。</p>}
      </section>
      <section className="replay-card cl-plots">
        <header><h3>走行時系列</h3><span>共通時刻 (s)</span></header>
        <p className="cl-guide">グラフをクリックまたはドラッグして時刻を選択できます。「—」は対応する記録がない時刻です。</p>
        {charts.map(chart=><TimelinePlot key={chart.label} chart={chart} span={span} at={at} contactTime={data.contact_s} select={setCursor}/>)}
        <p className="cl-guide">連続値は 0.25 秒以内の観測点間を補間します。指令角と指令採用時の遅れは次の観測まで直前の値を表示します。</p>
      </section>
    </div>
  </>;
}

function Result({data}: {data:ClosedLoopDetail}) {
  const {run} = data;
  return <>
    <div className="cl-summary"><strong>{label(run)}</strong><span>地図 {run.map_id ?? '未記録'}</span>
      <span>選択ライン {run.line ?? '未記録'}</span>{run.max_speed_mps!==null && <span>速度上限 {run.max_speed_mps.toFixed(2)} m/s</span>}
      {run.reason && <p>{run.reason}</p>}
    </div>
    {data.warnings.length>0 && <details className="cl-warnings">
      <summary>記録元・欠測について（{data.warnings.length} 件）</summary>
      <ul>{data.warnings.map((w,i)=><li key={i}>{w}</li>)}</ul>
    </details>}
    <RunTimeline data={data}/>
  </>;
}

export default function ClosedLoopWorkspace() {
  const [runs,setRuns] = useState<GazeboRun[]|null>(null);
  const [id,setId] = useState(''), [reload,setReload] = useState(0);
  const [data,setData] = useState<ClosedLoopDetail|null>(null);
  const [listError,setListError] = useState(''), [error,setError] = useState('');
  const selected = runs?.find(r=>runKey(r)===id) ?? runs?.[0];
  const key = selected ? runKey(selected) : '';
  useEffect(()=>{
    const req = new AbortController(); setListError('');
    api<{items:GazeboRun[]}>('simulation/gazebo-runs',undefined,undefined,req.signal)
      .then(({items})=>{if(!req.signal.aborted)setRuns(items);})
      .catch(e=>{if(!req.signal.aborted)setListError(e.message);});
    return ()=>req.abort();
  },[reload]);
  useEffect(()=>{
    const req = new AbortController(); setData(null); setError('');
    if (key) api<ClosedLoopDetail>('simulation/gazebo-runs/'+key.split('/').map(encodeURIComponent).join('/'),undefined,undefined,req.signal)
      .then(result=>{
        if (req.signal.aborted) return;
        if (runKey(result.run)===key) setData(result);
        else setError('選択した試行と取得した記録が一致しません。');
      }).catch(e=>{if(!req.signal.aborted)setError(e.message);});
    return ()=>req.abort();
  },[key,reload]);
  return <div className="content-page replay-errors-page cl-page">
    <div className="page-intro"><div><h2>Gazebo 閉ループ走行</h2><p>保存した試行の自己位置推定・ライン追従・境界接触を同じ時刻で確認します。</p></div>
      <button className="button" onClick={()=>setReload(n=>n+1)}><Icon name="refresh" size={15}/>更新</button></div>
    {listError ? <p role="alert" className="replay-error">{listError}</p> : runs===null ? <p role="status">保存結果を読み込んでいます…</p> :
      !runs.length ? <section className="content-card"><Empty title="閉ループ走行の記録がありません" icon="chart"
        text="Notebook で atd gazebo closed-loop を実行すると、ここで試行を選べます。"/></section> :
        <><Field label="保存した閉ループ走行"><select aria-label="保存した閉ループ走行" value={key} onChange={e=>setId(e.target.value)}>
          {runs.map(run=><option key={runKey(run)} title={run.id} value={runKey(run)}>{run.started ? new Date(run.started * 1000).toLocaleString('ja-JP', { hour12: false }) : run.id} · {run.circuit} / {run.map_id ?? '地図未記録'} / {run.line ?? 'ライン未記録'} · {label(run)}</option>)}
        </select></Field>
          {error ? <p role="alert" className="replay-error">{error}</p> : data && runKey(data.run)===key ? <Result key={key} data={data}/> :
            <p role="status">走行軌跡と時系列を読み込んでいます…</p>}
        </>}
  </div>;
}
