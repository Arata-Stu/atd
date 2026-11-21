import { useEffect, useMemo, useState, type ReactNode } from 'react';
import { api } from './api';
import { CourseMap, ErrorChart, metrics } from './ReplayErrorPlots';
import { errorAt, nearest, recordedLaps, selectWindow } from './replay_errors';
import type { ReplayDetail, ReplayRun } from './replay_errors';
import { Empty, Field, Icon } from './ui';
import './replay_errors.css';

const statuses: Record<string,string> = {running:'再生中', complete:'再生完了', partial:'途中終了', failed:'失敗'};
function statusLabel(s: string) { return statuses[s] ?? s; }
function dateLabel(iso: string) {
  const date = new Date(iso);
  return Number.isFinite(date.getTime()) ? date.toLocaleString('ja-JP', {hour12:false}) : iso;
}

function ReplayResult({run, picker}: {run: ReplayDetail; picker: ReactNode}) {
  const laps = useMemo(() => recordedLaps(run),[run]);
  const [sectionId,setSection] = useState('');
  const section = run.sections.find(area => area.id === sectionId);
  const wraps = !!section && section.start_m > section.end_m;
  const [lap,setLap] = useState(laps[0] ?? null);
  const [zoom,setZoom] = useState(false);
  const [cursor,setCursor] = useState<number | null>(null);
  const view = useMemo(() => selectWindow(run, section, lap), [run, section, lap]);
  const at = Math.max(view.min, Math.min(view.max, cursor ?? view.samples[0]?.at ?? view.min));
  const estimate = errorAt(view,at);
  const gt = nearest(view.reference,at)?.point;
  const location = estimate?.point ?? gt;
  const passages = wraps ? [...new Set(laps.flatMap(n => [n-1,n]))].sort((a,b) => a-b) : laps;
  function passageName(n: number) {
    const pre = laps.includes(n), post = laps.includes(n+1);
    if (pre && post) return n + ' → ' + (n+1) + ' 周目';
    return pre ? n + ' 周目 · ゴール手前' : (n+1) + ' 周目 · スタート直後';
  }
  function changeSection(id: string) {
    setSection(id); setCursor(null); setZoom(false);
    const next = run.sections.find(area => area.id === id);
    if ((!next || next.start_m <= next.end_m) && lap !== null && !laps.includes(lap)) setLap(laps[0] ?? null);
  }
  return <>
    <div className="replay-selectors">
      {picker}
      <Field label="コース区間"><select aria-label="コース区間" value={sectionId} onChange={e => changeSection(e.target.value)}>
        <option value="">全コース</option>{run.sections.map(area => <option key={area.id} value={area.id}>{area.label}</option>)}
      </select></Field>
      {laps.length > 0 && <Field label={wraps ? '計測線の通過' : '周回'}><select aria-label={wraps ? '計測線の通過' : '周回'} value={lap ?? ''} onChange={e => {
        setLap(Number(e.target.value)); setCursor(null);
      }}>{passages.map(n => <option key={n} value={n}>{wraps ? passageName(n) : n + ' 周目'}</option>)}</select></Field>}
      <span className="replay-run-meta">
        {run.circuit && <strong>{run.circuit}</strong>} {statusLabel(run.status)} · {run.sample_count.toLocaleString()} 姿勢
      </span>
    </div>
    {(run.status === 'failed' || run.status === 'partial') && run.reason &&
      <p className={run.status === 'failed' ? 'replay-error' : 'replay-status'} role="status">{statusLabel(run.status)} · {run.reason}</p>}
    {!run.sample_count && <p className="replay-status">同じ画像時刻の正解と照合できた推定姿勢は 0 件です。</p>}
    {!view.reference.length && !view.samples.length ? <div className="content-card">
      <Empty icon="chart" title="この範囲に記録がありません" text="別の区間または走行結果を選択してください。"/>
    </div> : <>
      <div className="replay-panes">
        <section className="replay-card">
          <header><h3>コース図</h3>{section && <button className="button small" aria-pressed={zoom} onClick={() => setZoom(z => !z)}>
            <Icon name="expand" size={14}/>{zoom ? '全体表示' : '区間拡大'}</button>}</header>
          <div className="replay-legend"><span className="replay-truth">Gazebo 正解</span><span className="replay-estimate">cuVSLAM 推定</span></div>
          <CourseMap course={run.reference} view={view} zoom={zoom} selected={estimate?.point} reference={gt} select={setCursor}/>
          <label className="replay-slider"><span>比較地点 <b>{at.toFixed(2)} m</b></span>
            <input type="range" aria-label="比較地点（メートル）" min={view.min} max={view.max} step="any" disabled={view.min === view.max}
              value={at} onChange={e => setCursor(e.target.valueAsNumber)}/>
          </label>
          <div className="replay-location">{location && <>{location.lap !== null && <span>{location.lap} 周目</span>}
            <span>累積 {location.distance_m.toFixed(2)} m</span><span>GT 記録開始から {location.time_s.toFixed(2)} s</span></>}
            {!estimate && <strong>対応する推定なし</strong>}
          </div>
          <dl className="replay-readout">{metrics.map(metric => <div key={metric.key}><dt>{metric.label}</dt><dd>
            {estimate ? estimate.point[metric.key].toFixed(metric.unit === '°' ? 1 : 2) : '—'} <small>{metric.unit}</small>
          </dd></div>)}</dl>
        </section>
        <section className="replay-card">
          <header><h3>地点ごとの誤差</h3><span>{view.axis} (m)</span></header>
          {view.gaps.length > 0 && <p className="replay-gap-note"><i/>斜線は対応する推定がない区間</p>}
          {metrics.map(metric => <ErrorChart key={metric.key} metric={metric} view={view} at={at} selection={estimate} select={setCursor}/>)}
        </section>
      </div>
      <p className="replay-instruction">コース図・グラフをクリックするか、比較地点のスライダーで位置を選択できます。</p>
    </>}
  </>;
}

export default function ReplayErrorsWorkspace() {
  const [runs,setRuns] = useState<ReplayRun[] | null>(null), [id,setId] = useState('');
  const [data,setData] = useState<ReplayDetail | null>(null);
  const [listError,setListError] = useState(''), [runError,setRunError] = useState('');
  const [reload,setReload] = useState(0);
  const selected = runs?.find(run => run.id === id) ?? runs?.[0];
  const selectedId = selected?.id;
  useEffect(() => {
    const request = new AbortController();
    setListError('');
    api<{items:ReplayRun[]}>('replay/runs', undefined, undefined, request.signal).then(({items}) => {
      if (!request.signal.aborted) setRuns(items);
    }).catch(e => { if (!request.signal.aborted) { setRuns(null); setListError(e.message); } });
    return () => request.abort();
  },[reload]);
  useEffect(() => {
    const request = new AbortController();
    setData(null); setRunError('');
    if (selectedId) api<ReplayDetail>('replay/runs/' + encodeURIComponent(selectedId),undefined,undefined,request.signal).then(result => {
      if (request.signal.aborted) return;
      if (result.id !== selectedId) setRunError('選択した記録と取得した記録が一致しません。');
      else setData(result);
    }).catch(e => { if (!request.signal.aborted) setRunError(e.message); });
    return () => request.abort();
  },[selectedId,reload]);
  const picker = <div className="replay-picker"><Field label="保存した再生結果">
    <select aria-label="保存した再生結果" title={selected?.bag} value={selectedId} onChange={e => setId(e.target.value)}>
      {runs?.map(item => <option value={item.id} key={item.id}>{item.bag} · {dateLabel(item.created_at)} · {statusLabel(item.status)}</option>)}
    </select>
  </Field></div>;
  return <div className="content-page replay-errors-page">
    <div className="page-intro"><div><h2>VSLAM 軌跡誤差</h2><p>初期姿勢を合わせた Gazebo と cuVSLAM の車体位置・向きを、同じ画像時刻で比較します。</p></div>
      <button className="button" onClick={() => setReload(n => n+1)}><Icon name="refresh" size={16}/>更新</button></div>
    {listError ? <p role="alert" className="replay-error">{listError}</p> : runs === null ? <p role="status">保存結果を読み込んでいます…</p> :
      !runs.length ? <section className="content-card"><Empty icon="chart" title="再生結果はまだありません"
        text="Notebook で atd replay を実行して、Gazebo の正解軌跡を含む bag を再生してください。"/></section> :
      <>
        {runError ? <>{picker}<p role="alert" className="replay-error">{runError}</p></> :
          data && data.id === selectedId ? <ReplayResult key={selectedId} run={data} picker={picker}/> :
          <>{picker}<p role="status">軌跡を読み込んでいます…</p></>}
      </>}
  </div>;
}
