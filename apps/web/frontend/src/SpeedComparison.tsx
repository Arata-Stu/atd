import { useEffect, useState } from 'react';
import { api } from './api';
import { throttleInScale, type ThrottleScale } from './propo_profile';
import { finite, HISTORY_S, plotSegments, SOURCE_LABELS, sourceIntervals, speedPoint } from './speed_view';
import type { SpeedGains, SpeedMetric, SpeedPoint, SpeedSample } from './speed_view';
import './speed_control.css';

export interface SpeedTable {
  run_id: string; proposal_sha256: string; bag: string; speed_min_mps: number; speed_max_mps: number; direction: string;
}
export function useSpeedTables() {
  const [data, setData] = useState<{items: SpeedTable[]; defaults: SpeedGains; issues?: string[]} | null>(null);
  const [error, setError] = useState(''), [version, setVersion] = useState(0);
  useEffect(() => {
    const abort = new AbortController();
    api<NonNullable<typeof data>>('speed/tables', undefined, undefined, abort.signal).then(reply => {
      if (!abort.signal.aborted) { setData(reply); setError(''); }
    }).catch(e => { if (!abort.signal.aborted) { setData(null); setError(e.message); } });
    return () => abort.abort();
  }, [version]);
  return {data, error, refresh: () => setVersion(n => n + 1)};
}
export function TableSelect({tables, value, onChange}: {tables: SpeedTable[]; value: string; onChange: (id: string) => void}) {
  return <label className="speed-table-choice">校正済み FF 表
    <select value={value} onChange={e => onChange(e.target.value)}>
      <option value="">表を選択</option>
      {tables.map(t => <option key={t.proposal_sha256} value={t.proposal_sha256}>
        {t.run_id} · {t.bag} · {t.speed_min_mps.toFixed(2)}–{t.speed_max_mps.toFixed(2)} m/s · {t.proposal_sha256.slice(0,8)}
      </option>)}
    </select></label>;
}
export function TablesNotice({tables}: {tables: ReturnType<typeof useSpeedTables>}) {
  if (tables.error) return <p className="planning-note" role="alert">{tables.error} <button className="button small" onClick={tables.refresh}>表を再取得</button></p>;
  if (!tables.data) return <p className="planning-sample">採用済みの表を読み込み中…</p>;
  return <>{!tables.data.items.length && <p className="planning-note">車両校正でフィードフォワード表を「速度比較に採用」してください。</p>}
    {tables.data.issues?.map(text => <p className="planning-note" role="alert" key={text}>{text}</p>)}</>;
}
export function GainsText({gains}: {gains: SpeedGains}) {
  return <>Kp {gains.kp} · Ki {gains.ki} · Kd {gains.kd}</>;
}
const labels: Record<SpeedMetric, string> = {target_speed_mps:'目標速度', estimated_speed_mps:'推定速度',
  calculated_raw:'計算スロットル', propo_raw:'実プロポ入力', joy_throttle:'DualSense 操作入力'};
function n(value: number | null | undefined, digits: number) { return finite(value) ? value.toFixed(digits) : '—'; }
const throttleKeys = ['calculated_raw','propo_raw','joy_throttle'] as const;
const axes = {
  speed: {title:'速度 [m/s]', keys:['target_speed_mps','estimated_speed_mps'], digits:2},
  raw: {title:'スロットル [S.BUS CH2 raw]', keys:['calculated_raw','propo_raw'], digits:0},
  joy: {title:'DualSense スロットル [正規化 −1〜+1]', keys:['joy_throttle'], digits:0},
  normalized: {title:'スロットル比較 [正規化 −1〜+1]', keys:throttleKeys, digits:0},
  pulse: {title:'スロットル比較 [PWM 相当 µs]', keys:throttleKeys, digits:0},
} as const;
const xAt = (t: number, start: number) => 48 + (t - start) / HISTORY_S * 622;

function ComparisonPlot({points, end, axis, current}: {
  points: SpeedPoint[]; end: number; axis: keyof typeof axes; current?: SpeedPoint | null;
}) {
  const {keys, title, digits} = axes[axis];
  const start = end - HISTORY_S;
  const scale = axis === 'normalized' || axis === 'pulse' ? axis : null;
  const lines = scale
    ? throttleKeys.map(key => plotSegments(points, key, start, end, value => throttleInScale(key, value, scale)))
    : keys.map(key => plotSegments(points, key, start, end));
  const readings = scale && throttleKeys.map(key => throttleInScale(key, current?.[key] ?? null, scale));
  const values = lines.flat(2).map(p => p[1]);
  const min = values.length ? Math.min(...values) : 0;
  const max = values.length ? Math.max(...values) : 1;
  const margin = Math.max((max - min) * .12, axis === 'speed' ? .1 : 5);
  const fixedRange = axis === 'pulse' ? [1000, 2000] : (axis === 'joy' || axis === 'normalized') ? [-1, 1] : null;
  const lo = fixedRange?.[0] ?? (axis === 'speed' ? Math.min(0,min-margin) : Math.max(0,min-margin));
  const hi = fixedRange?.[1] ?? (max + margin);
  const y = (v: number) => 89 - (v - lo) / (hi - lo) * 78;
  return <figure className={'comparison-plot axis-' + axis}>
    <figcaption><strong>{title}</strong>
      {keys.map((key,i) => <span className={'chart-key key-' + i} key={key}>{labels[key]}
        {readings && <b>{n(readings[i], scale === 'pulse' ? 0 : 2)}{scale === 'pulse' && finite(readings[i]) ? ' µs' : ''}</b>}
      </span>)}</figcaption>
    <svg viewBox="0 0 690 113" role="img" aria-label={title + 'の時系列。右端が表示時刻、左端が5秒前。'}>
      {[lo,(lo+hi)/2,hi].map(v => <g key={v}><line x1="48" x2="670" y1={y(v)} y2={y(v)} stroke="#d8e0dc"/>
        {(values.length > 0 || fixedRange) && <text textAnchor="end" x="43" y={y(v)+4}>{n(v,digits)}</text>}</g>)}
      {lines.map((segments, i) => <g key={i} className={'chart-series series-' + i}>{segments.map((seg,j) =>
        seg.length === 1 ? <circle key={j} cx={xAt(seg[0][0],start)} cy={y(seg[0][1])} r="2.6"/> :
          <polyline key={j} points={seg.map(([t,v]) => xAt(t,start).toFixed(2)+','+y(v).toFixed(2)).join(' ')} />)}</g>)}
      <text x="48" y="108">−5 s</text><text x="670" y="108" textAnchor="end">表示時刻</text>
      {!values.length && <text x="355" y="53" textAnchor="middle">この時間帯のデータなし</text>}
    </svg>
  </figure>;
}
function SourceTimeline({points, end}: {points: SpeedPoint[]; end: number}) {
  const start = end - HISTORY_S, ranges = sourceIntervals(points,start,end);
  const sources = [...new Set(ranges.map(range => range.source))];
  return <figure className="comparison-plot source-plot">
    <figcaption><strong>操作経路の履歴</strong>{sources.map(source => <span key={source} className={'source-key source-' + source}>{SOURCE_LABELS[source]}</span>)}</figcaption>
    <svg viewBox="0 0 690 41" role="img" aria-label="操作経路の直近5秒。空白は観測の間隔が開いた区間。">
      <line x1="48" x2="670" y1="12" y2="12" stroke="#d8e0dc"/>
      {ranges.map((range, i) => <rect key={i} x={xAt(range.from,start)} y="6" height="12"
        width={Math.max(2,xAt(range.to,start)-xAt(range.from,start))} className={'source-' + range.source}>
        <title>{SOURCE_LABELS[range.source]}　{(range.from-end).toFixed(2)} 〜 {(range.to-end).toFixed(2)} 秒</title></rect>)}
      <text x="48" y="36">−5 s</text><text x="670" y="36" textAnchor="end">表示時刻</text>
    </svg>
  </figure>;
}
export function SpeedComparison({sample, points, time, issue, children}: {sample?: SpeedSample | null; points: SpeedPoint[];
  time: number; issue?: string; children?: React.ReactNode}) {
  const [scale,setScale] = useState<ThrottleScale>('original');
  const row = sample && !issue ? speedPoint(time, sample) : null;
  return <section className="speed-comparison" aria-label="速度制御の比較">
    <header><h4>速度制御</h4><span>FF ＋ PID · 観測のみ</span></header>
    {children}
    <p className="source-current"><span>観測した操作経路</span> <strong className={'source-key source-' + (row?.manual_source ?? 'unknown')}>{row ? SOURCE_LABELS[row.manual_source] : '—'}</strong></p>
    <dl className="planning-metrics speed-metrics">
      {(['target_speed_mps','estimated_speed_mps','calculated_raw','propo_raw','joy_throttle'] as const).map((key,i) =>
        <div key={key}><dt>{labels[key]}</dt><dd>{n(row?.[key], key === 'propo_raw' ? 0 : key === 'calculated_raw' ? 1 : 2)} <small>{i < 2 ? 'm/s' : key === 'joy_throttle' ? '正規化' : 'CH2 raw'}</small></dd></div>)}
    </dl>
    {issue && <p className="planning-note" role="status">{issue}</p>}
    {row && sample && <>
      {!sample.valid && <p className="planning-note" role="status">{sample.reason || '計算スロットルを取得できません。'}</p>}
      {row.propo_raw === null ? sample.reason !== sample.propo_reason && <p className="planning-note" role="status">{sample.propo_reason || '対応するプロポ入力がありません。'}</p> :
        finite(sample.propo_age_s) && <p className="planning-sample">プロポ CH2 の受信は速度の観測から {(sample.propo_age_s * 1000).toFixed(0)} ms 前</p>}
      {row.joy_throttle === null ? sample.reason !== sample.joy_reason && sample.propo_reason !== sample.joy_reason &&
        <p className="planning-note" role="status">{sample.joy_reason || '対応する DualSense 操作入力がありません。'}</p> :
        <p className="planning-sample">DualSense の操作要求{finite(sample.joy_age_s) && <>　速度観測の {(sample.joy_age_s * 1000).toFixed(0)} ms 前</>}
          {typeof sample.joy_enable_requested === 'boolean' && <>　enable 要求 {sample.joy_enable_requested ? 'あり' : 'なし'}</>}</p>}
      {sample.valid && <p className="planning-sample">FF {n(sample.feedforward_raw,1)}　P {n(sample.p_raw,1)}　I {n(sample.i_raw,1)}　D {n(sample.d_raw,1)} [raw]
        {sample.saturated && ' · 校正範囲で出力制限'}</p>}
    </>}
    <div className="comparison-plots">
      <ComparisonPlot points={points} end={time} axis="speed"/>
      <label className="throttle-scale">スロットル表示
        <select value={scale} onChange={e => setScale(e.target.value as ThrottleScale)}>
          <option value="original">元の入力単位</option>
          <option value="normalized">共通正規化（−1〜+1）</option>
          <option value="pulse">PWM 相当（µs）</option>
        </select>
      </label>
      {scale === 'original' ? <>
        <ComparisonPlot points={points} end={time} axis="raw"/>
        <ComparisonPlot points={points} end={time} axis="joy"/>
      </> : <>
        <p className="planning-sample">Rev6 既定換算（CH2 反転なし） · 前進側 +1＝1000 µs、中立 0＝1500 µs、制動・後退側 −1＝2000 µs</p>
        <ComparisonPlot points={points} end={time} axis={scale} current={row}/>
      </>}
      <SourceTimeline points={points} end={time}/>
    </div>
  </section>;
}
