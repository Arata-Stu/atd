import { useState } from 'react';
import { controlConfigIssue, controlFrameIssue, controlQuery } from './control_view';
import type { ControlConfig, ControlFrame } from './control_view';

function format(value: number | null | undefined, digits: number, signed = false) {
  if (value == null || !Number.isFinite(value)) return '—';
  return (signed && value > 0 ? '+' : '') + value.toFixed(digits);
}

export function ControlReadout({frame, issue = '', children}: {
  frame?: ControlFrame | null; issue?: string; children?: React.ReactNode;
}) {
  const reason = issue || controlFrameIssue(frame);
  const value = reason ? null : frame;
  const degrees = value?.steering_rad == null ? null : value.steering_rad * 180 / Math.PI;
  return <section className="control-readout" aria-label="Pure Pursuit の観測">
    <header><h4>Pure Pursuit</h4><span>操舵プレビュー</span></header>
    <dl className="planning-metrics control-metrics">
      <div><dt>先読み距離</dt><dd>{format(value?.requested_lookahead_m, 2)} <small>m</small></dd></div>
      <div><dt>追従点までの直線距離</dt><dd>{format(value?.actual_lookahead_m, 2)} <small>m</small></dd></div>
      <div><dt>曲率 <small>左が＋</small></dt><dd>{format(value?.curvature_per_m, 2, true)} <small>1/m</small></dd></div>
      <div><dt>等価操舵角 <small>左が＋</small></dt><dd>{format(degrees, 1, true)} <small>°</small></dd></div>
    </dl>
    {reason && <p className="planning-note" role="status">{reason}</p>}
    {value?.preview_limited && <p className="planning-note">前方軌道が先読み円の内側で終わるため、終端を追従点に使います。</p>}
    {value && degrees === null && <p className="planning-note">実測ホイールベースを設定すると等価操舵角も表示されます。</p>}
    {children}
  </section>;
}

export function LiveControlConfig({config}: {config: ControlConfig}) {
  return <details className="control-config"><summary>Jetson の追従設定</summary>
    <p>先読み距離は {config.min_lookahead_m} m ＋ |推定速度| × {config.lookahead_time_s} s、上限 {config.max_lookahead_m} m。</p>
    <p>実測ホイールベース {config.wheelbase_m === null ? '未設定' : format(config.wheelbase_m * 1000, 1) + ' mm'}</p>
  </details>;
}

export function ReplayControlConfig({defaults, config, apply}: {
  defaults: ControlConfig; config: ControlConfig | null; apply: (config: ControlConfig | null) => void;
}) {
  const fieldsFrom = (c: ControlConfig) => ({min: String(c.min_lookahead_m), time: String(c.lookahead_time_s),
    max: String(c.max_lookahead_m), wheelbase: c.wheelbase_m === null ? '' : String(c.wheelbase_m * 1000)});
  const [draft, setDraft] = useState(() => fieldsFrom(config ?? defaults));
  const parse = (s: string) => s.trim() ? Number(s) : NaN;
  const candidate: ControlConfig = {min_lookahead_m: parse(draft.min), lookahead_time_s: parse(draft.time),
    max_lookahead_m: parse(draft.max), wheelbase_m: draft.wheelbase ? parse(draft.wheelbase) / 1000 : null};
  const issue = controlConfigIssue(candidate);
  const changed = controlQuery(config ?? defaults) !== controlQuery(candidate);
  const field = (key: keyof typeof draft, label: string) => <label>{label}
    <input type="number" step="any" value={draft[key]} onChange={e => setDraft({...draft, [key]: e.target.value})}/></label>;
  return <details className="control-config"><summary>この bag で先読み設定を試す</summary>
    <p>最小距離に「|推定速度| × 先読み時間」を加え、最大距離で制限します。</p>
    <form onSubmit={e => { e.preventDefault(); if (!issue && changed) apply(candidate); }}>
      <div className="control-fields">{field('min', '最小距離 [m]')}{field('time', '先読み時間 [s]')}
        {field('max', '最大距離 [m]')}{field('wheelbase', '実測ホイールベース [mm]（任意）')}</div>
      {issue && <p className="planning-note" role="status">{issue}</p>}
      <div className="planning-action"><button className="button small" disabled={!!issue || !changed}>この再生に適用</button>
        <button className="button small quiet" type="button" disabled={!config && !changed} onClick={() => {
          setDraft(fieldsFrom(defaults));
          apply(null);
        }}>初期値に戻す</button>
        {changed && !issue && <span className="planning-sample">未適用の変更あり</span>}</div>
    </form>
  </details>;
}
