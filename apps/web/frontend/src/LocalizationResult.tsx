import { useCallback, useMemo, useState } from 'react';
import { LINE_NAMES, savedLines } from './planning_view';
import { ReplayPlanControls, useReplayPlanning } from './ReplayPlanning';
import type { Job, LineKind } from './types';
import { controlOverlay } from './control_view';
import type { ControlConfig } from './control_view';
import type { SpeedSettings } from './ReplaySpeed';
import { MapCanvas } from './MapCanvas';
import { ReferenceLineLegend } from './SpeedLegend';
import { GAP_LABELS, OUTCOMES, resultMapIssue, seconds, validationOverlay } from './localization_validation';
import type { Observation, ValidationResult } from './localization_validation';
import { trackingLabels } from './localization_view';

function Endpoint({title, sample}: {title: string; sample: Observation | null}) {
  return <div><dt>{title}</dt><dd>{sample
    ? <>{seconds(sample.time_s)} · X {sample.point[0].toFixed(2)} / Y {sample.point[1].toFixed(2)} m</>
    : '観測位置なし'}</dd></div>;
}

interface Props {result: ValidationResult; jobs: Job[]; onLog: (id: string) => void}
export function LocalizationResult(props: Props) {
  const lines = useMemo(() => savedLines(props.result.document), [props.result.document]);
  const [line, chooseLine] = useState<LineKind>(() => lines[0] || 'raceline');
  const [time, seek] = useState(props.result.first_tracking_delay_s ?? 0);
  const [config, applyConfig] = useState<ControlConfig | null>(null);
  const [speedSettings,setSpeedSettings] = useState<SpeedSettings | null>(null);
  return <LineResult speedSettings={speedSettings} setSpeedSettings={setSpeedSettings} key={line} config={config} applyConfig={applyConfig} {...props} lines={lines} line={line} chooseLine={chooseLine} time={time} seek={seek}/>;
}
function LineResult({result, jobs, onLog, line, lines, chooseLine, time, seek, config, applyConfig, speedSettings, setSpeedSettings}: Props & {
  speedSettings: SpeedSettings | null; setSpeedSettings: (value:SpeedSettings | null) => void;
  config: ControlConfig | null; applyConfig: (c: ControlConfig | null) => void;
  line: LineKind; lines: LineKind[]; chooseLine: (line: LineKind) => void; time: number; seek: (t: number) => void;
}) {
  const [selection, setSelection] = useState<{index: number} | null>(null);
  const selected = selection?.index ?? null;
  const setSelected = (index: number | null) => {
    setSelection(index === null ? null : {index});
    if (index !== null) {
      const gap = result.gaps[index];
      seek((gap.start_s + gap.end_s) / 2);
    }
  };
  const seekFromTimeline = useCallback((value: number) => {
    setSelection(null);
    seek(value);
  }, [seek]);
  const overlay = useMemo(() => validationOverlay(result), [result]);
  const issue = resultMapIssue(result);
  const planningIssue = issue || (!lines.length ? '検証時の地図に生成・保存された走行ラインがありません。' :
    !result.pose_samples ? '軌道を計算できる姿勢が記録されていません。' : '');
  const plan = useReplayPlanning(result.id, line, planningIssue, time, jobs, config);
  const lastGap = selected === null ? undefined : result.gaps[selected];
  const diagnostics = result.states;
  return <section className="content-card validation-review" aria-label="自己位置検証の結果">
    <div className="card-heading"><div><h3>{result.map_id} の検証結果</h3><span>{result.bag}</span></div>
      <strong className={'validation-outcome outcome-' + result.outcome}>{OUTCOMES[result.outcome] || result.outcome}</strong></div>
    {result.error && <div className="inline-note" role="alert">{result.error}</div>}
    <dl className="validation-metrics">
      <div><dt>追跡時間率</dt><dd>{result.tracking_time_fraction === null ? '—' : (result.tracking_time_fraction * 100).toFixed(1) + '%'}</dd></div>
      <div><dt>初回の位置確定</dt><dd>{seconds(result.first_tracking_delay_s)}</dd></div>
      <div><dt>ステレオ観測時間</dt><dd>{seconds(result.stereo_duration_s)}</dd></div>
      <div><dt>有効な姿勢</dt><dd>{result.pose_samples === null ? '—' : result.pose_samples.toLocaleString() + ' 点'}</dd></div>
    </dl>
    <ReplayPlanControls speedSettings={speedSettings} setSpeedSettings={setSpeedSettings} line={line} lines={lines} choose={chooseLine} time={time} seek={seekFromTimeline} selection={selection} issue={planningIssue} plan={plan} onLog={onLog} config={config} applyConfig={applyConfig}/>
    <div className="validation-review-grid">
      <section className="validation-map" aria-label={LINE_NAMES[line] + ' と推定軌跡'}>
        <h4>{LINE_NAMES[line]} と推定軌跡</h4>
        {issue ? <p className="inline-note" role="status">{issue}</p> :
          <><div className="validation-map-frame">
            <MapCanvas doc={result.document} mode="pan" overlay={overlay} selectedGap={selection} onSelectGap={setSelected}
              referenceLine={line} controlTarget={plan.frame?.valid ? controlOverlay(plan.frame.control, plan.frame.vehicle_pose) : undefined} pose={plan.frame?.vehicle_pose} forwardPath={plan.frame?.valid ? plan.frame.trajectory : undefined}/>
          </div>
          <div className="validation-legend" aria-label="地図の凡例">
            <ReferenceLineLegend raceline={line === 'raceline' ? result.document?.lines.raceline : undefined}>
              <span><i className="race"/>{LINE_NAMES[line]}</span>
            </ReferenceLineLegend>
            <span><i className="plan-path"/>前方軌道</span><span><i className="control-target"/>追従点</span><span><i className="track"/>推定軌跡</span>
            <span><i className="before"/>途絶前の最終姿勢</span><span><i className="after"/>復帰後の初回姿勢</span>
          </div>
          </>}
        {result.pose_samples === 0 && <p className="small-note">この走行では地図上の姿勢を確定できませんでした。</p>}
        {result.pose_samples !== null && result.displayed_samples < result.pose_samples && !issue &&
          <p className="small-note">描画 {result.displayed_samples.toLocaleString()} / {result.pose_samples.toLocaleString()} 点（区間の始点・終点を保持）</p>}
      </section>
      <section className="validation-gap-panel" aria-label="追跡が途切れた区間">
        <div className="validation-gap-title"><h4>追跡が途切れた区間</h4>{selected !== null && <button className="button small" onClick={() => setSelected(null)}>全区間を表示</button>}</div>
        {!!result.gaps.length && <p className="small-note">時刻はステレオ観測開始からの秒数です。{!issue && '区間を選ぶと前後の観測位置を拡大します。'}</p>}
        {result.gaps.length ? <div className="validation-gap-list">{result.gaps.map((gap, i) => <button type="button"
          key={i} aria-pressed={selected === i} className={selected === i ? 'selected' : ''} onClick={() => setSelected(i)}>
          <span><strong>{GAP_LABELS[gap.kind]}</strong> {gap.start_s.toFixed(2)} 〜 {seconds(gap.end_s)}</span>
          <span>{gap.after ? '復帰あり' : '末尾まで復帰なし'}</span>
        </button>)}</div> : <p className="small-note">{result.pose_samples === null ? '完了した追跡区間の集計がありません。' :
          result.pose_samples === 0 ? '途絶前の観測位置がありません。' : '初回の位置確定後に追跡の途絶は記録されていません。'}</p>}
        {lastGap && <div className="validation-gap-detail" aria-live="polite">
          {lastGap.reason && <p>{lastGap.reason}</p>}
          <dl><Endpoint title="途絶前の最終姿勢" sample={lastGap.before}/><Endpoint title="復帰後の初回姿勢" sample={lastGap.after}/></dl>
          {!lastGap.after && <p>この区間では最後の有効な位置までを表示します。</p>}
        </div>}
      </section>
    </div>
    {diagnostics.length > 0 && <details className="validation-events">
      <summary>追跡状態の履歴（{diagnostics.length} 件）</summary>
      <div className="validation-event-table"><table><thead><tr><th>経過 [s]</th><th>状態</th><th>理由</th></tr></thead>
        <tbody>{diagnostics.map((event, i) => <tr key={i}><td>{event.time_s.toFixed(2)}</td>
          <td>{trackingLabels[event.state as keyof typeof trackingLabels] || event.state}</td><td>{event.reason}</td></tr>)}</tbody></table></div>
    </details>}
  </section>;
}
