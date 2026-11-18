import type { GenerationSettings, LineKind, MapDocument, RacelineLimitKey } from './types';
import { Field, Icon } from './ui';

type NumericSetting = Exclude<keyof GenerationSettings, 'optimizer' | 'centerline_method'>;
const NUMBERS: Record<NumericSetting, { label: string; min: number; max?: number; step: number }> = {
  spacing: { label: '点の間隔 [m]', min: .05, max: 1, step: .01 },
  region_resolution: { label: '領域格子 [m]', min: .005, max: .25, step: .005 },
  centerline_max_points: { label: 'Centerの点数上限', min: 12, max: 1500, step: 1 },
  width: { label: '車幅 [m]', min: .01, step: .01 }, margin: { label: '余裕 [m]', min: 0, step: .01 },
  vehicle_length: { label: '全長 [m]', min: .1, max: 5, step: .01 },
  rear_axle_to_rear: { label: '後輪軸→後端 [m]', min: 0, max: 2, step: .0001 },
  curvature_limit: { label: '曲率上限 [1/m]', min: .01, max: 20, step: .01 },
  max_speed: { label: '速度上限 [m/s]', min: .01, step: .1 },
  lateral_accel: { label: '横加速度 [m/s²]', min: .01, step: .1 },
  accel: { label: '加速 [m/s²]', min: .01, step: .1 }, decel: { label: '減速 [m/s²]', min: .01, step: .1 },
};
const METHOD_NAMES: Record<string, string> = {
  region_equal_distance_v1: '領域・等距離線', tph_mincurv: '最小曲率', tph_mincurv_iqp: '反復最小曲率',
};
interface Props {
  doc: MapDocument; kind: LineKind; settings: GenerationSettings; busy: boolean; issue: string;
  onKind: (value: LineKind) => void; onSettings: (value: GenerationSettings) => void;
  onGenerate: (kind: LineKind) => void; onClearCalibration: () => void; dirty: boolean;
}
export function LineSettingsPanel({ doc, kind, settings, busy, issue, onKind, onSettings, onGenerate, onClearCalibration, dirty }: Props) {
  const calibration = kind === 'raceline' ? doc.raceline_calibration : undefined;
  function numberField(key: NumericSetting) {
    const definition = NUMBERS[key];
    const adopted = calibration && key in calibration.limits;
    return <Field key={key} label={definition.label + (adopted ? ' · 校正値' : '')}><input type="number" {...{ min: definition.min, max: definition.max, step: definition.step }} value={settings[key]} disabled={busy} readOnly={adopted} onChange={e => onSettings({ ...settings, [key]: Number(e.target.value) })}/></Field>;
  }
  const line = doc.lines[kind];
  return <div className="property-section">
    <div className="section-top"><h3>走行ライン生成</h3><span className="step-number">04</span></div>
    <Field label="生成するライン"><select value={kind} disabled={busy} onChange={e => onKind(e.target.value as LineKind)}><option value="centerline">Centerline · 中央線</option><option value="raceline">Raceline · 最小曲率</option><option value="custom">Customline · 編集した点列</option></select></Field>
    {kind === 'centerline' && <><div className="method-caption"><span className="line-dot"/><strong>領域・等距離線</strong><small>Centerlineの生成方式</small></div><div className="field-grid">{numberField('region_resolution')}{numberField('centerline_max_points')}</div></>}
    {kind === 'raceline' && <Field label="最適化方式"><select value={settings.optimizer} disabled={busy} onChange={e => onSettings({ ...settings, optimizer: e.target.value as GenerationSettings['optimizer'] })}><option value="mincurv">最小曲率（mincurv）</option><option value="mincurv_iqp">反復最小曲率（mincurv_iqp）</option></select></Field>}
    {calibration && <section className="adopted-limits" aria-label="採用済みの校正制限">
      <h4>車両校正から採用した制限</h4>
      <p className="small-note">{calibration.bag} · run {calibration.run_id}</p>
      <dl>{(Object.keys(calibration.limits) as RacelineLimitKey[]).map(key =>
        <div key={key}><dt>{NUMBERS[key].label}</dt><dd>{calibration.limits[key]}</dd></div>)}</dl>
      <p className="small-note">{line?.raceline_calibration?.applied_at === calibration.applied_at
        ? '保存済みの Raceline に反映済みです。' : line
        ? '表示中の Raceline は以前の設定です。再生成すると採用値が反映されます。'
        : '次の Raceline 生成に採用値を使います。'}</p>
      <button className="text-button" disabled={busy || dirty} onClick={onClearCalibration}>採用を解除して手動設定に戻す</button>
      {dirty && <p className="small-note">地図の編集を保存してから解除してください。</p>}
    </section>}
    {numberField('spacing')}
    <details className="generation-details"><summary>車体寸法・制約</summary><div className="field-grid">{(['width', 'vehicle_length', 'rear_axle_to_rear', 'margin', 'curvature_limit'] as NumericSetting[]).map(numberField)}</div></details>
    <details className="generation-details"><summary>速度・加減速</summary><div className="field-grid">{(['max_speed', 'lateral_accel', 'accel', 'decel'] as NumericSetting[]).map(numberField)}</div></details>
    {issue && <div className="generation-issue"><p>{issue}</p>{kind === 'raceline' && (issue === '先にCenterlineを生成してください' || issue === '領域方式でCenterlineを再生成してください') && <button className="text-button" disabled={busy} onClick={() => onGenerate('centerline')}>Centerlineを先に生成<Icon name="arrow" size={13}/></button>}</div>}
    <button className="button primary full" disabled={busy || !!issue} onClick={() => onGenerate(kind)}><Icon name="play" size={16}/>{dirty ? '保存してライン生成' : 'ラインを生成'}</button>
    {line && <p className="small-note">保存済み: {METHOD_NAMES[line.method || ''] || '以前の方式'} · {line.points.length} 点</p>}
    <p className="small-note">生成された速度は目標値です。実車の速度・制動性能は別に確認します。</p>
  </div>;
}
