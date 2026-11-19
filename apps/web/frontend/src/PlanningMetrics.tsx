import './planning.css';
import type { PlanFrame } from './planning_view';

export function PlanningMetrics({frame, reason}: {frame: PlanFrame | null; reason: string}) {
  const format = (n: number | null | undefined, signed = false) =>
    typeof n === 'number' && Number.isFinite(n) ? (signed && n > 0 ? '+' : '') + n.toFixed(2) : '—';
  return <div className="planning-readout">
    <dl className="planning-metrics">
      <div><dt>横ずれ <small>左が＋</small></dt><dd>{format(frame?.valid ? frame.lateral_error_m : null, true)} <small>m</small></dd></div>
      <div><dt>目標速度</dt><dd>{format(frame?.valid ? frame.target_speed_mps : null)} <small>m/s</small></dd></div>
      <div><dt>推定速度</dt><dd>{format(frame?.estimated_speed_mps)} <small>m/s</small></dd></div>
      <div><dt>ライン上の距離</dt><dd>{format(frame?.valid ? frame.progress_m : null)} <small>m</small></dd></div>
    </dl>
    {reason && <p className="planning-note" role="status">{reason}</p>}
  </div>;
}
