import type { LineKind, MapDocument, Point } from './types';
import type { LiveStatus } from './workspace_types';

export const LINE_NAMES: Record<LineKind, string> = {
  raceline: 'Raceline', centerline: 'Centerline', custom: 'Customline',
};
export const LINE_CHOICES = Object.keys(LINE_NAMES) as LineKind[];
export interface PlanPose {x: number; y: number; yaw: number}
export interface PlanFrame {
  valid: boolean; reason: string; progress_m: number | null;
  lateral_error_m: number | null; target_speed_mps: number | null; estimated_speed_mps: number | null;
  vehicle_pose: PlanPose | null; trajectory: Point[]; trajectory_speeds_mps: number[];
}
export interface LivePlan extends PlanFrame {
  map_id: string; map_sha256: string; hdmap_sha256: string; session_id: string; epoch: number;
  line: LineKind; stamp: number; remaining_s: number; stale: boolean;
}
export function savedLines(doc: MapDocument | null): LineKind[] {
  return LINE_CHOICES.filter(key => (doc?.lines[key]?.points?.length ?? 0) >= 2);
}
export function planPose(value: PlanPose | null | undefined): PlanPose | null {
  return value && [value.x, value.y, value.yaw].every(Number.isFinite) ? value : null;
}
export function usablePlan(plan: PlanFrame | null | undefined): plan is PlanFrame {
  return !!plan?.valid && !!planPose(plan.vehicle_pose) &&
    [plan.lateral_error_m, plan.target_speed_mps, plan.estimated_speed_mps, plan.progress_m].every(Number.isFinite) &&
    Array.isArray(plan.trajectory) && plan.trajectory.length > 0 &&
    plan.trajectory.every(p => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite));
}

export function livePlanIssue(status: LiveStatus | null, timedOut: boolean) {
  if (!status?.connected) return '観測先への接続を待っています。';
  const plan = status.planning;
  if (!plan) return '走行計画の受信を待っています。';
  const loc = status.localization;
  if (!plan.valid && !plan.stale) return plan.reason || '計画が無効です。';
  if (!loc || loc.stale || loc.state !== 'tracking') return '自己位置の追跡待ちです。';
  if (timedOut || plan.stale || !Number.isFinite(plan.remaining_s) || plan.remaining_s <= 0)
    return '新しい計画の受信を待っています。';
  if (plan.map_id !== loc.map_id || plan.map_sha256 !== loc.map_sha256 || plan.hdmap_sha256 !== loc.hdmap_sha256 ||
      plan.session_id !== loc.session_id || plan.epoch !== loc.epoch)
    return '自己位置と計画の地図または追跡区間が一致しません。';
  return LINE_CHOICES.includes(plan.line) && usablePlan(plan) ? '' : '計画の姿勢・軌道・速度を確認できません。';
}

export function captureRoute(status: LiveStatus | null, timedOut: boolean) {
  const loc = status?.localization, drive = status?.auto_drive;
  const observedPlan = livePlanIssue(status, false) ? null : status?.planning ?? null;
  // 動的な前方軌道が取得中に失効しても、その応答で照合した静的なラインは表示する。
  const plan = timedOut ? null : observedPlan;
  const auto = drive?.publishers === 1 && drive.session_id ? drive.current : null;
  if (drive && drive.publishers > 1)
    return {line: null, plan: null, issue: '自動走行のラインの配信元が複数あります。'};
  if (auto && loc) {
    if (auto.map_id !== loc.map_id || (auto.speed &&
        (auto.speed.tracking_session_id !== loc.session_id || auto.speed.epoch !== loc.epoch)))
      return {line: null, plan: null, issue: '自動走行と自己位置の地図または追跡区間が一致しません。'};
    if (observedPlan && auto.line !== observedPlan.line)
      return {line: null, plan: null, issue: '自動走行と計画が選択しているラインが異なります。'};
    if (LINE_CHOICES.includes(auto.line)) return {line: auto.line, plan, issue: ''};
  }
  const sim = status?.identity?.environment === 'gazebo' && status.connected ? status.gazebo : null;
  const driveMap = loc?.map_id ?? sim?.ground_truth?.map_id;
  if (sim?.current && driveMap) {
    if (sim.current.map_id !== driveMap || (plan && plan.line !== sim.current.line))
      return {line: null, plan: null, issue: 'Gazebo と走行計画の地図またはラインが一致しません。'};
    if (LINE_CHOICES.includes(sim.current.line)) return {line: sim.current.line, plan, issue: ''};
  }
  return {line: observedPlan?.line ?? null, plan, issue: ''};
}
