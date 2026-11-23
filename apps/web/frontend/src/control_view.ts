import type { Point } from './types';
import type { LivePlan, PlanPose } from './planning_view';
import type { LiveStatus } from './workspace_types';

export interface ControlConfig {
  min_lookahead_m: number; lookahead_time_s: number; max_lookahead_m: number; wheelbase_m: number | null;
}
export interface ControlFrame {
  controller: string; valid: boolean; reason: string; target: Point | null;
  requested_lookahead_m: number | null; actual_lookahead_m: number | null;
  curvature_per_m: number | null; steering_rad: number | null; preview_limited: boolean; config: ControlConfig;
}
export interface LiveControl extends ControlFrame, Pick<LivePlan,
  'map_id' | 'map_sha256' | 'hdmap_sha256' | 'session_id' | 'epoch' | 'line' | 'stamp' | 'remaining_s' | 'stale'> {
  vehicle_pose: PlanPose | null;
}
export interface ControlOverlay { origin: Point; target: Point }

export function controlConfigIssue(c: ControlConfig | null | undefined): string {
  if (!c) return '追従設定を確認できません。';
  if (!Number.isFinite(c.min_lookahead_m) || c.min_lookahead_m <= 0) return '最小距離には正の値を入力してください。';
  if (!Number.isFinite(c.max_lookahead_m) || c.max_lookahead_m < c.min_lookahead_m) return '最大距離は最小距離以上にしてください。';
  if (!Number.isFinite(c.lookahead_time_s) || c.lookahead_time_s < 0) return '先読み時間には 0 以上の値を入力してください。';
  if (c.wheelbase_m !== null && (!Number.isFinite(c.wheelbase_m) || c.wheelbase_m <= 0))
    return '実測ホイールベースには正の値を入力してください。';
  return '';
}
export function controlQuery(config: ControlConfig | null): string {
  if (!config) return '';
  const q = new URLSearchParams();
  for (const name of ['min_lookahead_m', 'lookahead_time_s', 'max_lookahead_m', 'wheelbase_m'] as const) {
    if (config[name] !== null) q.set(name, String(config[name]));
  }
  return '&' + q;
}

export function controlFrameIssue(frame: ControlFrame | null | undefined): string {
  if (!frame) return '追従計算の結果を待っています。';
  if (!frame.valid) return frame.reason || '追従計算が無効です。';
  if (frame.controller !== 'pure_pursuit') return '追従方式を確認できません。';
  const configIssue = controlConfigIssue(frame.config);
  if (configIssue) return configIssue;
  if (!Array.isArray(frame.target) || frame.target.length !== 2 || !frame.target.every(Number.isFinite) ||
      typeof frame.requested_lookahead_m !== 'number' || !Number.isFinite(frame.requested_lookahead_m) || frame.requested_lookahead_m <= 0 ||
      typeof frame.actual_lookahead_m !== 'number' || !Number.isFinite(frame.actual_lookahead_m) || frame.actual_lookahead_m <= 0 ||
      !Number.isFinite(frame.curvature_per_m) || typeof frame.preview_limited !== 'boolean' ||
      (frame.config.wheelbase_m === null ? frame.steering_rad !== null : !Number.isFinite(frame.steering_rad)))
    return '追従点・距離・操舵の計算結果を確認できません。';
  return '';
}

export function liveControlIssue(status: LiveStatus | null, timedOut: boolean): string {
  const control = status?.control, plan = status?.planning;
  if (!status?.connected || !control) return '追従計算の受信を待っています。';
  if (!control.stale && !control.valid) return controlFrameIssue(control);
  if (timedOut || control.stale || !Number.isFinite(control.remaining_s) || control.remaining_s <= 0)
    return '新しい追従計算の受信を待っています。';
  if (!plan || plan.map_id !== control.map_id || plan.map_sha256 !== control.map_sha256 ||
      plan.hdmap_sha256 !== control.hdmap_sha256 || plan.session_id !== control.session_id ||
      plan.epoch !== control.epoch || plan.line !== control.line)
    return '計画と追従計算の地図・追跡区間・ラインが一致しません。';
  const pose = control.vehicle_pose;
  if (!pose || ![pose.x, pose.y, pose.yaw, control.stamp].every(Number.isFinite))
    return '追従計算に使った姿勢と時刻を確認できません。';
  return controlFrameIssue(control);
}

export function controlOverlay(frame: ControlFrame | null | undefined, pose: PlanPose | null | undefined): ControlOverlay | undefined {
  if (controlFrameIssue(frame) || !frame?.target || !pose || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return;
  return {origin: [pose.x, pose.y], target: frame.target};
}
