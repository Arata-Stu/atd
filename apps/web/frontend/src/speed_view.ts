import type { LiveStatus } from './workspace_types';
import type { LivePlan } from './planning_view';

export interface SpeedGains { kp: number; ki: number; kd: number }
export type ManualSource = 'propo' | 'host_joy' | 'host' | 'standby' | 'fault' | 'unknown';
export const SOURCE_LABELS: Record<ManualSource, string> = {
  propo:'プロポ（Pico）', host_joy:'HOST・DualSense 入力あり', host:'HOST・入力元未確認',
  standby:'待機', fault:'基板異常', unknown:'経路不明',
};
function manualSource(value: unknown): ManualSource {
  return typeof value === 'string' && Object.hasOwn(SOURCE_LABELS, value) ? value as ManualSource : 'unknown';
}
export interface SpeedSample {
  valid: boolean; reason: string; target_speed_mps: number | null; estimated_speed_mps: number | null;
  calculated_raw: number | null; propo_raw: number | null; feedforward_raw: number | null;
  p_raw: number | null; i_raw: number | null; d_raw: number | null; saturated: boolean;
  propo_age_s: number | null; propo_reason: string;
  manual_source: ManualSource; joy_throttle: number | null; joy_age_s: number | null;
  joy_enable_requested: boolean | null; joy_reason: string;
}
export interface SpeedPoint {time_s: number; segment?: number | null; target_speed_mps: number | null; estimated_speed_mps: number | null;
  calculated_raw: number | null; propo_raw: number | null; manual_source: ManualSource; joy_throttle: number | null}
export interface LiveSpeed extends SpeedSample, SpeedGains, Pick<LivePlan,
  'map_id' | 'map_sha256' | 'hdmap_sha256' | 'session_id' | 'epoch' | 'line' | 'stamp' | 'remaining_s' | 'stale'> {
  calibration_run_id: string; proposal_sha256: string; history?: SpeedPoint[];
}
export const HISTORY_S = 5;
export const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const finiteOrNull = (value: unknown) => finite(value) ? value : null;
const rawOrNull = (value: unknown) => finite(value) && value >= 0 && value <= 2047 ? value : null;
const normalizedOrNull = (value: unknown) => finite(value) && value >= -1 && value <= 1 ? value : null;

function comparisonPoint(time_s: number, sample: SpeedPoint | SpeedSample): SpeedPoint {
  return { time_s, target_speed_mps: finiteOrNull(sample.target_speed_mps), estimated_speed_mps: finiteOrNull(sample.estimated_speed_mps),
    calculated_raw: rawOrNull(sample.calculated_raw), propo_raw: Number.isInteger(sample.propo_raw) ? rawOrNull(sample.propo_raw) : null,
    joy_throttle: normalizedOrNull(sample.joy_throttle), manual_source: manualSource(sample.manual_source) };
}

export function speedPoint(time_s: number, sample: SpeedSample): SpeedPoint {
  const point = comparisonPoint(time_s, sample);
  if (!sample.valid) point.calculated_raw = null;
  return point;
}

export function liveSpeedIssue(status: LiveStatus | null, timedOut: boolean): string {
  const speed = status?.speed_control, plan = status?.planning, loc = status?.localization;
  if (!status?.connected || !speed) return 'Jetson の速度制御の観測を待っています。';
  if (!loc || loc.state !== 'tracking' || loc.stale || !plan || !plan.valid || plan.stale)
    return speed.reason || '走行計画の更新を待っています。';
  for (const field of ['map_id','map_sha256','hdmap_sha256','session_id','epoch'] as const)
    if (plan[field] !== speed[field] || loc[field] !== speed[field]) return '地図または追跡区間が一致しません。';
  if (plan.line !== speed.line) return '走行計画と速度制御のラインが一致しません。';
  if (speed.stale || timedOut || !finite(speed.stamp) || !finite(speed.remaining_s) || speed.remaining_s <= 0)
    return speed.reason || '新しい速度制御の観測を待っています。';
  return '';
}

export function liveSpeedHistory(row: LiveSpeed): SpeedPoint[] {
  const current = speedPoint(row.stamp, row);
  if (!Array.isArray(row.history)) return [current];
  const points: SpeedPoint[] = [];
  for (const p of row.history) {
    if (!finite(p.time_s) || p.time_s < row.stamp - HISTORY_S || p.time_s >= row.stamp) continue;
    points.push(comparisonPoint(p.time_s, p));
  }
  return [...points, current];
}

export type SpeedMetric = 'target_speed_mps' | 'estimated_speed_mps' | 'calculated_raw' | 'propo_raw' | 'joy_throttle';

export function sourceIntervals(points: SpeedPoint[], start: number, end: number) {
  const intervals: {source: ManualSource; from: number; to: number; segment?: number | null}[] = [];
  for (const point of points) {
    if (!finite(point.time_s) || point.time_s < start || point.time_s > end) continue;
    const last = intervals.at(-1);
    if (last && point.manual_source === last.source && point.segment === last.segment &&
        point.time_s >= last.to && point.time_s - last.to <= .5) last.to = point.time_s;
    else intervals.push({source:point.manual_source, from:point.time_s, to:point.time_s, segment:point.segment});
  }
  return intervals;
}

export function plotSegments(points: SpeedPoint[], metric: SpeedMetric, start: number, end: number,
  convert?: (value: number) => number | null): [number, number][][] {
  const segments: [number, number][][] = [];
  let line: [number, number][] = [];
  let segment: number | null | undefined;
  for (const row of points) {
    if (!finite(row.time_s) || row.time_s < start || row.time_s > end) continue;
    const input = row[metric];
    const value = convert && finite(input) ? convert(input) : input;
    const last = line.at(-1);
    if (row.segment !== segment || !finite(value) || (last && (row.time_s <= last[0] || row.time_s - last[0] > .5))) line = [];
    segment = row.segment;
    if (!finite(value)) continue;
    if (!line.length) segments.push(line);
    line.push([row.time_s, value]);
  }
  return segments;
}
