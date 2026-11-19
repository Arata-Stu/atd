import type { ManualDriveBoard } from './drive_monitor';

export interface Span { start_s: number; end_s: number }
export interface JoySpan extends Span { steering: number; throttle: number }
export interface VelocitySpan extends Span { speed_mps: number; segment: number | string }
export interface AccelSpan extends Span { accel_mps2: number }
export interface PropoSpan extends Span { throttle_raw: number }
export interface BoardSpan extends Span {
  source: ManualDriveBoard['source']; ch3: ManualDriveBoard['ch3'];
  control_source: ManualDriveBoard['control_source']; rx_mode_us: number | null;
  ch3_raw: number | null; pwm_enabled: boolean | null; servo_us: number | null; esc_us: number | null;
}
export interface DriveWindow extends Span {
  bag: string; duration_s: number; joy: JoySpan[]; board: BoardSpan[];
  speed_source: string | null; speed_issue: string; speed: VelocitySpan[]; imu: AccelSpan[]; propo: PropoSpan[];
}
export interface DriveReplay {
  bag: string; ready: boolean; duration_s: number | null; max_window_s: number; job_id: string | null; warnings: string[];
  speed_sources: {id: string; label: string}[]; default_speed_source: string | null;
  speed_reason: string; imu_reason: string; can_generate_vo: boolean; vo_job_id: string | null; vo_reason: string;
}

export function windowAt(time: number, duration: number, width: number): Span {
  const last = Math.max(0, Math.ceil(duration / width) - 1);
  const start_s = Math.min(last, Math.max(0, Math.floor(time / width))) * width;
  return {start_s, end_s: Math.min(duration, start_s + width)};
}

// 記録上の有効期間の終端は含まない。空白を隣の値で補間しない。
export function atTime<T extends Span>(rows: readonly T[], time: number): T | null {
  let lo = 0, hi = rows.length;
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (rows[mid].start_s <= time) lo = mid + 1;
    else hi = mid;
  }
  const row = rows[lo - 1];
  return row && time < row.end_s ? row : null;
}

/** 共通軸に表示する有効範囲ごとの帯。欠測を挟むと同じ状態でも別区間になる。 */
export function stateBands<T extends Span>(rows: readonly T[], read: (row: T) => string) {
  const result: (Span & {state: string})[] = [];
  for (const row of rows) {
    const state = read(row), prev = result[result.length - 1];
    if (prev?.state === state && prev.end_s === row.start_s) prev.end_s = row.end_s;
    else result.push({state, start_s:row.start_s, end_s:row.end_s});
  }
  return result;
}

/** 選択区間のピークが入るゼロ中心の軸。目盛りは1, 2, 5 の系列を使う。 */
export function symmetricLimit<T>(rows: readonly T[], read: (row: T) => number, floor: number) {
  let peak = floor;
  for (const row of rows) {
    const value = read(row);
    if (Number.isFinite(value)) peak = Math.max(peak, Math.abs(value));
  }
  const decade = 10 ** Math.floor(Math.log10(peak));
  return ([1,2,5].find(k => k * decade >= peak) ?? 10) * decade;
}

export function numericPath<T extends Span>(rows: readonly T[], read: (row: T) => number | null,
  window: Span, min: number, max: number, segment?: (row: T) => string | number): string {
  const path: string[] = [];
  let lastEnd = -Infinity, lastValue: number | null = null;
  let lastSegment: string | number | undefined;
  const x = (t: number) => +((t - window.start_s) / (window.end_s - window.start_s) * 1000).toFixed(4);
  for (const row of rows) {
    const start = Math.max(window.start_s, row.start_s), end = Math.min(window.end_s, row.end_s);
    const value = read(row);
    if (value === null || !Number.isFinite(value) || value < min || value > max || start >= end) continue;
    const y = +(4 + 92 * (max - value) / (max - min)).toFixed(4);
    const group = segment?.(row);
    if (lastEnd !== start || group !== lastSegment) path.push(`M${x(start)} ${y}`);
    else if (value !== lastValue) path.push(`V${y}`);
    path.push(`H${x(end)}`);
    lastEnd = end; lastValue = value; lastSegment = group;
  }
  return path.join(' ');
}

export const STOP_BAND_MPS = 0.05;
export function motionState(v: number) {
  if (Math.abs(v) <= STOP_BAND_MPS) return 'near_stop';
  return v > 0 ? 'forward' : 'reverse';
}
export const motionLabels = {forward:'前進', near_stop:'停止付近', reverse:'後退'};

export type RecordedDirection = 'forward' | 'near_zero' | 'reverse';
export interface DirectionChange {
  time_s: number; from: RecordedDirection; to: RecordedDirection; near_zero_start_s: number | null;
}

/** 同じ切替を再度選ばず、前後の確認済み切替に移動する。 */
export function adjacentChanges(changes: readonly DirectionChange[], time: number) {
  let lo = 0, hi = changes.length;
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (changes[mid].time_s < time) lo = mid + 1;
    else hi = mid;
  }
  const current = changes[lo]?.time_s === time ? changes[lo] : undefined;
  return {previous: changes[lo - 1], current, next: changes[lo + (current ? 1 : 0)]};
}

export const ch3Labels = {propo:'プロポ側', jetson:'Jetson 側', middle:'中間位置', unknown:'未確認', absent:'未受信'};
export const sourceLabels = {propo:'プロポ', dualsense:'DualSense', host:'HOST（操作元未確認）', standby:'待機', fault:'基板異常', unknown:'未確認'};
export const ch3State = (row: BoardSpan) => row.source === 'cdc0' && row.ch3 === 'unknown' && row.rx_mode_us === null
  ? 'absent' : row.ch3;
export const servoPwm = (row: BoardSpan) => row.source === 'cdc0' && row.pwm_enabled ? row.servo_us : null;
export const escPwm = (row: BoardSpan) => row.source === 'cdc0' && row.pwm_enabled ? row.esc_us : null;
