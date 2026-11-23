export type Mode = 'fixed' | 'adaptive';
export type Point = { x_m: number; y_m: number };
export type SimSample = Point & {
  t_s: number; yaw_rad: number; lateral_error_m: number;
  speed_mps: number; target_speed_mps: number; steering_deg: number; steering_command_deg: number;
  lookahead_m: number; target_x_m: number; target_y_m: number; throttle: number; progress_m: number;
};
export interface SimRun {
  mode: Mode; completed: boolean; reason: string; samples: SimSample[];
  collision?: (Point & { t_s: number }) | null;
  metrics: { max_abs_lateral_error_m: number; rms_lateral_error_m: number; steering_saturation_s: number; elapsed_s: number; laps_completed: number };
}
export const defaultSettings = {
  fixed_lookahead_m: 0.65, min_lookahead_m: 0.35, max_lookahead_m: 1.5, lookahead_time_s: 0.4,
  steering_delay_ms: 0, throttle_delay_ms: 0, steering_tau_s: 0.12, speed_tau_s: 0.5,
};
export type SimSettings = typeof defaultSettings;
export interface Comparison {
  course: { name: string; points: Point[]; length_m: number; left?: Point[]; right?: Point[];
    line?: string; speeds_mps?: number[] };
  model: { wheelbase_m: number; steering_limit_deg: number; dt_s: number; laps: number };
  settings: SimSettings; runs: SimRun[];
}
export const modeLabel = { fixed: '固定', adaptive: '速度連動' };
export const modeColor = { fixed: '#226ca1', adaptive: '#ac4b19' };

// A run can finish or lose tracking before the other run. Never extend its trace.
export function sampleAt(samples: SimSample[], time: number): SimSample | undefined {
  if (!samples.length || time < samples[0].t_s || time > samples[samples.length - 1].t_s) return;
  let low = 0, high = samples.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (samples[mid].t_s <= time) low = mid;
    else high = mid - 1;
  }
  const a = samples[low], b = samples[low + 1];
  if (!b || a.t_s === time) return a;
  const fraction = (time - a.t_s) / (b.t_s - a.t_s);
  // Match the linear segments in the time plots. Heading takes the shorter arc.
  const point = { ...a };
  for (const key of Object.keys(a) as (keyof SimSample)[]) {
    point[key] = a[key] + fraction * (b[key] - a[key]);
  }
  const turn = b.yaw_rad - a.yaw_rad;
  point.yaw_rad = a.yaw_rad + fraction * Math.atan2(Math.sin(turn), Math.cos(turn));
  return point;
}
