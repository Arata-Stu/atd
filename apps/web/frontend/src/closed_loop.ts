export type XY = [number, number];
export type Span = [number, number];
export type Timed = {t: number; segment?: number};
export type Pose = Timed & {x: number; y: number; yaw_rad: number};
export type Sample = Pose & {
  steering_rad: number | null; requested_steering_rad: number | null; speed_mps: number | null;
  clearance_m: number | null; image_to_apply_ms: number | null; image_to_control_ms: number | null; command_age_ms: number | null;
  actuator_state: string | null; tracking_lateral_error_m: number | null; control_image_s: number | null;
};
export type Estimate = Pose & {
  segment: number; lateral_error_m: number; yaw_error_rad: number; received_delay_ms: number | null;
};
export type Vslam = Timed & {track_ms: number | null; callback_ms: number | null; vo_state: number | null};
export type GazeboRun = {
  circuit: string; id: string; status: string; reason: string; map_id: string | null; line: string | null;
  started: number | null; laps_completed: number | null; laps_requested: number | null; max_speed_mps: number | null;
};
export type ClosedLoopDetail = {
  run: GazeboRun; samples: Sample[]; estimates: Estimate[]; vslam: Vslam[]; reference: XY[];
  reference_speeds_mps?: number[]; reference_source: 'snapshot' | 'verified_map' | 'current_map' | 'missing';
  boundaries: {left: XY[]; right: XY[]}; contact_s: number | null; warnings: string[]; anchor_s: number | null;
  footprint: {vehicle_length: number; vehicle_width: number; rear_axle_to_rear: number} | null;
};

export const maxGap = .25;
export const degrees = (value: number) => value * 180 / Math.PI;
export const runKey = (run: GazeboRun) => run.circuit + '/' + run.id;
export const statuses: Record<string,string> = {
  completed: '完走', off_course: '境界接触', command_timeout: '指令タイムアウト', control_stopped: '制御停止',
  no_progress: '進行停止', invalid_trace: '記録異常', cancelled: '中断', interrupted: '中断',
  running: '走行中', error: 'エラー', failed: '失敗', unknown: '状態不明',
};

function lowerBound<T extends Timed>(rows: T[], t: number) {
  let low = 0, high = rows.length;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (rows[mid].t < t) low = mid + 1;
    else high = mid;
  }
  return low;
}

export function timeBounds(run: ClosedLoopDetail): Span | null {
  const times: number[] = [];
  for (const rows of [run.samples, run.estimates, run.vslam]) {
    if (rows.length) times.push(rows[0].t, rows[rows.length - 1].t);
  }
  if (run.contact_s !== null) times.push(run.contact_s);
  return times.length ? [Math.min(...times), Math.max(...times)] : null;
}

export function around(t: number, full: Span): Span {
  return [Math.max(full[0], t - 3), Math.min(full[1], t + 3)];
}

export function windowRows<T extends Timed>(rows: T[], [start,end]: Span, neighbors = false): T[] {
  let low = lowerBound(rows, start), high = lowerBound(rows, end);
  if (rows[high]?.t === end) high++;
  if (neighbors) { low = Math.max(0, low - 1); high++; }
  return rows.slice(low, high);
}

export type Reading = 'linear' | 'step' | 'degrees';
// Never bridge a missing interval or extend the last value beyond the trace.
export function valueAt<T extends Timed>(rows: T[], t: number, get: (row: T) => number | null, mode: Reading = 'linear'): number | null {
  const i = lowerBound(rows,t), after = rows[i];
  if (after?.t === t) return get(after);
  const before = rows[i-1];
  if (!after || !before || after.t-before.t > maxGap || after.segment !== before.segment) return null;
  const a = get(before), b = get(after);
  if (a === null || b === null || (mode === 'degrees' && Math.abs(b-a) > 180)) return null;
  return mode === 'step' ? a : a + (b-a) * (t-before.t)/(after.t-before.t);
}

export function poseAt(rows: Pose[], t: number): XY | null {
  const x = valueAt(rows, t, p => p.x), y = valueAt(rows, t, p => p.y);
  return x === null || y === null ? null : [x,y];
}

export type Series = {label:string; color:string; rows:Timed[]; get:(p:Timed)=>number|null; mode:Reading; dashed?:boolean};
export type Chart = {label:string; unit:string; digits:number; hint:string; series:Series[]};

function series<T extends Timed>(rows:T[], label:string, color:string, get:(p:T)=>number|null, mode:Reading='linear', dashed=false): Series {
  return {rows, label, color, get:(p:Timed)=>get(p as T), mode, dashed};
}

// A trace may repeat the most recent controller report across several physics samples.
export function controlObservations(samples: Sample[]) {
  const rows: (Timed & {lateral: number|null; delay: number|null})[] = [];
  let last = -Infinity, segment = 0, active = false;
  for (const source of samples) {
    const t = source.control_image_s;
    if (t === null) { active = false; continue; }
    if (t <= last) continue;
    if (!active) segment++;
    rows.push({t, segment, lateral:source.tracking_lateral_error_m, delay:source.image_to_control_ms});
    active = true;
    last = t;
  }
  return rows;
}

export function chartsFor(run: ClosedLoopDetail): Chart[] {
  const {samples, estimates, vslam} = run;
  const control = controlObservations(samples);
  const teal = '#236a55', orange = '#ad511c', blue = '#1878a0', purple = '#7553a1';
  return [
    {label:'横ずれ', unit:'m', digits:3, hint:'推定誤差は GT 車体の左、ライン偏差はライン進行方向の左が正',
      series:[series(estimates,'推定 − GT',orange,p=>p.lateral_error_m),
        series(control,'推定 − 選択ライン（制御状態）',teal,p=>p.lateral,'linear',true)]},
    {label:'方位誤差', unit:'°', digits:2, hint:'推定 − GT、反時計回りが正',
      series:[series(estimates,'推定 − GT',orange,p=>degrees(p.yaw_error_rad),'degrees')]},
    {label:'境界余裕', unit:'m', digits:3, hint:'各 GT 姿勢の外形から舗装境界までの非負距離。接触・路外は 0 m。接触判定は観測間の移動も含みます',
      series:[series(samples,'GT 車体外形',teal,p=>p.clearance_m)]},
    {label:'操舵角', unit:'°', digits:1, hint:'Gazebo 車両モデルの実舵角と指令角。左旋回が正',
      series:[series(samples,'実舵角',blue,p=>p.steering_rad === null ? null : degrees(p.steering_rad)),
        series(samples,'指令角',orange,p=>p.requested_steering_rad === null ? null : degrees(p.requested_steering_rad),'step',true)]},
    {label:'画像からの遅れ',unit:'ms',digits:1,hint:'紫は車体の記録時刻から使用中の指令の元画像時刻を引いた値です。新しい指令に替わるまで増えます。青は画像時刻、紫は車体の記録時刻に表示するため、同じ縦線でも元画像が異なる場合があります。すべてシミュレーション時間です。',
      series:[series(estimates,'画像 → 推定受信',orange,p=>p.received_delay_ms),
        series(control,'画像 → 制御指令生成',blue,p=>p.delay,'linear',true),
        series(samples,'画像 → 指令採用',teal,p=>p.image_to_apply_ms,'step',true),
        series(samples,'使用中の指令の元画像は何 ms 前か',purple,p=>p.command_age_ms)]},
    {label:'VSLAM 処理時間',unit:'ms',digits:1,hint:'画像時刻に対応する VisualSlamStatus の実行所要時間（実時間）',
      series:[series(vslam,'追跡処理',blue,p=>p.track_ms),series(vslam,'コールバック全体',purple,p=>p.callback_ms,'linear',true)]},
  ];
}
