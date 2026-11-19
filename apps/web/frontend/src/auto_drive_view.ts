import type { LineKind } from './types';

type Switch = 'propo' | 'jetson' | 'middle' | 'unknown';
type Source = 'autonomous' | 'propo' | 'standby' | 'host' | 'fault' | 'unknown';
type State = 'driving' | 'braking' | 'latched_stop' | 'arming' | 'propo' | 'standby' | 'disconnected';

interface Stamped { stamp_s: number }
interface Pulse { servo_us: number | null; esc_us: number | null }
interface AutoBoard extends Stamped, Pulse {
  serial: string; ch3: Switch; rx_mode_us: number | null; pwm_enabled: boolean;
}
interface SentCommand extends Stamped, Pulse {
  kind: 'arm' | 'drive' | 'brake' | 'disable'; steering: number; throttle: number;
}
interface AutoSpeed extends Stamped {
  target_mps: number; estimated_mps: number; tracking_session_id: string; epoch: number; validity_s: number;
}
export interface AutoSample extends Stamped {
  map_id: string; line: LineKind; max_speed_mps: number; state: State; source: Source; reason: string;
  board: AutoBoard | null; command: SentCommand | null; speed: AutoSpeed | null;
}
export interface AutoObservation {
  now_s: number; publishers: number; session_id: string | null; issue: string;
  current: AutoSample | null; history: AutoSample[]; last_stop: (Stamped & {reason: string}) | null;
}
export const autoStates: Record<State, string> = {
  driving:'自動走行中', braking:'制動中', latched_stop:'停止要求・再開待ち', arming:'中立出力を確認中',
  propo:'手動走行', standby:'待機', disconnected:'Pico 未接続',
};
export const controlSources: Record<Source, string> = {
  autonomous:'自動（Jetson）', propo:'プロポ（Pico）', standby:'待機', host:'HOST・操作元未確認',
  fault:'基板異常', unknown:'未確認',
};
export const switchPositions: Record<Switch, string> = {
  propo:'プロポ側', jetson:'Jetson 側', middle:'中間位置', unknown:'未確認',
};
export const commandLabels: Record<SentCommand['kind'], string> = {
  arm:'中立で許可要求', drive:'走行指令', brake:'制動指令', disable:'HOST 解除',
};
const STATE_TTL = .5, IO_TTL = .15;
export const AUTO_HISTORY_S = 15;

function speedTtl(speed: AutoSpeed | null) {
  return Math.min(.25, speed?.validity_s ?? 0);
}
function deadline(sample: Stamped | null, ttl: number) {
  return (sample?.stamp_s ?? -Infinity) + ttl;
}

/** ブラウザでは Jetson のスナップショット時刻に単調経過を加えて各系列を個別に失効させる。 */
export function autoReadout(data: AutoObservation | null | undefined, elapsed: number) {
  const now = (data?.now_s ?? NaN) + elapsed;
  const snap = data?.publishers === 1 && data.session_id ? data.current : null;
  const fresh = <T extends Stamped>(row: T | null, ttl: number) =>
    row && row.stamp_s <= now && now < deadline(row, ttl) ? row : null;
  const state = fresh(snap, STATE_TTL);
  const board = fresh(state?.board ?? null, IO_TTL);
  const command = fresh(state?.command ?? null, IO_TTL);
  const speed = fresh(state?.speed ?? null, speedTtl(state?.speed ?? null));
  const remaining = [deadline(state, STATE_TTL), deadline(board, IO_TTL), deadline(command, IO_TTL),
    deadline(speed, speedTtl(speed))].filter(t => t > now);
  let source: Source = board && state ? state.source : 'unknown';
  if (source === 'autonomous' && !command) source = 'host';
  return {state, board, command, speed, source, refreshIn: remaining.length ? Math.min(...remaining) - now : null};
}

type Series = 'target_mps' | 'estimated_mps' | 'command_servo' | 'board_servo' | 'command_esc' | 'board_esc';
export type AutoSeries = Series;
type XY = [number, number];

function sampleValues(row: AutoSample, series: Series) {
  if (series === 'target_mps' || series === 'estimated_mps') {
    const s = row.speed;
    if (s) return {time:s.stamp_s, value:s[series], group:JSON.stringify([s.tracking_session_id,s.epoch]),
      ttl:speedTtl(s)};
  } else {
    const servo = series === 'command_servo' || series === 'board_servo';
    if (series === 'command_servo' || series === 'command_esc') {
      const c = row.command;
      if (c && c.kind !== 'disable') return {time:c.stamp_s, value:servo ? c.servo_us : c.esc_us,
        group:'command', ttl:IO_TTL};
    } else {
      const b = row.board;
      if (b?.pwm_enabled) return {time:b.stamp_s, value:servo ? b.servo_us : b.esc_us,
        group:JSON.stringify([b.serial,row.source]), ttl:IO_TTL};
    }
  }
  return null;
}

/** 非同期の各系列を元時刻で描画する。停止後の欠測・通信断・追跡世代切替は線を分ける。 */
export function autoSegments(rows: AutoSample[], series: Series, end: number): XY[][] {
  const segments: XY[][] = [];
  let line: XY[] | null = null, group: string | null = null, lastTime = -Infinity;
  let previousTtl = 0;
  for (const row of rows) {
    const sample = sampleValues(row, series);
    if (!sample || typeof sample.value !== 'number' || !Number.isFinite(sample.value)
        || !Number.isFinite(sample.time) || sample.time > end || sample.time < end - AUTO_HISTORY_S) {
      line = null;
      continue;
    }
    if (sample.time < lastTime) { line = null; continue; }
    if (sample.time === lastTime) continue;
    if (!line || group !== sample.group || sample.time - lastTime > Math.min(previousTtl, sample.ttl) + 1e-6) {
      line = [];
      segments.push(line);
    }
    line.push([sample.time, sample.value]);
    group = sample.group; lastTime = sample.time; previousTtl = sample.ttl;
  }
  return segments;
}
