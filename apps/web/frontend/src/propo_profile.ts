/** Pico の監視と Jetson 側の役割設定。Pico の走行用 CH は変更しない。 */
export const PROPO_ROLES = ['recording', 'auxiliary_1', 'auxiliary_2'] as const;
export const PROPO_CHANNELS = Array.from({ length: 13 }, (_, index) => index + 4);
export const PROPO_DISPLAY_COUNTS = [6, 10, 16] as const;
export type PropoDisplayCount = typeof PROPO_DISPLAY_COUNTS[number];
export type PropoRole = typeof PROPO_ROLES[number];
export type PropoPosition = 'low' | 'middle' | 'high' | 'unassigned';
export interface PropoBinding { channel: number; low_threshold: number; high_threshold: number; inverted: boolean }
export interface PropoProfile { version: number; debounce_ms: number; roles: Record<PropoRole, PropoBinding> }
export interface PropoIdentity { instance_id?: string; hostname?: string }
interface Freshness { age_s?: number; stale?: boolean }
export interface PicoTelemetry extends Freshness {
  channels: number[]; instance_id?: string; serial?: string; stamp?: number;
  rc_valid?: boolean; sbus_flags?: number; sbus_fresh?: boolean; sbus_age_ms?: number;
  vbec_mv?: number; power_ready?: boolean; active_path?: number; fault_bits?: number;
  sbus_frames?: number; sbus_lost_frames?: number; sbus_failsafe_frames?: number;
  uart_errors?: number; rpm_edges?: number; tx0_dropped?: number; tx1_dropped?: number;
  rx0_rejected?: number; rx1_rejected?: number;
}
export interface PropoState {
  connected: boolean; identity?: PropoIdentity; message?: string; ready?: boolean; input_current?: boolean;
  board: (Freshness & { connected: boolean; streaming: boolean; serial?: string; path?: string;
    instance_id?: string; reason?: string; telemetry_timeout_s?: number; telemetry: PicoTelemetry | null }) | null;
  input: (Freshness & { configuring: boolean; profile_valid: boolean; profile_path: string;
    profile?: PropoProfile; ready: boolean; reason: string; input_instance_id?: string; serial?: string; input_age_s?: number;
    roles?: Partial<Record<PropoRole, { channel: number; raw: number | null; position: PropoPosition | 'unknown' }>> }) | null;
}
export interface PropoProfileResult {
  profile: PropoProfile; revision: string; saved: boolean; applied: boolean;
  path: string; roles: string[]; channels: number[];
  fixed_roles: { steering: number; throttle: number; authority: number };
  connected?: boolean; identity?: PropoIdentity;
}

export function samePropoProfile(a: PropoProfile | null | undefined, b: PropoProfile | null | undefined): boolean {
  return !!a && !!b && a.version === b.version && a.debounce_ms === b.debounce_ms
    && PROPO_ROLES.every(role => !!a.roles?.[role] && !!b.roles?.[role]
      && a.roles[role].channel === b.roles[role].channel
      && a.roles[role].low_threshold === b.roles[role].low_threshold
      && a.roles[role].high_threshold === b.roles[role].high_threshold
      && a.roles[role].inverted === b.roles[role].inverted);
}

export function propoProfileIssue(profile: PropoProfile, channels: readonly number[] = PROPO_CHANNELS): string {
  if (!profile || profile.version !== 1 || !profile.roles
      || Object.keys(profile).length !== 3 || Object.keys(profile.roles).length !== PROPO_ROLES.length) return '役割設定の形式を確認してください。';
  if (!Number.isInteger(profile.debounce_ms) || profile.debounce_ms < 0 || profile.debounce_ms > 1000) return '位置の確認時間は 0 ～ 1000 ms の整数にしてください。';
  const used = new Set<number>();
  for (const role of PROPO_ROLES) {
    const binding = profile.roles[role];
    if (!binding || Object.keys(binding).length !== 4) return '役割設定の形式を確認してください。';
    if (!Number.isInteger(binding.channel) || ![0, ...PROPO_CHANNELS].includes(binding.channel)) return '役割には CH4 ～ CH16、または未割当を選択してください。';
    if (binding.channel && !channels.includes(binding.channel)) return '接続先で利用できる役割チャンネルから選択してください。';
    if (!Number.isInteger(binding.low_threshold) || !Number.isInteger(binding.high_threshold)
        || binding.low_threshold < 0 || binding.high_threshold > 2047
        || binding.low_threshold >= binding.high_threshold) return '閾値は 0 ～ 2047 の整数で、低側を高側より小さくしてください。';
    if (typeof binding.inverted !== 'boolean') return '位置反転の設定を確認してください。';
    if (binding.channel && used.has(binding.channel)) return '同じチャンネルを複数の役割へ割り当てることはできません。';
    if (binding.channel) used.add(binding.channel);
  }
  return '';
}

/** API が提示した候補だけを使い、Pico 固定 CH と範囲外の値を除外する。 */
export function propoAvailableChannels(value: unknown): number[] {
  return Array.isArray(value) ? [...new Set(value.filter((channel): channel is number =>
    Number.isInteger(channel) && PROPO_CHANNELS.includes(channel)))].sort((a, b) => a - b) : [];
}

export function propoDisplayCount(value: unknown): PropoDisplayCount {
  return value === 10 || value === '10' ? 10 : value === 16 || value === '16' ? 16 : 6;
}

/** 表示の希望値を保持しつつ、保存前・保存済み・適用中の割当を隠さない。 */
export function propoVisibleChannelCount(preferred: PropoDisplayCount, ...profiles: (PropoProfile | null | undefined)[]): PropoDisplayCount {
  const assigned = profiles.flatMap(profile => PROPO_ROLES.map(role => profile?.roles?.[role]?.channel ?? 0))
    .filter(channel => Number.isInteger(channel) && channel >= 4 && channel <= 16);
  const needed = Math.max(preferred, ...assigned);
  return PROPO_DISPLAY_COUNTS.find(count => count >= needed) ?? 16;
}

/** 参照 rev6 channel_us(raw, false) と同じ換算。実際の出力や CH 別の反転は含まない。 */
export function propoPulseUs(raw: unknown): number | null {
  if (typeof raw !== 'number' || !Number.isInteger(raw) || raw < 0 || raw > 2047) return null;
  if (raw <= 172) return 1000;
  if (raw >= 1811) return 2000;
  return raw <= 992 ? 1000 + Math.floor(((raw - 172) * 500 + 410) / 820)
    : 1500 + Math.floor(((raw - 992) * 500 + Math.floor(819 / 2)) / 819);
}

export function propoConnectionMatches(profile: PropoIdentity | null, status: PropoIdentity | undefined): boolean {
  return !!profile?.instance_id && profile.instance_id === status?.instance_id;
}

export function propoResponseFresh(state: PropoState | null, receivedAt: number, now: number): boolean {
  return !!state?.connected && receivedAt > 0 && now >= receivedAt && now - receivedAt <= 3000;
}

export function propoInputFresh(state: PropoState | null, receivedAt: number, now: number): boolean {
  const age = state?.input?.age_s;
  return propoResponseFresh(state, receivedAt, now) && state?.input?.stale === false
    && typeof age === 'number' && Number.isFinite(age) && age >= 0 && age + (now - receivedAt) / 1000 <= 3;
}

function sampleTimeout(state: PropoState | null): number {
  const timeout = state?.board?.telemetry_timeout_s ?? .3;
  return typeof timeout === 'number' && Number.isFinite(timeout) && timeout >= .01 && timeout <= 5 ? timeout : -1;
}

/** 設定状態の定期配信と、役割判定に使われた元入力の期限を分ける。 */
export function propoRoleInputCurrent(state: PropoState | null, receivedAt: number, now: number): boolean {
  const age = state?.input?.input_age_s;
  return propoInputFresh(state, receivedAt, now) && propoSourceMatches(state)
    && state?.input_current === true && propoRcValid(state, receivedAt, now)
    && typeof age === 'number' && Number.isFinite(age) && age >= 0
    && age + (now - receivedAt) / 1000 <= sampleTimeout(state);
}

/** USB状態と、元の受信サンプルの失効を独立して検査する。 */
export function propoTelemetryFresh(state: PropoState | null, receivedAt: number, now: number): boolean {
  const board = state?.board, telemetry = board?.telemetry;
  return propoResponseFresh(state, receivedAt, now) && board?.connected === true && board.streaming === true
    && board.stale === false && telemetry?.stale === false
    && Number.isFinite(telemetry.age_s) && (telemetry.age_s ?? -1) >= 0
    && (telemetry.age_s ?? Infinity) + (now - receivedAt) / 1000 <= sampleTimeout(state)
    && Array.isArray(telemetry.channels) && telemetry.channels.length === 16
    && telemetry.channels.every(value => Number.isInteger(value) && value >= 0 && value <= 2047)
    && !!board.instance_id && (!telemetry.instance_id || telemetry.instance_id === board.instance_id)
    && (!telemetry.serial || telemetry.serial === board.serial);
}

export function propoSourceMatches(state: PropoState | null): boolean {
  const source = state?.board?.instance_id, input = state?.input?.input_instance_id;
  return !!source && source === input
    && (!state?.board?.serial || !state.input?.serial || state.board.serial === state.input.serial);
}

export function propoRcValid(state: PropoState | null, receivedAt: number, now: number): boolean {
  const telemetry = state?.board?.telemetry;
  return propoTelemetryFresh(state, receivedAt, now) && telemetry?.rc_valid === true
    && telemetry.sbus_fresh === true && Number.isInteger(telemetry.sbus_flags)
    && (telemetry.sbus_flags ?? -1) >= 0 && (telemetry.sbus_flags ?? 256) <= 255
    && ((telemetry.sbus_flags ?? 12) & 12) === 0;
}

/** 編集中の閾値による瞬時の位置表示。デバウンス後のイベントとは別。 */
export function propoPosition(binding: PropoBinding, raw: number | null): PropoPosition | null {
  if (binding.channel === 0) return 'unassigned';
  if (raw === null || !Number.isInteger(raw) || raw < 0 || raw > 2047) return null;
  const position = raw <= binding.low_threshold ? 'low' : raw >= binding.high_threshold ? 'high' : 'middle';
  return binding.inverted && position !== 'middle' ? position === 'low' ? 'high' : 'low' : position;
}

const PROPO_REASONS: Record<string, string> = {
  configuring: '役割設定中です。記録などの操作要求を止めています。',
  profile_missing_or_invalid: '役割設定を保存し、再読込してください。',
  usb_unavailable: 'Pico の新しい入力を待っています。USB 接続と入力の起動を確認してください。',
  rc_signal_invalid: 'プロポの入力が無効です。送信機と受信機の状態を確認してください。',
  rc_stale: 'プロポからの入力が途絶えています。',
  baseline_pending: '現在のスイッチ位置を確認しています。再接続だけでは記録を開始しません。',
  ready: 'プロポ入力を監視し、割り当てた操作要求を生成しています。',
};
export function propoReasonText(reason: unknown): string {
  return typeof reason === 'string' && Object.hasOwn(PROPO_REASONS, reason)
    ? PROPO_REASONS[reason] : '入力状態を確認できません。接続と入力の起動を確認してください。';
}

const PICO_CONNECTION_REASONS: Record<string, string> = {
  waiting_for_device: 'Pico の USB 接続を待っています。接続と Jetson 側での認識を確認してください。',
  multiple_devices: 'Pico の候補が複数あります。Jetson 側の起動設定で接続先のシリアル番号またはパスを指定してください。',
  serial_error: 'Pico と USB 通信できません。接続、デバイスの権限、他のプロセスによる使用を確認してください。',
  awaiting_stream: 'USB は接続済みです。Pico からの監視データを待っています。',
  telemetry_stale: 'Pico の監視データが途絶えています。USB 接続と基板の起動状態を確認してください。',
  rc_unavailable: 'Pico は監視できていますが、プロポの入力は無効です。送信機と受信機を確認してください。',
};
export function picoConnectionText(reason: unknown): string {
  return typeof reason === 'string' && Object.hasOwn(PICO_CONNECTION_REASONS, reason)
    ? PICO_CONNECTION_REASONS[reason] : '';
}

export function picoPathText(path: unknown): string {
  return path === 0 ? '待機' : path === 1 ? 'プロポ操作' : path === 2 ? 'Jetson 操作' : path === 3 ? '異常' : '未確認';
}
export function picoFaultText(bits: unknown): string {
  if (!Number.isInteger(bits) || typeof bits !== 'number' || bits < 0 || bits > 0xffffffff) return '未確認';
  if (!bits) return 'なし';
  const names = ['電源', 'USB', '受信喪失', '切替位置', '受信 failsafe', '指令期限', 'heartbeat 期限', '指令値'];
  const faults = names.filter((_, index) => ((bits >>> index) & 1) !== 0);
  if ((bits >>> 8) !== 0) faults.push('未定義の状態');
  return faults.join('・');
}
