/** Jetson の設定を編集・確認するための型と純粋関数。走行指令は送信しない。 */
export const TELEOP_AXES = ['left_x', 'left_y', 'right_x', 'right_y', 'l2', 'r2', 'dpad_x', 'dpad_y'] as const;
export const TELEOP_BUTTONS = ['cross', 'circle', 'triangle', 'square', 'l1', 'r1', 'l2', 'r2', 'create', 'options', 'ps', 'l3', 'r3', 'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right'] as const;
export type AxisRole = 'steering' | 'throttle';
export type ButtonRole = 'enable' | 'record_start' | 'record_stop';
export interface TeleopAxis { axis: string; subtract_axis: string; scale: number }
export interface TeleopProfile { version: number; axes: Record<AxisRole, TeleopAxis>; buttons: Record<ButtonRole, string> }
export interface JoySample { axes: number[]; buttons: number[]; stale?: boolean; age_s?: number }
export interface BridgeIdentity { instance_id?: string; hostname?: string }
export interface TeleopState {
  connected: boolean;
  identity?: BridgeIdentity;
  status: { configuring: boolean; profile_valid: boolean; profile_path: string; ready: boolean; reason: string; profile?: TeleopProfile; age_s?: number; stale?: boolean } | null;
  message?: string;
}
export interface TeleopProfileResult {
  profile: TeleopProfile; revision: string; saved: boolean; applied: boolean;
  axes: string[]; buttons: string[]; identity?: BridgeIdentity;
}

export function sameTeleopProfile(a: TeleopProfile | null | undefined, b: TeleopProfile | null | undefined): boolean {
  if (!a || !b || a.version !== b.version) return false;
  return (['steering', 'throttle'] as const).every(role => a.axes?.[role]?.axis === b.axes?.[role]?.axis
      && a.axes?.[role]?.subtract_axis === b.axes?.[role]?.subtract_axis
      && a.axes?.[role]?.scale === b.axes?.[role]?.scale)
    && (['enable', 'record_start', 'record_stop'] as const).every(role => a.buttons?.[role] === b.buttons?.[role]);
}

export function teleopProfileIssue(profile: TeleopProfile): string {
  for (const binding of Object.values(profile.axes)) {
    if (![binding.axis, binding.subtract_axis].every(name => name === '' || (TELEOP_AXES as readonly string[]).includes(name))) return '利用できる標準軸を選択してください。';
    if (!Number.isFinite(binding.scale) || Math.abs(binding.scale) > 1) return '倍率は −1 ～ 1 の数値を指定してください。';
  }
  const buttons = Object.values(profile.buttons).filter(Boolean);
  if (buttons.some(name => !(TELEOP_BUTTONS as readonly string[]).includes(name))) return '利用できる標準ボタンを選択してください。';
  if (new Set(buttons).size !== buttons.length) return 'ボタンの役割には、それぞれ異なるボタンを選択してください。';
  return '';
}

export function teleopRolesAssigned(profile: TeleopProfile): boolean {
  return (['steering', 'throttle'] as const).every(role => !!(profile.axes[role].axis || profile.axes[role].subtract_axis));
}

/** status 自体の鮮度と、ブラウザが最後に受信してからの時間を両方確認する。 */
export function teleopFresh(state: TeleopState | null, receivedAt: number, now: number): boolean {
  return !!state?.connected && !!state.status && state.status.stale === false && receivedAt > 0
    && now >= receivedAt && now - receivedAt <= 3000;
}

export function teleopConnectionMatches(profile: BridgeIdentity | null, state: BridgeIdentity | undefined): boolean {
  return !!profile?.instance_id && profile.instance_id === state?.instance_id;
}

const TELEOP_REASONS: Record<string, string> = {
  configuring: '役割設定中です。指令出力を停止しています。',
  profile_missing_or_invalid: '役割設定を保存し、再読込してください。',
  steering_and_throttle_are_unassigned: '操舵とスロットルの入力を割り当ててください。',
  joy_status_unavailable: 'コントローラの状態を確認できません。接続と入力の起動を確認してください。',
  joy_not_ready: 'コントローラを接続し、入力設定を終了してください。',
  joy_input_unavailable: '新しいコントローラ入力を待っています。',
  release_all_controls: 'スティック・トリガー・ボタンをすべて離してください。',
  ready: '操作入力を運転指令に変換しています。',
};

export function teleopReasonText(reason: unknown): string {
  return typeof reason === 'string' && Object.hasOwn(TELEOP_REASONS, reason)
    ? TELEOP_REASONS[reason] : '状態を確認できません。Jetsonの接続と入力を確認してください。';
}

/** 保存前の割り当てを確認するローカル表示。Jetson が出力した指令ではない。 */
export function teleopPreview(profile: TeleopProfile, sample: JoySample | null, ready: boolean): { steering: number | null; throttle: number | null; enable: boolean } | null {
  if (!ready || !sample || sample.stale !== false || sample.axes.length !== TELEOP_AXES.length
      || sample.buttons.length !== TELEOP_BUTTONS.length || sample.axes.some(value => !Number.isFinite(value) || Math.abs(value) > 1)
      || sample.axes[4] < 0 || sample.axes[5] < 0
      || sample.buttons.some(value => value !== 0 && value !== 1) || teleopProfileIssue(profile)) return null;
  const axisValue = (name: string) => name ? sample.axes[(TELEOP_AXES as readonly string[]).indexOf(name)] : 0;
  const mapped = (role: AxisRole) => {
    const binding = profile.axes[role];
    return binding.axis || binding.subtract_axis ? Math.max(-1, Math.min(1, (axisValue(binding.axis) - axisValue(binding.subtract_axis)) * binding.scale)) : null;
  };
  return { steering: mapped('steering'), throttle: mapped('throttle'),
    enable: !!profile.buttons.enable && sample.buttons[(TELEOP_BUTTONS as readonly string[]).indexOf(profile.buttons.enable)] === 1 };
}
