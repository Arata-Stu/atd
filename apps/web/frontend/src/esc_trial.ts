import type { VehicleIdentity } from './vehicle_profile';

export const ACTIONS = {
  forward: '前進', brake: '制動', wait_stopped: '停止確認', neutral: 'ニュートラル', reverse: '後退',
} as const;
export type EscAction = keyof typeof ACTIONS;
export type TrialStep = {action: EscAction; duration_s: number; level?: number};
export type DraftStep = {key: number; action: EscAction; duration: string; level: string};
export type TrialStatus = 'queued' | 'running' | 'cancel_requested' | 'succeeded' | 'failed' | 'cancelled';
export interface EscObservation {
  status: TrialStatus; step_index: number | null;
  esc_state: string; throttle: number | null; pwm_us: number | null; message: string;
}
export interface EscTrial extends EscObservation {
  id: string; steps: TrialStep[]; state_history?: (EscObservation & {updated_at: number})[];
  requested_pwm_us?: number | null; pwm_observed?: boolean | null; feedback_stale?: boolean; stop_evidence?: string;
}
export interface EscStatus {
  connected?: boolean; enabled: boolean; identity?: VehicleIdentity; availability?: unknown; message?: string; trial: EscTrial | null;
}
export const TRIAL_LABELS: Record<TrialStatus, string> = {
  queued: '端末の開始待ち', running: '実行中', cancel_requested: '中止要求中',
  succeeded: '完了', failed: '失敗', cancelled: '中止済み',
};
export const STATE_LABELS: Record<string, string> = {
  neutral: 'ニュートラル', forward: '前進', braking: '制動中', wait_stopped: '停止確認待ち',
  stopped_braking: '停止確認済み・制動中', neutral_after_stop: '停止確認後のニュートラル', reverse: '後退',
  unknown: '不明',
};
export const ACTIVE_TRIAL = new Set<TrialStatus>(['queued', 'running', 'cancel_requested']);
export const needsLevel = (action: EscAction) => action === 'forward' || action === 'reverse' || action === 'brake';
export const signedThrottle = (n: number) => (n > 0 ? '+' : '') + n.toFixed(3);

export const EXAMPLE_STEPS: TrialStep[] = [
  {action: 'forward', level: .15, duration_s: 1}, {action: 'brake', level: .15, duration_s: .3},
  {action: 'wait_stopped', duration_s: 10}, {action: 'neutral', duration_s: .3},
  {action: 'reverse', level: .12, duration_s: .5}, {action: 'neutral', duration_s: .5},
];

export function draftStep(key: number, step: TrialStep): DraftStep {
  return {key, action: step.action, level: String(step.level ?? .15), duration: String(step.duration_s)};
}

export function trialPlan(draft: DraftStep[]): {steps: TrialStep[]; issue: string} {
  if (draft.length < 1 || draft.length > 128) return {steps: [], issue: '1〜128 個のステップを設定してください。'};
  const steps: TrialStep[] = [];
  for (const [i, row] of draft.entries()) {
    const duration_s = Number(row.duration), level = Number(row.level);
    if (!Number.isFinite(duration_s) || duration_s < .02 || duration_s > 300)
      return {steps: [], issue: 'ステップ ' + (i + 1) + ' の時間は 0.02〜300 秒で指定してください。'};
    if (needsLevel(row.action) && (!Number.isFinite(level) || level <= 0 || level > 1))
      return {steps: [], issue: 'ステップ ' + (i + 1) + ' の強度は 0 より大きく 1 以下で指定してください。'};
    if (needsLevel(row.action) && Math.round(level * 1000) < 2)
      return {steps: [], issue: 'ステップ ' + (i + 1) + ' の指令は基板の PWM 分解能 1 µs を下回ります。'};
    steps.push(needsLevel(row.action) ? {action: row.action, duration_s, level} : {action: row.action, duration_s});
  }
  return {steps, issue: ''};
}

export function stepCommand(step: TrialStep): string {
  if (step.action === 'wait_stopped') return '制動を維持';
  if (step.action === 'neutral') return '0.000';
  if (step.level === undefined) return '—';
  return signedThrottle(step.action === 'forward' ? step.level : -step.level);
}
