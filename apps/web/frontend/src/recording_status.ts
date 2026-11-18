export const RECORDING_PHASES = ['idle', 'starting', 'recording', 'stopping', 'error'] as const;
export type RecordingPhase = typeof RECORDING_PHASES[number];
export interface RecordingRequest {
  source: string; action: 'start' | 'stop'; stamp: number;
  result: 'accepted' | 'ignored' | 'rejected'; reason: string;
}
export interface RecordingResult {
  bag_path: string; success: boolean; reason: string; elapsed_s: number; closed_at: number;
  size_bytes: number | null; message_count: number | null;
}
export interface RecordingSample {
  schema_version: 1; instance_id: string;
  state: RecordingPhase; stamp: number; age_s: number; stale: boolean; source_age_s?: number;
  profile: string; bag_path: string; elapsed_s: number | null; started_at: number | null; message: string;
  last_request: RecordingRequest | null; last_result: RecordingResult | null;
}
export interface RecordingEnvelope {
  connected: boolean; identity?: { instance_id?: string; hostname?: string }; freshness_s?: number;
  status: RecordingSample | null; availability?: unknown;
}
export type RecordingAvailability = 'loading' | 'disconnected' | 'missing' | 'stale' | 'invalid' | 'unavailable' | 'fresh';

/** age_s は bridge が計算した元状態の経過時間。source_age_s を再加算しない。 */
export function recordingSampleAge(sample: RecordingSample | null, receivedAt: number, now: number): number | null {
  return sample && typeof sample.age_s === 'number' && Number.isFinite(sample.age_s) && sample.age_s >= 0
    && receivedAt > 0 && Number.isFinite(now) && now >= receivedAt ? sample.age_s + (now - receivedAt) / 1000 : null;
}
export function recordingAvailability(envelope: RecordingEnvelope | null, receivedAt: number, now: number, requestFailed = false): RecordingAvailability {
  if (requestFailed) return 'unavailable';
  if (!envelope) return 'loading';
  if (envelope.connected !== true) return 'disconnected';
  const sample = envelope.status;
  if (!sample) return 'missing';
  const age = recordingSampleAge(sample, receivedAt, now), limit = envelope.freshness_s;
  if (age === null || typeof limit !== 'number' || !Number.isFinite(limit) || limit <= 0
      || sample.schema_version !== 1 || typeof sample.instance_id !== 'string' || !sample.instance_id
      || typeof sample.stamp !== 'number' || !Number.isFinite(sample.stamp) || sample.stamp < 0
      || !RECORDING_PHASES.includes(sample.state) || typeof sample.stale !== 'boolean') return 'invalid';
  return sample.stale || age > limit ? 'stale' : 'fresh';
}
const PHASE_LABELS: Record<RecordingPhase, string> = { idle: '待機中', starting: '記録を開始中', recording: '記録中', stopping: '記録を終了処理中', error: '記録エラー' };
const AVAILABILITY_LABELS: Record<Exclude<RecordingAvailability, 'fresh'>, string> = {
  loading: '記録状態を取得中', disconnected: 'Jetson 未接続', missing: '記録ノードの状態未受信',
  stale: '記録状態の更新なし', invalid: '記録状態を確認できません', unavailable: '記録監視の接続エラー',
};
export function recordingPhaseText(phase: unknown): string {
  return typeof phase === 'string' && Object.hasOwn(PHASE_LABELS, phase) ? PHASE_LABELS[phase as RecordingPhase] : '未確認';
}
export function recordingStatusText(availability: RecordingAvailability, phase: unknown): string {
  return availability === 'fresh' ? recordingPhaseText(phase) : AVAILABILITY_LABELS[availability];
}
export function recordingDuration(seconds: unknown): string {
  if (typeof seconds !== 'number' || !Number.isFinite(seconds) || seconds < 0) return '—';
  const total = Math.floor(seconds), hours = Math.floor(total / 3600), minutes = Math.floor(total / 60) % 60, remainder = total % 60;
  return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(remainder).padStart(2, '0')}`;
}
export function recordingSize(bytes: unknown): string {
  if (typeof bytes !== 'number' || !Number.isSafeInteger(bytes) || bytes < 0) return '—';
  if (bytes < 1024) return `${bytes} B`;
  const units = ['KiB', 'MiB', 'GiB', 'TiB', 'PiB'];
  const exponent = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length);
  return `${(bytes / 1024 ** exponent).toFixed(1)} ${units[exponent - 1]}`;
}
export function recordingRequestText(request: RecordingRequest | null): string {
  if (!request) return '操作要求はまだありません';
  const action = request.action === 'start' ? '開始要求' : request.action === 'stop' ? '停止要求' : '操作要求';
  const result = request.result === 'accepted' ? '受付済み' : request.result === 'ignored' ? '実行せず' : request.result === 'rejected' ? '拒否' : '結果未確認';
  return `${action} · ${result}`;
}
export function recordingSourceText(source: unknown): string {
  return source === 'joy' || source === 'teleop' || source === 'atd_teleop' ? 'Bluetooth コントローラ'
    : source === 'propo' || source === 'rc' || source === 'atd_rc_input' ? 'プロポ' : '入力元未確認';
}
export function recordingResultText(result: RecordingResult | null): string {
  return !result ? '終了結果はまだありません' : result.success === true ? '保存確定' : result.success === false ? '正常終了を確認できず' : '終了結果は未確認';
}
const RECORDING_REASONS: Record<string, string> = {
  recording_started: '記録の開始を確認しました', stopped_by_request: '停止要求を受けて終了しました',
  shutdown: '記録ノードの終了に伴う停止', start_failed: '記録を開始できませんでした', stop_failed: '記録の終了処理に失敗しました',
  missing_topics: '必要なトピックを受信できません', invalid_event: '操作要求の形式が無効です',
  stale_event: '古い操作要求を実行しませんでした', future_event: '時刻が先の操作要求を実行しませんでした',
  duplicate_event: '重複した操作要求を実行しませんでした', already_recording: 'すでに記録を開始しています',
  already_idle: 'すでに待機中です', stopping: '記録の終了処理中です', disk_space_low: '保存先の空き容量が不足しています',
  empty_bag: '記録データを確認できません', metadata_invalid: 'bag の保存結果を確認できません',
  start_accepted: '開始要求を受け付けました', stop_accepted: '停止要求を受け付けました',
  already_starting: 'すでに開始処理中です', already_stopping: 'すでに終了処理中です', busy_stopping: '終了処理中のため開始できません',
  invalid_source: '許可されていない入力元の操作要求です', invalid_action: '対応していない操作要求です',
  invalid_stamp: '操作要求の時刻形式が不正です',
  zero_stamp: '時刻が未設定の操作要求を実行しませんでした', stale_request: '古い操作要求を実行しませんでした',
  future_request: '時刻が先の操作要求を実行しませんでした', out_of_order_request: '順序が逆転した操作要求を実行しませんでした',
  duplicate_request: '重複した操作要求を実行しませんでした', stop_has_priority: '停止要求を優先しました', shutting_down: '記録ノードの終了処理中です',
  recording_completed: 'bag の保存内容を確認しました', start_cancelled: '記録の開始を取り消しました',
  missing_required_topics: '開始に必要なトピックを受信できません', low_disk_space: '保存先の空き容量が不足しています',
  storage_unavailable: '保存先を使用できません', write_failed: '記録データの書き込みに失敗しました',
  metadata_missing: 'bag の metadata を確認できません', mcap_missing: 'MCAP の記録ファイルを確認できません',
  required_topic_missing: '保存結果に必要なトピックがありません', required_topic_empty: '必要なトピックの記録メッセージがありません',
  required_type_mismatch: '必要なトピックの型が一致していません', required_publishers_lost: '必要なトピックの配信元を確認できなくなりました',
  manager_shutdown: '記録ノードの終了に伴う停止',
};
export function recordingReasonText(reason: unknown): string {
  return typeof reason === 'string' && Object.hasOwn(RECORDING_REASONS, reason) ? RECORDING_REASONS[reason]
    : '詳しい理由は受信情報を確認してください';
}
