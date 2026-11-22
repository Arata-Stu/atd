export const ACTIVE = new Set(['starting', 'queued', 'running', 'stopping', 'cancel_requested']);
export const STATUS: Record<string, string> = {
  starting: '開始中', queued: '待機中', running: '実行中', stopping: '停止中', cancel_requested: '停止要求中',
  succeeded: '完了', completed: '完了', failed: '失敗', cancelled: '中止', canceled: '中止',
  interrupted: '中断', orphaned: '要確認',
};

export function historyBoundary(date: string, time: string, end: boolean) {
  if (!date) return end ? Infinity : -Infinity;
  const local = new Date(date + 'T' + (time || '00:00'));
  if (end) {
    if (time) local.setTime(local.getTime() + 60000);
    else local.setDate(local.getDate() + 1);
  }
  return local.getTime() / 1000;
}
