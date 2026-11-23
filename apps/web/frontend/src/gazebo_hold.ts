import type { LiveStatus } from './workspace_types';

export function gazeboHoldStatus(data: LiveStatus['gazebo'], pollStart: number, now: number) {
  const row = data?.current;
  const hold = data?.vehicle_hold;
  const fresh = !!hold && Number.isFinite(hold.remaining_s) && hold.remaining_s > 0 &&
    now - pollStart < hold.remaining_s * 1000;
  const held = fresh && hold.held && hold.speed_mps === 0;
  const paused = row?.state === 'paused';
  const running = row?.state === 'waiting' || row?.state === 'waiting_observation' || row?.state === 'driving';
  let text = '';
  if (data?.pause_supported !== true) text = '車両の一時停止機能に未接続';
  else if (row?.state === 'stopped') text = row.reason || 'この走行は終了しました';
  else if (!row) text = 'Gazebo の制御状態を受信待ち';
  else if (paused) text = held ? '車両は一時停止中（0 m/s）' : '一時停止指令を受付済み・車両の停止確認待ち';
  else if (held) text = '車両は停止保持中・新しい走行指令を待機';
  else if (row.state === 'waiting_observation') text = '自己位置・計画待ち';
  else text = row.state === 'waiting' ? '発進待ち' : '走行指令中';
  return {text, canPause: data?.pause_supported === true && running,
    canResume: data?.pause_supported === true && paused && held};
}
