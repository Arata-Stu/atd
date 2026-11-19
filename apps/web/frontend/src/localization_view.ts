import type { MapDocument } from './types';
import type { LiveStatus, Localization, TrackingState } from './workspace_types';

export interface MapReference { document: MapDocument; hdmap_sha256: string; map_sha256: string | null }
export const trackingLabels: Record<TrackingState, string> = {
  waiting_hint: '初期位置待ち', localizing: '地図照合中', tracking: '追跡中', lost: '追跡喪失', stale: '更新途絶',
};

export function streamId(status: LiveStatus) {
  const l = status.localization;
  if (!status.connected || !l || l.stale || l.state !== 'tracking') return '';
  return [status.identity?.instance_id, l.session_id, l.epoch, l.map_id, l.map_sha256, l.hdmap_sha256].join('/');
}

export function mapIssue(loc: Localization, ref: MapReference): string {
  if (loc.map_id !== ref.document.id) return 'Jetson が使用している地図と表示中の地図が異なります。';
  if (ref.document.frame !== 'map' || !ref.map_sha256 || ref.map_sha256 !== loc.map_sha256)
    return 'Jetson と Notebook の自己位置推定用地図が一致しません。';
  if (!ref.hdmap_sha256 || ref.hdmap_sha256 !== loc.hdmap_sha256)
    return 'Jetson と Notebook の HDMap が異なります。転送した Raceline の版を確認してください。';
  return '';
}
