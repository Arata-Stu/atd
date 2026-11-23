import type { MapDocument } from './types';
import type { GazeboGroundTruth, LiveStatus, Localization, TrackingState } from './workspace_types';

export interface MapReference { document: MapDocument; hdmap_sha256: string; map_sha256: string | null }
export const trackingLabels: Record<TrackingState, string> = {
  waiting_hint: '初期位置待ち', localizing: '地図照合中', tracking: '追跡中', lost: '追跡喪失', stale: '更新途絶',
};

export function streamId(status: LiveStatus) {
  const l = status.localization;
  if (!status.connected || !l || l.stale || l.state !== 'tracking') return '';
  return [status.identity?.instance_id, l.session_id, l.epoch, l.map_id, l.map_sha256, l.hdmap_sha256].join('/');
}

/** Map上の車両と軌跡には、同じ追跡観測に含まれる自己位置だけを使う。 */
export function trackingPose(status: LiveStatus | null) {
  const pose = status?.pose, loc = status?.localization;
  if (!status || !streamId(status) || !loc || !pose || pose.stale || pose.frame !== 'map' ||
      !Number.isFinite(loc.stamp) || loc.stamp <= 0 || pose.stamp !== loc.stamp ||
      ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return null;
  return pose;
}

type MapIdentity = Pick<Localization, 'map_id' | 'map_sha256' | 'hdmap_sha256'>;
export function mapIssue(loc: MapIdentity, ref: MapReference): string {
  if (loc.map_id !== ref.document.id) return '走行中の地図と表示中の地図が異なります。';
  if (ref.document.frame !== 'map' || !ref.map_sha256 || ref.map_sha256 !== loc.map_sha256)
    return '走行中の自己位置推定用地図と Notebook の地図が一致しません。';
  if (!ref.hdmap_sha256 || ref.hdmap_sha256 !== loc.hdmap_sha256)
    return '走行中の HDMap と Notebook の HDMap が異なります。走行ラインの版を確認してください。';
  return '';
}

export function gazeboGroundTruth(status: LiveStatus | null, ref: MapReference | null, expired: boolean) {
  const gt = status?.gazebo?.ground_truth;
  if (expired || !status?.connected || status.identity?.environment !== 'gazebo' || !gt || !ref ||
      gt.frame !== 'map' || mapIssue(gt, ref) || (status.localization && mapIssue(status.localization, ref)) ||
      ![gt.x, gt.y, gt.yaw, gt.stamp_s, gt.remaining_s].every(Number.isFinite) ||
      gt.stamp_s <= 0 || gt.remaining_s <= 0) return null;
  const f = gt.footprint;
  if (!f || ![f.vehicle_length, f.vehicle_width, f.rear_axle_to_rear, f.margin].every(Number.isFinite) ||
      f.vehicle_length <= 0 || f.vehicle_width <= 0 || f.rear_axle_to_rear < 0 ||
      f.rear_axle_to_rear >= f.vehicle_length || f.margin < 0) return null;
  return gt;
}

/** Gazebo の境界接触判定と同じ後輪軸基準・余裕込みの矩形 [m]。 */
export function gtFootprintBounds(f: GazeboGroundTruth['footprint']) {
  return {rear: -f.rear_axle_to_rear - f.margin, front: f.vehicle_length - f.rear_axle_to_rear + f.margin,
    halfWidth: f.vehicle_width / 2 + f.margin};
}
