"""Expose the observed plan in its source pose and tracking epoch."""
import math
import re


def _invalid(status, reason):
    return dict(status, valid=False, reason=reason, vehicle_pose=None, trajectory=[],
                trajectory_speeds_mps=[], lateral_error_m=None, target_speed_mps=None,
                estimated_speed_mps=None, progress_m=None, remaining_s=0.)


def source_status(message):
    return dict(map_id=message.map_id, map_sha256=message.map_sha256, hdmap_sha256=message.hdmap_sha256,
                  session_id=message.session_id, epoch=message.epoch, line=message.line,
                  stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                  validity_s=message.validity_s if math.isfinite(message.validity_s) else None)


def reference_pose(message):
    if (message.header.frame_id != 'map' or not message.map_id or not message.session_id
            or message.line not in ('raceline', 'centerline', 'custom')
            or not math.isfinite(message.validity_s) or message.validity_s <= 0
            or not all(re.fullmatch('[0-9a-f]{64}', h) for h in (message.map_sha256, message.hdmap_sha256))):
        return None
    p, q = message.pose.position, message.pose.orientation
    if (not all(map(math.isfinite, (p.x, p.y, q.x, q.y, q.z, q.w)))
            or not .999 <= math.hypot(q.x, q.y, q.z, q.w) <= 1.001):
        return None
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return dict(x=p.x, y=p.y, yaw=yaw)


def observation(message):
    status = source_status(message)
    if not message.valid:
        return _invalid(status, message.reason)
    pose = reference_pose(message)
    if pose is None:
        return _invalid(status, '計画の姿勢・座標系・ライン・有効期限・出典が不正です')
    xy = [[point.x, point.y] for point in message.trajectory]
    speeds = list(message.trajectory_speeds_mps)
    if (not xy or len(xy) != len(speeds) or not all(math.isfinite(v) for pt in xy for v in pt)
            or not all(math.isfinite(v) and v >= 0 for v in speeds) or message.target_speed_mps < 0
            or not all(map(math.isfinite, (message.lateral_error_m, message.progress_m,
                                           message.target_speed_mps, message.estimated_speed_mps)))):
        return _invalid(status, '計画の参照軌道・速度・姿勢が不正です')
    return dict(status, valid=True, reason=message.reason, vehicle_pose=pose,
                trajectory=xy, trajectory_speeds_mps=speeds, lateral_error_m=message.lateral_error_m,
                target_speed_mps=message.target_speed_mps, estimated_speed_mps=message.estimated_speed_mps,
                progress_m=message.progress_m)


def live_plan(cache, publishers, localization, ros_now):
    plan = cache.get('planning')
    if plan is None:
        return None
    if publishers != 1:
        return _invalid(plan, '計画の配信元が一つに定まりません')
    if not plan['valid']:
        return plan
    if not localization or localization['state'] != 'tracking' or localization['stale']:
        return _invalid(plan, '自己位置を追跡できていません')
    if any(plan[key] != localization[key] for key in ('map_id', 'hdmap_sha256', 'map_sha256', 'session_id', 'epoch')):
        return _invalid(plan, '計画と現在の追跡区間・地図の版が一致しません')
    age = ros_now - plan['stamp']
    remaining = min(plan['validity_s'], cache.fresh_s) - max(age, plan['age_s'])
    if age < -.05 or remaining <= 0 or plan['stale']:
        return _invalid(plan, '前方軌道の有効期限が切れました')
    return dict(plan, remaining_s=min(remaining, localization['remaining_s']))
