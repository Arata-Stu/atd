"""同じ地図・追跡区間で有効な操舵計算を Notebook の監視用に返す。"""
import math

from .planning import reference_pose, source_status


def _invalid(status, reason):
    return dict(status, valid=False, reason=reason, remaining_s=0., vehicle_pose=None, target=None,
                requested_lookahead_m=None, actual_lookahead_m=None, curvature_per_m=None,
                steering_rad=None, preview_limited=False)


def observation(message):
    status = dict(source_status(message), controller=message.controller)
    low, gain, high = message.min_lookahead_m, message.lookahead_time_s, message.max_lookahead_m
    wheelbase = message.wheelbase_m if message.has_steering else None
    config_ok = (all(map(math.isfinite, (low, gain, high))) and 0 < low <= high and gain >= 0
                 and (wheelbase is None or math.isfinite(wheelbase) and wheelbase > 0))
    status['config'] = (dict(min_lookahead_m=low, lookahead_time_s=gain, max_lookahead_m=high,
                             wheelbase_m=wheelbase) if config_ok else None)
    if not message.valid:
        return _invalid(status, message.reason)
    pose = reference_pose(message)
    target = [message.target.x, message.target.y]
    if (pose is None or not message.controller or not config_ok
            or not all(map(math.isfinite, (*target, message.curvature_per_m,
                                           message.requested_lookahead_m, message.actual_lookahead_m)))
            or min(message.requested_lookahead_m, message.actual_lookahead_m) <= 0
            or message.has_steering and not math.isfinite(message.steering_rad)):
        return _invalid(status, '操舵計算の姿勢・目標点・設定または出典が不正です')
    return dict(status, valid=True, reason=message.reason, vehicle_pose=pose, target=target,
                requested_lookahead_m=message.requested_lookahead_m, actual_lookahead_m=message.actual_lookahead_m,
                curvature_per_m=message.curvature_per_m, steering_rad=message.steering_rad if message.has_steering else None,
                preview_limited=message.preview_limited)


def live_control(cache, publishers, plan, ros_now, *, gid=None):
    control = cache.get('control')
    if control is None:
        return None
    source_gid = control.pop('publisher_gid', None)
    if publishers != 1 or (gid is not None and gid != source_gid):
        return _invalid(control, '操舵計算の配信元が一つに定まりません')
    if not control['valid']:
        return control
    if not plan or not plan['valid']:
        return _invalid(control, '有効な前方軌道がありません')
    if any(control[key] != plan[key] for key in ('map_id', 'map_sha256', 'hdmap_sha256', 'session_id', 'epoch', 'line')):
        return _invalid(control, '操舵計算と現在の計画の地図・ライン・追跡区間が一致しません')
    age = ros_now - control['stamp']
    remaining = min(control['validity_s'], cache.fresh_s) - max(age, control['age_s'])
    if control['stale'] or age < -.05 or remaining <= 0:
        return _invalid(control, '操舵計算の有効期限が切れました')
    return dict(control, remaining_s=min(remaining, plan['remaining_s']))
