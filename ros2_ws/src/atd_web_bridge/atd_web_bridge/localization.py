"""追跡状態と同じ ROS 試料の地図姿勢・前後速度をまとめて公開する。"""
import math
import re


def observation(message):
    status = dict(map_id=message.map_id, map_sha256=message.map_sha256, hdmap_sha256=message.hdmap_sha256,
                  session_id=message.session_id, epoch=message.epoch, state=message.state, reason=message.reason,
                  stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
    if (not status['session_id'] or not status['map_id'] or message.header.frame_id != 'map'
            or not all(re.fullmatch(r'[0-9a-f]{64}', status[k]) for k in ('map_sha256', 'hdmap_sha256'))
            or status['state'] not in ('waiting_hint', 'localizing', 'tracking', 'lost', 'stale')):
        return dict(status, state='stale', reason='自己位置の座標系・出典または状態が不正です')
    if message.state == 'tracking':
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, message.speed_mps, message.yaw_rate_rps)
        if not all(map(math.isfinite, values)) or not .999 <= math.hypot(q.x, q.y, q.z, q.w) <= 1.001:
            return dict(status, state='stale', reason='自己位置の計測値が不正です')
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        status.update(pose=dict(x=p.x, y=p.y, z=p.z, yaw=yaw, frame='map', stamp=status['stamp']),
                      velocity=dict(linear_mps=message.speed_mps, angular_rps=message.yaw_rate_rps,
                                    frame='base_link', stamp=status['stamp']))
    return status


def live_observations(cache, publishers, ros_now):
    status = cache.get('localization')
    if status is None and publishers == 0:
        return dict(localization=None, pose=cache.get('pose'), velocity=cache.get('velocity'))
    if status is None:
        return dict(localization=None, pose=None, velocity=None)
    pose, velocity = status.pop('pose', None), status.pop('velocity', None)
    age = ros_now - status['stamp']
    if publishers != 1 or status['stale'] or not -.05 <= age <= cache.fresh_s:
        reason = '自己位置の配信元が一つに定まりません' if publishers != 1 else '自己位置の更新が途絶えています'
        status.update(state='stale', reason=reason, stale=True)
    status['remaining_s'] = max(0., cache.fresh_s - max(age, status['age_s']))
    if status['state'] != 'tracking':
        return dict(localization=status, pose=None, velocity=None)
    timing = dict(age_s=max(age, status['age_s']), stale=False)
    return dict(localization=status, pose=dict(pose, **timing), velocity=dict(velocity, **timing))
