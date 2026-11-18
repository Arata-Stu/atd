"""Time-local odometry derivatives at the rear axle. No ROS/GPU dependencies."""
from __future__ import annotations

from bisect import bisect_left, bisect_right
import math
from numbers import Real
from pathlib import Path

from .inputs import StaticTransformGraph, conjugate, finite_vector, multiply, quaternion, rotate, _frame


def load_tum(path, *, world_frame='odom', pose_frame):
    """Read explicit ROS-clock TUM: seconds tx ty tz qx qy qz qw.

    Numeric invalid values remain visible to estimate_motion's rejection report;
    malformed columns are a file error. Clock offsets/frame conventions are never guessed.
    """
    if world_frame != 'odom':
        raise ValueError('calibration requires odom trajectory; map/loop-closure poses are unsuitable')
    _frame(pose_frame)
    poses = []
    for number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        content = line.split('#', 1)[0].strip()
        if not content:
            continue
        values = content.split()
        if len(values) != 8:
            raise ValueError(f'TUM line {number}: expected stamp tx ty tz qx qy qz qw')
        try:
            values = [float(v) for v in values]
        except ValueError as error:
            raise ValueError(f'TUM line {number}: non-numeric field') from error
        poses.append(dict(stamp=values[0], position=values[1:4], orientation=values[4:8],
                          world_frame=world_frame, pose_frame=pose_frame, line=number))
    if not poses:
        raise ValueError('empty TUM trajectory')
    return poses


def _solve(matrix, rhs):
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for column in range(len(rows)):
        pivot = max(range(column, len(rows)), key=lambda index: abs(rows[index][column]))
        if abs(rows[pivot][column]) < 1e-12:
            raise ValueError('ill-conditioned derivative window')
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        rows[column] = [v / divisor for v in rows[column]]
        for index in range(len(rows)):
            if index != column:
                multiplier = rows[index][column]
                rows[index] = [a-multiplier*b for a, b in zip(rows[index], rows[column])]
    return [row[-1] for row in rows]


def _fit(times, values, center, half_window):
    # Scale local time before solving; absolute ROS epoch never enters a power.
    xs = [(stamp-center)/half_window for stamp in times]
    degree = min(3, len(times)-1)
    weights = [max(.05, (1-abs(x)**3)**3) for x in xs]
    matrix = [[sum(w*x**(i+j) for x, w in zip(xs, weights))
               for j in range(degree+1)] for i in range(degree+1)]
    rhs = [sum(w*x**i*y for x, w, y in zip(xs, weights, values)) for i in range(degree+1)]
    coefficients = _solve(matrix, rhs)
    return coefficients[1]/half_window, 2*coefficients[2]/half_window**2


def estimate_motion(poses, *, pose_frame, static_transforms, world_frame='odom', base_frame='base_link',
                    window_s=.3, max_gap_s=.2, max_speed_mps=20., max_yaw_rate_rps=12.,
                    max_acceleration_mps2=50., min_samples=5):
    """Fit centered local cubics independently on continuous valid pose segments.

    Returns source-centered samples; velocity/kinematic acceleration are rotated
    into body axes, angular velocity comes from qdot (not Euler yaw derivative).
    Endpoints without observations on both sides are excluded, not extrapolated.
    """
    if world_frame != 'odom' or base_frame != 'base_link':
        raise ValueError('explicit odom -> base_link calibration is required')
    if (any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in
            (window_s, max_gap_s, max_speed_mps, max_yaw_rate_rps, max_acceleration_mps2))
            or type(min_samples) is not int or min_samples < 3):
        raise ValueError('invalid motion estimation limits')
    _frame(pose_frame)
    graph = StaticTransformGraph(static_transforms)
    mount = graph.transform(base_frame, pose_frame)
    inverse_mount_q = conjugate(mount['rotation'])
    prepared, rejected = [], []
    segment, previous, seen, highest_stamp = 0, None, set(), -math.inf
    for index, pose in enumerate(poses):
        try:
            stamp = pose['stamp']
            if isinstance(stamp, bool) or not isinstance(stamp, Real) or not math.isfinite(stamp) or stamp <= 0:
                raise ValueError('invalid pose ROS stamp')
            if pose.get('world_frame', world_frame) != world_frame or pose.get('pose_frame', pose_frame) != pose_frame:
                raise ValueError('pose frame disagrees with explicit trajectory convention')
            if stamp in seen:
                raise ValueError('duplicate pose stamp')
            seen.add(stamp)
            if stamp < highest_stamp:
                raise ValueError('pose ROS clock rollback; refusing overlapping time segments')
            highest_stamp = stamp
            position, orientation = finite_vector(pose['position']), quaternion(pose['orientation'])
            body_q = quaternion(multiply(orientation, inverse_mount_q))
            body_p = [a-b for a, b in zip(position, rotate(mount['translation'], body_q))]
            sample = dict(stamp=float(stamp), position=body_p, orientation=body_q, source_index=index)
            if previous:
                dt = stamp-previous['stamp']
                if dt < 1e-5:
                    raise ValueError('non-monotonic or near-duplicate pose stamp')
                if dt > max_gap_s:
                    segment += 1
                else:
                    distance = math.sqrt(sum((a-b)**2 for a, b in zip(body_p, previous['position'])))
                    dot = sum(a*b for a, b in zip(body_q, previous['orientation']))
                    angle = 2*math.acos(min(1., abs(dot)))
                    if distance/dt > max_speed_mps or angle/dt > max_yaw_rate_rps:
                        raise ValueError('pose translation/rotation jump')
                # q and -q are equivalent; derivatives need a continuous sign.
                if sum(a*b for a, b in zip(body_q, previous['orientation'])) < 0:
                    sample['orientation'] = [-v for v in body_q]
            sample['segment'] = segment
            prepared.append(sample)
            previous = sample
        except (ValueError, KeyError, TypeError) as error:
            rejected.append(dict(source='pose', index=index, reason=str(error)))
            segment += 1
            previous = None
    samples = []
    groups = {}
    for pose in prepared:
        groups.setdefault(pose['segment'], []).append(pose)
    half = window_s/2
    for group in groups.values():
        times = [row['stamp'] for row in group]
        for index, pose in enumerate(group):
            stamp = pose['stamp']
            begin, end = bisect_left(times, stamp-half), bisect_right(times, stamp+half)
            window = group[begin:end]
            if len(window) < min_samples or begin >= index or end <= index+1:
                rejected.append(dict(source='motion', index=pose['source_index'], reason='insufficient centered derivative window'))
                continue
            try:
                velocity, acceleration, qdot = [], [], []
                local_times = times[begin:end]
                for axis in range(3):
                    first, second = _fit(local_times, [p['position'][axis] for p in window], stamp, half)
                    velocity.append(first)
                    acceleration.append(second)
                for axis in range(4):
                    derivative, _ = _fit(local_times, [p['orientation'][axis] for p in window], stamp, half)
                    qdot.append(derivative)
                angular = [2*v for v in multiply(conjugate(pose['orientation']), qdot)[:3]]
                velocity_body = rotate(velocity, conjugate(pose['orientation']))
                acceleration_body = rotate(acceleration, conjugate(pose['orientation']))
                if (not all(math.isfinite(v) for v in velocity_body+acceleration_body+angular)
                        or math.sqrt(sum(v*v for v in velocity_body)) > max_speed_mps
                        or math.sqrt(sum(v*v for v in acceleration_body)) > max_acceleration_mps2
                        or math.sqrt(sum(v*v for v in angular)) > max_yaw_rate_rps):
                    raise ValueError('estimated motion outside quality limits')
                samples.append(dict(stamp=stamp, position=pose['position'], orientation=pose['orientation'],
                                    velocity_body=velocity_body, acceleration_body=acceleration_body,
                                    angular_velocity_body=angular, speed_mps=velocity_body[0],
                                    acceleration_mps2=acceleration_body[0], yaw_rate_rps=angular[2],
                                    segment=pose['segment'], source_index=pose['source_index'],
                                    window_start=window[0]['stamp'], window_end=window[-1]['stamp'],
                                    window_samples=len(window)))
            except ValueError as error:
                rejected.append(dict(source='motion', index=pose['source_index'], reason=str(error)))
    return dict(samples=samples, rejected=rejected,
                provenance=dict(world_frame=world_frame, input_pose_frame=pose_frame, base_frame=base_frame,
                                body_to_pose=mount, derivative='centered weighted local cubic; no extrapolation',
                                angular_velocity='2*vector(conjugate(q_world_body)*qdot_world_body)',
                                window_s=window_s, max_gap_s=max_gap_s, min_samples=min_samples,
                                max_speed_mps=max_speed_mps, max_angular_rate_rps=max_yaw_rate_rps,
                                max_acceleration_mps2=max_acceleration_mps2,
                                acceleration='kinematic acceleration at base_link; excludes gravity'))
