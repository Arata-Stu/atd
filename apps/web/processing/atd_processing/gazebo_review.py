"""閉ループ記録をシミュレーション時刻で対応させ、Gazebo world の後輪軸で比較する。"""
from array import array
from bisect import bisect_left
import math

from shapely import prepare
from shapely.geometry import Polygon

from .geometry import corridor, points
from .planning.footprint import rectangle


def finite(value):
    return type(value) in (float, int) and abs(value) < 1e100 and math.isfinite(value)


def number(value):
    return value if finite(value) else None


def stamp(value):
    return value if type(value) is int and 0 < value < 2**63 else None


def delay_ms(start, end):
    return (end - start) * 1e-6 if start is not None and end is not None and end >= start else None


# Original CourseEvaluation constants used before the dimensions were archived in scene.json.
LEGACY_FOOTPRINT = dict(vehicle_length=.35, vehicle_width=.19, rear_axle_to_rear=.1065, margin=0.)


def mesh_boundaries(stream):
    """Reconstruct the two sides from the shared transverse edges of the archived road ribbon."""
    vertices = []
    for line in stream:
        if line.startswith('v '):
            fields = line.split()
            if len(fields) != 4:
                raise ValueError('舗装メッシュの頂点が不正です')
            x, y, z = map(float, fields[1:])
            if not (finite(x) and finite(y) and z == 0):
                raise ValueError('舗装メッシュが平面にありません')
            vertices.append((x, y))
    if len(vertices) < 12 or len(vertices) % 4:
        raise ValueError('舗装メッシュの帯を復元できません')
    quads = [set(vertices[i:i+4]) for i in range(0, len(vertices), 4)]
    gates = [q & quads[i-1] for i, q in enumerate(quads)]
    if any(len(gate) != 2 for gate in gates):
        raise ValueError('舗装メッシュの隣接区間がつながっていません')
    mid = [(sum(x for x, y in gate)/2, sum(y for x, y in gate)/2) for gate in gates]
    left, right = [], []
    for i, (a, b) in enumerate(gates):
        prev, following = mid[i-1], mid[(i+1) % len(mid)]
        dx, dy = following[0] - prev[0], following[1] - prev[1]
        if dx*(a[1] - b[1]) - dy*(a[0] - b[0]) < 0:
            a, b = b, a
        left.append(list(a))
        right.append(list(b))
    return dict(left=left, right=right)


class Road:
    def __init__(self, boundaries, footprint):
        self.boundaries = {side: points(boundaries.get(side), minimum=3) for side in ('left', 'right')}
        self.area = corridor(dict(closed=True, **self.boundaries))
        self.walls = self.area.boundary
        prepare(self.area)
        if (not isinstance(footprint, dict)
                or any(not finite(footprint.get(k)) for k in LEGACY_FOOTPRINT)
                or not 0 <= footprint['rear_axle_to_rear'] < footprint['vehicle_length']
                or footprint['vehicle_width'] <= 0 or footprint['margin'] != 0):
            raise ValueError('評価用の車体寸法が不正です')
        self.footprint = {k: footprint[k] for k in LEGACY_FOOTPRINT}

    def clearance(self, row):
        body = Polygon(rectangle((row['x'], row['y']), row['yaw_rad'], self.footprint))
        return self.walls.distance(body) if self.area.covers(body) else 0.


class DisplaySeries:
    """Bound the overview while keeping native samples around a recorded contact."""
    def __init__(self, contact):
        self.contact = contact
        self.rows = []
        self.near = []
        self.count = 0
        self.stride = 1
        self.last = None

    def append(self, row):
        if self.contact is not None and abs(row['t'] - self.contact) <= 10:
            self.near.append(row)
        elif self.count % self.stride == 0:
            self.rows.append(row)
            if len(self.rows) == 8192:
                self.rows = self.rows[::2]
                self.stride *= 2
        self.count += 1
        self.last = row

    def result(self):
        result = sorted(self.rows + self.near, key=lambda row: row['t'])
        if self.last is not None and (not result or result[-1] is not self.last):
            result.append(self.last)
        return result


class GroundTruth:
    def __init__(self):
        self.ns = array('q')
        self.pose = array('d')

    def append(self, ns, x, y, yaw):
        if self.ns and ns <= self.ns[-1]:
            raise ValueError('GT のシミュレーション時刻が単調増加していません')
        self.ns.append(ns)
        self.pose.extend((x, y, yaw))

    def at(self, ns):
        i = bisect_left(self.ns, ns)
        if i == len(self.ns):
            return None
        x, y, yaw = self.pose[3*i:3*i+3]
        if self.ns[i] == ns:
            return x, y, yaw
        if i == 0 or self.ns[i] - self.ns[i-1] > 250_000_000:
            return None
        a, b, angle = self.pose[3*(i-1):3*i]
        r = (ns-self.ns[i-1]) / (self.ns[i]-self.ns[i-1])
        return a+r*(x-a), b+r*(y-b), angle+r*math.remainder(yaw-angle, math.tau)


def motion_sample(row, truth):
    t = number(row.get('sim_time_s'))
    x, y, yaw = (number(row.get(k)) for k in ('x_m', 'y_m', 'yaw_rad'))
    if t is None or not 0 <= t < 2**31 or None in (x, y, yaw):
        raise ValueError('GT のシミュレーション時刻または後輪軸姿勢が不正です')
    ns = round(t * 1e9)
    truth.append(ns, x, y, yaw)
    state = row.get('actuator_state')
    image = stamp(row.get('command_stamp_ns')) if state == 'driving' else None
    applied = stamp(row.get('command_applied_ns'))
    control = row.get('control_state')
    if not isinstance(control, dict) or control.get('state') != 'driving':
        control = {}
    control_image, publish = stamp(control.get('image_stamp_ns')), stamp(control.get('stamp_ns'))
    return dict(t=t, x=x, y=y, yaw_rad=yaw, actuator_state=state if isinstance(state, str) else None,
                speed_mps=number(row.get('speed_mps')), steering_rad=number(row.get('steering_rad')),
                requested_steering_rad=number(row.get('requested_steering_rad')), clearance_m=None,
                tracking_lateral_error_m=number(control.get('lateral_error_m')) if control_image else None,
                control_line=control.get('line') if control_image and control.get('line') in ('centerline', 'raceline', 'custom') else None,
                control_image_s=control_image/1e9 if control_image else None,
                image_to_control_ms=delay_ms(control_image, publish), command_age_ms=delay_ms(image, ns),
                image_to_apply_ms=delay_ms(image, applied) if applied is not None and applied <= ns else None)


class PoseComparison:
    def __init__(self, truth, contact):
        self.truth = truth
        self.series = DisplaySeries(contact)
        self.anchor = None
        self.anchor_s = None
        self.last_stamp = 0
        self.segment = 0
        self.active_segment = None

    def to_world(self, x, y):
        gx, gy, mx, my, c, s, _ = self.anchor
        return [gx + c*(x-mx) - s*(y-my), gy + s*(x-mx) + c*(y-my)]

    def observe(self, row, legacy=False):
        tracking = row.get('state') == ('driving' if legacy else 'tracking')
        if not tracking:
            self.active_segment = None
            return
        ns = stamp(row.get('image_stamp_ns'))
        if ns is None or ns <= self.last_stamp:
            return
        gt = self.truth.at(ns)
        x, y = number(row.get('x_m')), number(row.get('y_m'))
        yaw = number(row.get('yaw_rad'))
        if gt is None or None in (x, y, yaw):
            self.active_segment = None
            return
        gx, gy, gyaw = gt
        if self.anchor is None:
            angle = math.remainder(gyaw-yaw, math.tau)
            self.anchor = (gx, gy, x, y, math.cos(angle), math.sin(angle), angle)
            self.anchor_s = ns/1e9
        pair = row.get('session_id'), row.get('epoch')
        if self.active_segment != pair or ns - self.last_stamp > 250_000_000:
            self.segment += 1
        self.last_stamp, self.active_segment = ns, pair
        wx, wy = self.to_world(x, y)
        wyaw = math.remainder(yaw + self.anchor[-1], math.tau)
        lateral = -math.sin(gyaw)*(wx-gx) + math.cos(gyaw)*(wy-gy)
        self.series.append(dict(t=ns/1e9, x=wx, y=wy, yaw_rad=wyaw, lateral_error_m=lateral,
                                yaw_error_rad=math.remainder(wyaw-gyaw, math.tau), segment=self.segment,
                                received_delay_ms=None if legacy else delay_ms(ns, stamp(row.get('received_sim_ns')))))


def timing_sample(row):
    ns = stamp(row.get('image_stamp_ns'))
    if ns is None:
        return None

    def duration(name):
        seconds = number(row.get(name))
        return seconds*1000 if seconds is not None and seconds >= 0 else None

    vo = row.get('vo_state')
    return dict(t=ns/1e9, track_ms=duration('track_execution_time_s'),
                callback_ms=duration('node_callback_execution_time_s'), vo_state=vo if type(vo) is int else None)
