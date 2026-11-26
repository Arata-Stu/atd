"""cuVSLAM の同じ画像時刻の診断・map→odom・VO を結合する。ROS 非依存。"""
import math
from dataclasses import dataclass

from atd_mapping.frames import rotate


@dataclass(frozen=True)
class RigidPose:
    xyz: tuple
    xyzw: tuple

    @classmethod
    def checked(cls, xyz, xyzw):
        values = (*xyz, *xyzw)
        if not all(math.isfinite(x) for x in values) or len(xyz) != 3 or len(xyzw) != 4:
            raise ValueError('非有限値または不正な姿勢です')
        norm = math.sqrt(sum(v * v for v in xyzw))
        if abs(norm - 1.0) > 0.01:
            raise ValueError('クォータニオンが正規化されていません')
        return cls(tuple(xyz), tuple(v / norm for v in xyzw))

    def compose(self, child):
        x, y, z, w = self.xyzw
        a, b, c, d = child.xyzw
        p = rotate(child.xyz, self.xyzw)
        return RigidPose.checked(tuple(u + v for u, v in zip(self.xyz, p)),
                                 (w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
                                  w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c))


@dataclass(frozen=True)
class Observation:
    stamp: int
    pose: RigidPose
    speed: float
    yaw_rate: float


class TrackingState:
    def __init__(self, max_age_s=1.0, *, allow_sparse_tf=False):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('鮮度の判定秒数は正の有限値で指定してください')
        self.max_age_s = max_age_s
        self.allow_sparse_tf = allow_sparse_tf
        self.epoch = 0
        self.clock_ns = -1
        self.sources = {}
        self.conflict = ''
        self.camera_mismatch = None
        self._clear()

    def _clear(self):
        self.cameras = {}
        self.frames = {}
        self.corrections = {}
        self.registered_since = None
        self.last_sample = None
        self.last_output = None
        self.last_diag = self.barrier = self.hint_stamp = -1
        self.registered = self.needs_clear = self.lost = False
        self.was_tracking = False

    def clock(self, now_ns):
        if now_ns < self.clock_ns:
            self.epoch += 1
            self._clear()
        self.clock_ns = now_ns

    def _valid_stamp(self, stamp):
        # A queued message from before a /clock rewind must not advance stream
        # watermarks into the future and suppress the entire new playback.
        return 0 < stamp <= self.clock_ns + 100_000_000

    def source(self, stream, gid):
        # Only pin the publisher of participating messages (e.g. the map→odom TF).
        if not any(gid) or self.sources.setdefault(stream, gid) != gid:
            self.conflict = f'{stream} の配信元を一意に確認できません。起動構成を確認してください'
        return not self.conflict

    def camera(self, side, stamp, receipt, matches):
        previous = self.cameras.get(side)
        if self._valid_stamp(stamp) and (previous is None or stamp > previous[0]):
            # cuVSLAM initializes its intrinsics once; seeing matching metadata
            # later does not prove that it has reloaded the correct calibration.
            if not matches:
                self.camera_mismatch = side
            self.cameras[side] = (stamp, receipt)

    def hint(self, stamp):
        if self._valid_stamp(stamp) and stamp > max(self.barrier, self.hint_stamp):
            self.hint_stamp = stamp

    def diagnostic(self, stamp, receipt, values):
        if not self._valid_stamp(stamp) or stamp <= self.last_diag:
            return
        self.last_diag = stamp
        localized = values.get('localized_in_exist_map')
        vo_ok = values.get('vo_status') == 'OK'
        if localized == 'No':
            self.needs_clear = False
        if localized != 'Yes' or not vo_ok:
            if self.registered:
                self.epoch += 1
                self.registered = self.was_tracking = False
                self.lost = True
                self.needs_clear = localized != 'No'
                self.hint_stamp = -1
                self.barrier = stamp
                self.frames.clear()
                self.corrections.clear()
                self.registered_since = None
                self.last_sample = None
            return
        # A "Yes" still latched from before a VO loss cannot establish recovery.
        if self.hint_stamp <= self.barrier or stamp < self.hint_stamp or self.needs_clear:
            return
        if not self.registered:
            self.registered_since = stamp
        self.registered = True
        self.lost = False
        self._part(stamp, receipt, 'ok', True)

    def transform(self, stamp, receipt, pose):
        if self.allow_sparse_tf:
            if self._valid_stamp(stamp) and stamp > self.barrier:
                self.corrections[stamp] = pose
                if len(self.corrections) > 128:
                    del self.corrections[min(self.corrections)]
            return
        self._part(stamp, receipt, 'transform', pose)

    def odometry(self, stamp, receipt, pose, speed, yaw_rate):
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            return
        self._part(stamp, receipt, 'odometry', (pose, speed, yaw_rate))

    def _part(self, stamp, receipt, kind, value):
        if not self._valid_stamp(stamp) or stamp <= self.barrier or (self.last_sample and stamp <= self.last_sample[0].stamp):
            return
        # cuVSLAM uses one timestamp_output for all three. A small bounded buffer
        # allows DDS to deliver the three streams in different orders at 90 Hz.
        parts = self.frames.setdefault(stamp, {})
        if kind in parts:
            return
        parts[kind] = (value, receipt)
        if len(self.frames) > 128:
            del self.frames[min(self.frames)]
        if len(parts) == 3:
            self._compose(stamp, parts, parts['transform'][0])
            self.frames.pop(stamp, None)

    def _compose(self, stamp, parts, correction):
        vo_pose, speed, yaw_rate = parts['odometry'][0]
        pose = correction.compose(vo_pose)
        self.last_sample = (Observation(stamp, pose, speed, yaw_rate), min(w for _, w in parts.values()))

    def _apply_sparse_correction(self):
        if self.registered_since is None or not self.corrections:
            return
        # VO and its tracking diagnostic still describe one image. At publication time
        # prefer its exact TF, or hold a measured correction from the same registration.
        stamp = max((t for t, parts in self.frames.items() if 'ok' in parts and 'odometry' in parts), default=0)
        if not stamp:
            return
        correction = max((t for t in self.corrections if self.registered_since <= t <= stamp), default=0)
        if correction:
            self._compose(stamp, self.frames[stamp], self.corrections[correction])
            self.frames = {t: parts for t, parts in self.frames.items() if t > stamp}

    def _fresh(self, stamp, received, wall):
        # /clock can trail an image by one clock tick on replay. Old timestamps and
        # stopped ROS clocks are caught independently using ROS and steady time.
        return (-0.1 <= (self.clock_ns - stamp) * 1e-9 <= self.max_age_s
                and 0 <= wall - received <= self.max_age_s)

    def status(self, wall):
        if self.allow_sparse_tf:
            self._apply_sparse_correction()
        state, reason, sample = self._status(wall)
        if self.was_tracking and state != 'tracking' and not (self.allow_sparse_tf and state == 'stale'):
            self.epoch += 1
        self.was_tracking = state == 'tracking'
        return state, reason, sample

    def next_status(self, wall, *, heartbeat=False):
        """Publish each observation once, with periodic status while awaiting usable poses."""
        state, reason, sample = self.status(wall)
        key = (self.epoch, state, reason, sample.stamp if sample is not None else None)
        if key == self.last_output and (sample is not None or not heartbeat):
            return None
        self.last_output = key
        return state, reason, sample

    def _status(self, wall):
        if self.conflict:
            return 'lost', self.conflict, None
        if self.camera_mismatch:
            label = '左' if self.camera_mismatch == 'left' else '右'
            return 'lost', f'{label} CameraInfo の解像度・光学フレームを地図設定に合わせて再起動してください', None
        for side, label in [('left', '左'), ('right', '右')]:
            camera = self.cameras.get(side)
            if camera is None:
                return 'waiting_hint', f'{label} CameraInfo の受信を待っています', None
            if not self._fresh(camera[0], camera[1], wall) and not (self.allow_sparse_tf and self.lost):
                return 'stale', f'新しい{label} CameraInfo を待っています', None
        if self.lost:
            return 'lost', '追跡喪失。新しい地図照合の確証が必要です。必要に応じて自己位置推定を再起動してください', None
        if self.hint_stamp < 0:
            return 'waiting_hint', 'VGL の初期位置候補を待っています', None
        if not self.registered:
            return 'localizing', 'cuVSLAM の保存地図への位置合わせを待っています', None
        if not self.last_sample or not self._fresh(self.last_sample[0].stamp, self.last_sample[1], wall):
            if self.allow_sparse_tf:
                return 'stale', '保存地図の補正 TF と、新しい同時刻の追跡診断・車体オドメトリを待っています', None
            return 'stale', '同じ観測時刻の追跡診断・map→odom・車体オドメトリを待っています', None
        return 'tracking', '保存地図上で追跡中', self.last_sample[0]
