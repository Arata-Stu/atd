"""保存地図に位置合わせできた区間と、観測できなかった時間を ROS 時刻で集計する。"""
import math


class TrackingReport:
    def __init__(self, stereo, max_gap_s, tail_tolerance_s=.5):
        self.stereo = stereo
        self.max_gap_ns = round(max_gap_s * 1e9)
        self.tail_ns = round(tail_tolerance_s * 1e9)
        self.last_state = None
        self.previous = None
        self.intervals = []
        self.events = []
        self.samples = 0
        self.session_id = None
        self.last_unique_pose = None

    def observe(self, session, epoch, state, reason, stamp, sample=None):
        if state not in ('waiting_hint', 'localizing', 'tracking', 'lost', 'stale'):
            raise ValueError('未知の追跡状態です: ' + state)
        if not session or self.session_id is not None and self.session_id != session:
            raise ValueError('検証中に自己位置推定のセッションが変わりました')
        self.session_id = session
        first, end = self.stereo['first_stamp_ns'], self.stereo['last_stamp_ns']
        if (state, epoch) != self.last_state:
            self.events.append(dict(stamp_ns=stamp, state=state, epoch=epoch, reason=reason))
            self.last_state = (state, epoch)
            self.previous = None
        if state != 'tracking':
            return False
        if not sample or not all(math.isfinite(v) for v in sample):
            raise ValueError('非有限の追跡結果を受信しました')
        # cuVSLAM can stamp either member of a stereo pair (within 5 ms).
        if not first - 5_000_000 <= stamp <= end + 5_000_000:
            raise ValueError('追跡姿勢の観測時刻が検証 bag のステレオ区間外です')
        if self.last_unique_pose:
            last_epoch, last_stamp = self.last_unique_pose
            if epoch == last_epoch and stamp == last_stamp:
                return False
            if stamp < last_stamp or epoch < last_epoch:
                raise ValueError('追跡姿勢の時刻または世代が逆行しました')
        self.last_unique_pose = (epoch, stamp)
        if self.previous is None or stamp - self.previous > self.max_gap_ns:
            self.intervals.append(dict(epoch=epoch, first_stamp_ns=stamp, last_stamp_ns=stamp, samples=0))
        interval = self.intervals[-1]
        interval.update(last_stamp_ns=stamp, samples=interval['samples'] + 1)
        self.previous = stamp
        self.samples += 1
        return True

    def summary(self):
        start, end = self.stereo['first_stamp_ns'], self.stereo['last_stamp_ns']
        duration = end - start
        covered = sum(max(0, min(i['last_stamp_ns'], end) - max(i['first_stamp_ns'], start)) for i in self.intervals)
        last_stamp = self.intervals[-1]['last_stamp_ns'] if self.intervals else None
        tail_tracked = bool(covered and self.previous is not None and end - self.previous <= self.tail_ns)
        return dict(outcome='tracked_to_end' if tail_tracked else 'partial_tracking' if self.samples else 'no_tracking',
                    session_id=self.session_id, stereo=self.stereo, pose_samples=self.samples, state_changes=self.events,
                    tracking_intervals=self.intervals, tracking_duration_s=covered / 1e9, stereo_duration_s=duration / 1e9,
                    tracking_time_fraction=covered / duration, tail_tracked=tail_tracked,
                    first_tracking_delay_s=max(0., (self.intervals[0]['first_stamp_ns'] - start) / 1e9) if self.samples else None,
                    final_pose_age_at_stereo_end_s=max(0., (end - last_stamp) / 1e9) if self.samples else None,
                    gap_tolerance_s=self.max_gap_ns / 1e9, tail_tolerance_s=self.tail_ns / 1e9)
