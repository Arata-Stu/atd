"""シミュレーションの発着時刻と cuVSLAM が実際に処理したフレーム時刻を突き合わせる。"""
from bisect import bisect_left, bisect_right
from math import dist, isfinite

SECOND_NS = 1_000_000_000
# This is a coverage check, not the cuVSLAM image synchronization tolerance.
MAX_GAP_NS = SECOND_NS


class Tracking:
    def __init__(self):
        self.odometry_times = []
        self.positions = []
        self.status_times = []
        self.states = []

    def add_odometry(self, stamp, position):
        if not all(isfinite(value) for value in position):
            raise ValueError('VO の位置に非有限値があります')
        if self._append_time(self.odometry_times, stamp):
            self.positions.append(position)

    def add_status(self, stamp, state):
        if self._append_time(self.status_times, stamp):
            self.states.append(state)

    @staticmethod
    def _append_time(stamps, stamp):
        if stamps and stamp < stamps[-1]:
            raise ValueError('cuVSLAM の時刻が逆行しました')
        if stamp <= 0 or (stamps and stamp == stamps[-1]):
            return False
        stamps.append(stamp)
        return True

    def ready(self, clock_ns):
        return (clock_ns > 0 and self.states[-1:] == [1] and bool(self.odometry_times)
                and abs(clock_ns - self.odometry_times[-1]) <= MAX_GAP_NS
                and abs(clock_ns - self.status_times[-1]) <= MAX_GAP_NS)

    def reached(self, stamp):
        return (bool(self.odometry_times and self.status_times)
                and min(self.odometry_times[-1], self.status_times[-1]) >= stamp)

    def assess(self, start, finish):
        if start <= 0 or finish <= start:
            raise ValueError('周回の発着時刻が不正です')
        result = {'start_sim_ns': start, 'finish_sim_ns': finish,
                  'coverage': 'continuous', 'gap_threshold_sim_s': MAX_GAP_NS / SECOND_NS,
                  'first_loss_lap_s': None, 'first_gap_lap_s': None}
        for name, stamps in (('status', self.status_times), ('odometry', self.odometry_times)):
            before = bisect_right(stamps, start) - 1
            after = bisect_left(stamps, finish)
            if before < 0 or after >= len(stamps):
                raise ValueError(f'{name} の時刻が周回の両端を含んでいません')
            # The first good frame after a long outage cannot confirm tracking at the goal.
            for edge in (start, finish):
                left, right = bisect_right(stamps, edge)-1, bisect_left(stamps, edge)
                if max(edge - stamps[left], stamps[right] - edge) > MAX_GAP_NS:
                    raise ValueError(f'周回の発着時刻付近に {name} が届いていません')
                if name == 'status' and (self.states[left] != 1 or self.states[right] != 1):
                    raise ValueError('cuVSLAM が周回の発着時に追跡を確認できませんでした')
            interval = stamps[before:after + 1]
            result[name + '_samples'] = len(interval)
            result[name + '_max_gap_sim_s'] = max(b - a for a, b in zip(interval, interval[1:])) / SECOND_NS
            for a, b in zip(interval, interval[1:]):
                if b - a > MAX_GAP_NS:
                    first = max(0, a - start) / SECOND_NS
                    if result['first_gap_lap_s'] is None or first < result['first_gap_lap_s']:
                        result['first_gap_lap_s'] = first
                    break
            if name == 'status':
                losses = [stamp for stamp, state in zip(interval, self.states[before:after + 1]) if state != 1]
                result['loss_samples'] = len(losses)
                if losses:
                    result['first_loss_lap_s'] = (losses[0] - start) / SECOND_NS
            else:
                positions = self.positions[before:after + 1]
                if max(dist(p, positions[0]) for p in positions) < .01:
                    raise ValueError('周回中に VO による移動を確認できませんでした')
                result['vo_distance_m'] = sum(dist(a, b) for a, b in zip(positions, positions[1:]))
                result['vo_endpoint_separation_m'] = dist(positions[0], positions[-1])
        interruptions = [t for t in (result['first_loss_lap_s'], result['first_gap_lap_s']) if t is not None]
        result['first_interruption_lap_s'] = min(interruptions, default=None)
        if interruptions:
            result['coverage'] = 'interrupted'
        return result


def map_databases(folder):
    files = {}
    for path in sorted(folder.glob('*.mdb')):
        if path.name == 'lock.mdb':
            continue
        if path.is_symlink() or not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f'cuVSLAM 地図のデータベースが不完全です: {path.name}')
        files[path.name] = path.stat().st_size
    if not files:
        raise ValueError('cuVSLAM の保存先に非空の地図データベース (*.mdb) がありません')
    return files
