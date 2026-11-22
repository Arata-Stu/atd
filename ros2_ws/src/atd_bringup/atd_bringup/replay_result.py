"""同一画像時刻の Gazebo・cuVSLAM 姿勢を走路座標で比較して保存する。"""
from bisect import bisect_left, bisect_right
from datetime import datetime, timezone
import fcntl
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from atd_mapping.frames import rotate


GAP_NS = 250_000_000


def _write_json(path, value):
    temp = path.with_suffix('.tmp')
    with temp.open('w', encoding='utf-8') as file:
        json.dump(value, file, allow_nan=False, ensure_ascii=False, separators=(',', ':'))
        file.write('\n')
    temp.replace(path)


class CourseProgress:
    """正解の実測折線距離を積算し、計時線の間だけ既知の1周長に対応付ける。"""

    def __init__(self, trace, course):
        self.trace = trace
        self.distances = [0.0]
        for previous, current in zip(trace.poses, trace.poses[1:]):
            self.distances.append(self.distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y))
        self.boundaries = []
        self.lap_length = None
        self.boundary_distances = []
        laps, start, duration = (course.get(key) for key in ('laps', 'start_ns', 'lap_duration_s'))
        if laps is None or laps >= len(trace.stamps):
            return
        finish = start + round(laps * duration * 1e9)
        # post_update が開始・完走を記録する物理ステップの丸めを許容する。
        if trace.stamps[0] > start + 30_000_000 or trace.stamps[-1] < finish - 30_000_000:
            return
        # 初期の記録には計時区間外に静止したカメラ台のサンプルもある。
        if start - trace.stamps[0] > 30_000_000 and self.distance(start) > .05:
            return
        if trace.stamps[-1] - finish > 30_000_000 and self.distances[-1] - self.distance(finish) > .05:
            return
        boundaries = [start + round(i * duration * 1e9) for i in range(laps + 1)]
        distances = [self.distance(stamp) for stamp in boundaries]
        if any(b <= a for a, b in zip(distances, distances[1:])):
            return
        self.boundaries = boundaries
        self.lap_length = course['lap_length_m']
        self.boundary_distances = distances

    def distance(self, stamp):
        stamps = self.trace.stamps
        i = bisect_left(stamps, stamp)
        if i == 0:
            return 0.0
        if i == len(stamps):
            return self.distances[-1]
        fraction = (stamp - stamps[i-1]) / (stamps[i] - stamps[i-1])
        return self.distances[i-1] + fraction * (self.distances[i] - self.distances[i-1])

    def point(self, stamp, pose):
        distance = self.distance(stamp)
        lap, station = None, None
        if self.boundaries:
            lap = min(max(1, bisect_right(self.boundaries, stamp)), len(self.boundaries) - 1)
            start, end = self.boundary_distances[lap-1:lap+1]
            fraction = min(1., max(0., (distance - start) / (end - start)))
            station = self.lap_length * fraction
        return dict(time_s=(stamp - self.trace.stamps[0]) / 1e9, lap=lap, station_m=station,
                    distance_m=distance, gt_x_m=pose.x, gt_y_m=pose.y)


def compare(alignment, truth, position, orientation):
    """固定した map→Gazebo 変換で、推定と正解の差を後輪軸の前・左軸に射影する。"""
    x, y, z, w = alignment.map_from_world
    world_from_map = (-x, -y, -z, w)
    relative = [p - origin for p, origin in zip(position, alignment.map_origin)]
    x, y, _ = rotate(relative, world_from_map)
    est_x, est_y = alignment.world_origin.x + x, alignment.world_origin.y + y
    forward = rotate(rotate((1., 0., 0.), orientation), world_from_map)
    if math.hypot(*forward[:2]) < 1e-9:
        raise ValueError('車体前方軸の路面への射影が小さく、向きを計算できません')
    dx, dy = est_x - truth.x, est_y - truth.y
    c, s = math.cos(truth.yaw), math.sin(truth.yaw)
    est_yaw = math.atan2(forward[1], forward[0])
    yaw_error = math.remainder(est_yaw - truth.yaw, math.tau)
    return dict(est_x_m=est_x, est_y_m=est_y, along_m=c*dx + s*dy, lateral_m=-s*dx + c*dy,
                est_yaw_rad=est_yaw, gt_yaw_rad=truth.yaw, heading_deg=math.degrees(yaw_error))


class ReplayResult:
    def __init__(self, root, bag, rate, trace, course, *, slam_max_map_size=0, imu=None):
        now = datetime.now(timezone.utc)
        run_id = now.strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:12]
        self.directory = Path(root) / 'runs' / 'replay' / run_id
        self.directory.mkdir(parents=True)
        self.progress = CourseProgress(trace, course)
        self.segment = 0
        self.last_stamp = None
        self.last_cloud_stamp = None
        self.next_checkpoint = 0.
        self.committed = None
        self.summary = dict(version=1, id=run_id, bag=str(bag), rate=rate, created_at=now.isoformat(),
                            slam_max_map_size=slam_max_map_size, tracking='vio' if imu else 'vo',
                            imu_frame=imu.frame if imu else None, imu_hz=imu.frequency if imu else None,
                            status='running', reason=None, sample_count=0,
                            circuit=course.get('circuit'), laps=course.get('laps') if self.progress.boundaries else None,
                            lap_length_m=self.progress.lap_length, gap_threshold_s=GAP_NS / 1e9,
                            gt_start_stamp_ns=str(trace.stamps[0]), gt_finish_stamp_ns=str(trace.stamps[-1]),
                            anchor_stamp_ns=None, writer_lock='samples.jsonl',
                            loop_closure_graph=dict(status='not_checked' if len(self.progress.boundaries) >= 3
                                                    else 'no_lap_comparison'),
                            loop_closure_cloud=dict(updates=0, nonempty_updates=0,
                                                    first_nonempty_by_lap=[], last_nonempty=None))
        reference = [self.progress.point(stamp, pose) for stamp, pose in zip(trace.stamps, trace.poses)]
        _write_json(self.directory / 'reference.json', reference)
        self.file = (self.directory / 'samples.jsonl').open('x', encoding='utf-8')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.checkpoint(force=True)
        except BaseException:
            self.file.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, exc_type, error, traceback):
        self.close(error)

    def record(self, stamp, truth, alignment, position, orientation):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        row = self.progress.point(stamp, truth)
        row.update(compare(alignment, truth, position, orientation))
        if self.last_stamp is None:
            self.summary['anchor_stamp_ns'] = str(stamp)
            self.summary['alignment'] = dict(world_origin_xy_yaw=alignment.world_origin,
                                             map_origin_xyz=alignment.map_origin,
                                             map_from_world_xyzw=alignment.map_from_world)
        elif stamp - self.last_stamp > GAP_NS:
            self.segment += 1
        row.update(stamp_ns=str(stamp), segment=self.segment)
        self.file.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
        self.last_stamp = stamp
        self.summary['sample_count'] += 1

    def record_closure_cloud(self, stamp, point_count):
        """閉じ込み用点群の更新を画像時刻で周回に照合する。1更新は1回の閉じ込みを意味しない。"""
        if self.last_cloud_stamp is not None and stamp <= self.last_cloud_stamp:
            return
        bounds = self.progress.boundaries
        if bounds and not bounds[0] <= stamp <= bounds[-1]:
            return
        pose = self.progress.trace.at(stamp)
        if pose is None:
            return
        self.last_cloud_stamp = stamp
        cloud = self.summary['loop_closure_cloud']
        cloud['updates'] += 1
        if not point_count:
            return
        row = self.progress.point(stamp, pose)
        row.update(stamp_ns=str(stamp), point_count=point_count)
        cloud['nonempty_updates'] += 1
        cloud['last_nonempty'] = row
        first = cloud['first_nonempty_by_lap']
        if not first or first[-1]['lap'] != row['lap']:
            first.append(row)
            return row

    def closure_report(self):
        cloud = self.summary['loop_closure_cloud']
        lines = [f"ループ閉じ込み用点群（GT走行区間内）: 非空 {cloud['nonempty_updates']} / 受信 {cloud['updates']} 更新"]
        graph = self.summary['loop_closure_graph']
        if graph['status'] == 'cross_lap_cycle':
            edge = graph['example']
            lines.append(f"周回間の閉路を姿勢グラフで確認: {edge['from_lap']} 周目 {edge['from_station_m']:.1f} m と "
                         f"{edge['to_lap']} 周目 {edge['to_station_m']:.1f} m を結ぶ辺（GT上の距離 "
                         f"{edge['gt_separation_m']:.2f} m）")
        elif graph['status'] == 'no_lap_comparison':
            lines.append('姿勢グラフの周回間照合には2周以上のGTが必要です。')
        else:
            reason = graph.get('reason', '照合できたグラフに周回間の閉路がありません' if
                               graph['status'] == 'not_observed' else '再生末尾のグラフ検査を実行していません')
            lines.append(f"姿勢グラフの周回間閉路は未確認（{reason}）")
        return '\n'.join(lines)

    def checkpoint(self, *, force=False):
        now = time.monotonic()
        counts = self.summary['sample_count'], self.summary['loop_closure_cloud']['updates']
        if force or (now >= self.next_checkpoint and self.committed != counts):
            self.file.flush()
            _write_json(self.directory / 'summary.json', self.summary)
            self.next_checkpoint = now + 1.
            self.committed = counts

    def mark_eof(self):
        self.summary.update(status='complete', reason=None)
        self.checkpoint(force=True)

    def close(self, error=None):
        try:
            if error is not None and not isinstance(error, (KeyboardInterrupt, ReplayRewound)):
                self.summary.update(status='failed', reason=str(error))
            elif self.summary['status'] != 'complete' or isinstance(error, ReplayRewound):
                self.summary.update(status='partial', reason=str(error or '') or '再生終了前に中断しました')
            self.checkpoint(force=True)
        finally:
            self.file.close()


class ReplayRewound(RuntimeError):
    """時計が戻った再生を別の初期姿勢と混ぜずに終了する。"""
