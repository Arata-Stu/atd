"""保存 HDMap 上の閉ループ走行。Notebook のみで実行する。"""
import math
from collections.abc import Mapping
from statistics import fmean

from shapely import prepare
from shapely.geometry import Polygon

from atd_control.simulation import _model, _run, default_controllers, validate_settings
from atd_planning.core import ReferenceLine

from .geometry import DEFAULTS, corridor
from .planning.footprint import rectangle, swept


# 保存ラインの始点に対する横方向 [m] と姿勢 [deg]。全方式で同一の静止状態から開始する。
STARTS = ((0., 0.), (.06, 3.), (-.06, -3.))
FOOTPRINT = dict(vehicle_length=DEFAULTS['vehicle_length'], vehicle_width=DEFAULTS['width'],
                 rear_axle_to_rear=DEFAULTS['rear_axle_to_rear'], margin=0.)


class WallContact:
    """車体四隅の掃引領域と内外壁の接触を検出する。生成時の余裕幅は物理外形に加算しない。"""
    def __init__(self, road):
        self.road = road
        prepare(road)
        self.walls = road.boundary
        # 最後に検証した車体からどの隅が何 m 動いても壁に届かないか。
        self.clearance = 0.
        self.radius = max(math.hypot(x, y) for x, y in rectangle((0., 0.), 0., FOOTPRINT))

    def __call__(self, start, end):
        a, b = start[:2], end[:2]
        turn = abs(math.remainder(end[2] - start[2], math.tau))
        travel = math.dist(a, b)
        # 各物理積分区間の並進・回転および短い旋回弧の弦からのずれを保守的に覆う。
        arc_padding = travel * turn / 4
        motion = travel + self.radius * turn + arc_padding
        if self.clearance <= motion:
            before = Polygon(rectangle(a, start[2], FOOTPRINT))
            if not self.road.contains_properly(before):
                return True
            self.clearance = self.walls.distance(before)
        if self.clearance > motion:
            self.clearance -= motion
            return False
        for _, corners, turn_padding in swept([a, b], False, FOOTPRINT, [start[2], end[2]]):
            sweep = Polygon(corners)
            if (not self.road.contains_properly(sweep)
                    or self.walls.dwithin(sweep, turn_padding + arc_padding)):
                return True
        self.clearance = 0.
        return False


class _Course:
    def __init__(self, document, line, settings, laps, controllers=None):
        if type(laps) is not int or not 1 <= laps <= 5:
            raise ValueError('目標周回数は 1〜5 の整数で指定してください')
        self.settings = validate_settings({} if settings is None else settings)
        if controllers is None:
            controllers = default_controllers(self.settings)
        if (not isinstance(controllers, Mapping) or not controllers or
                any(not isinstance(key, str) or not key.strip() or not callable(build)
                    for key, build in controllers.items())):
            raise ValueError('操舵方式名と試行ごとのファクトリを指定してください')
        self.controllers = dict(controllers)
        self.reference = ReferenceLine.from_document(document, line)
        if not self.reference.closed:
            raise ValueError('周回ベンチマークには閉路の HDMap を指定してください')
        self.road = corridor(document)
        self.laps = laps
        # 速度プロファイルに従った走行時間の 2.5 倍と発進猶予。両方式に同じ上限を適用する。
        speeds = self.reference.speeds
        ideal = sum(edge.length / max(.2, (speeds[i] + speeds[(i+1) % len(speeds)]) / 2)
                    for i, edge in enumerate(self.reference.segments))
        self.timeout_s = math.ceil(10 + 2.5 * laps * ideal)

    def run(self, mode, trial, *, trace, check):
        offset, angle = STARTS[trial]
        # 長いコースでも図の解像度に十分な点を残しつつ、詳細表示の応答サイズを抑える。
        stride = max(5, math.ceil(self.timeout_s / (.02 * 6000))) if trace else 0
        steer = self.controllers[mode]()
        if not callable(steer):
            raise ValueError(f'{mode} のファクトリから操舵関数を返してください')
        return _run(self.reference, steer, mode, self.settings, laps=self.laps,
                    time_limit_s=self.timeout_s, start_offset_m=offset, start_yaw_rad=math.radians(angle),
                    contact=WallContact(self.road), sample_stride=stride, check=check)

    def model(self):
        return dict(_model(self.laps), vehicle_length_m=FOOTPRINT['vehicle_length'],
                    vehicle_width_m=FOOTPRINT['vehicle_width'], rear_overhang_m=FOOTPRINT['rear_axle_to_rear'],
                    time_limit_s=self.timeout_s)


def evaluate_course(document, settings=None, *, line='raceline', laps=2, trials=3,
                    controllers=None, check=lambda: None):
    """各試行は最初の壁接触で終了する。完走率の分母には打ち切りを含む全試行を使う。"""
    if type(trials) is not int or not 1 <= trials <= len(STARTS):
        raise ValueError(f'試行数は 1〜{len(STARTS)} の整数で指定してください')
    course = _Course(document, line, settings, laps, controllers)
    results = []
    for mode in course.controllers:
        runs = []
        for trial in range(trials):
            data = course.run(mode, trial, trace=False, check=check)
            offset, angle = STARTS[trial]
            runs.append(dict(trial=trial + 1, start=dict(lateral_offset_m=offset, yaw_offset_deg=angle),
                             outcome=data['outcome'], reason=data['reason'], lap_times_s=data['lap_times_s'],
                             progress_m=data['progress_m'], collision=data['collision'], **data['metrics']))
        times = [t for run in runs for t in run['lap_times_s']]
        finishes = sum(r['outcome'] == 'finished' for r in runs)
        collisions = sum(r['outcome'] == 'collision' for r in runs)
        results.append(dict(mode=mode, trials=trials, finishes=finishes, finish_rate=finishes / trials,
                            collisions=collisions, mean_lap_time_s=fmean(times) if times else None,
                            mean_trial_rms_lateral_error_m=fmean(r['rms_lateral_error_m'] for r in runs),
                            best_lap_time_s=min(times) if times else None, runs=runs))
    return dict(course_id=document['id'], line=line, length_m=course.reference.length, settings=course.settings,
                model=course.model(), laps=laps, trials=trials, results=results)


def compare_course(document, settings=None, *, line='raceline', laps=2, trial=1, check=lambda: None):
    """指定試行の位置・操作時系列を返す。番号と初期条件は evaluate_course と共通。"""
    if type(trial) is not int or not 1 <= trial <= len(STARTS):
        raise ValueError(f'表示する試行は 1〜{len(STARTS)} の整数で指定してください')
    course = _Course(document, line, settings, laps)
    def xy(points):
        return [dict(x_m=x, y_m=y) for x, y in points]

    return dict(course=dict(id=document['id'], name=document.get('title') or document['id'],
                            line=line, speeds_mps=course.reference.speeds,
                            length_m=course.reference.length, points=xy(document['lines'][line]['points']),
                            left=xy(document['left']), right=xy(document['right'])),
                model=course.model(), settings=course.settings, trial=trial,
                runs=[course.run(mode, trial-1, trace=True, check=check) for mode in course.controllers])
