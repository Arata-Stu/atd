"""Notebook に保存した手動走行 bag を一度読み、選択区間だけを返す。"""
from contextlib import closing
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import sqlite3
from tempfile import TemporaryDirectory
from uuid import uuid4

from fastapi import APIRouter, Query, Request
from pydantic import BaseModel, ConfigDict, Field

from .jobs import ACTIVE
from . import manual_motion_sources, offline
from .store import confined

router = APIRouter(prefix='/manual-drive/replay')
NS = 1_000_000_000
MAX_WINDOW_S = 30
TOPICS = {
    '/teleop/command': 'DualSense の操舵・スロットル',
    '/vehicle/joy_drive/status': 'Pico CDC0 の CH3 選択・操作元・サーボ／ESC PWM',
    '/vehicle/pico/telemetry': 'Pico CDC1 のテレメトリ',
}


class Selection(BaseModel):
    model_config = ConfigDict(extra='forbid')
    bag: str = Field(min_length=1)


class TimelineRemoval(Selection):
    index_id: str = Field(pattern='^[0-9a-f]{64}$')


@lru_cache(maxsize=1)
def _extractor_version():
    packages = Path(__file__).resolve().parents[5] / 'ros2_ws/src'
    bridge = packages / 'atd_web_bridge/atd_web_bridge'
    files = [bridge / name for name in ('bag_drive_timeline.py', 'drive_monitor.py', 'bag_motion.py', 'localization.py')]
    files.append(packages / 'atd_vehicle_calibration/atd_vehicle_calibration/inputs.py')
    return [hashlib.sha256(path.read_bytes()).hexdigest() for path in files]


@dataclass
class Recording:
    bag: str
    directory: Path
    index: Path

    @property
    def resource(self):
        return 'manual-drive:' + self.index.stem


def recording(store, bag):
    directory = confined(store.record_root, bag)
    metadata = confined(directory, 'metadata.yaml', exists=False)
    if not metadata.is_file() or not metadata.stat().st_size:
        raise ValueError('記録停止後の metadata.yaml がある bag を選択してください')
    relative = directory.relative_to(store.record_root).as_posix()
    files = []
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError('bag 内の symlink は扱えません')
        if path.is_file():
            stat = path.stat()
            files.append((path.relative_to(directory).as_posix(), stat.st_dev, stat.st_ino, stat.st_size,
                          stat.st_mtime_ns, stat.st_ctime_ns))
    key = hashlib.sha256(json.dumps([relative, files, _extractor_version()]).encode()).hexdigest()
    return Recording(relative, directory, confined(store.state_root, f'manual-drive/{key}.sqlite', exists=False))


def _open(path):
    return closing(sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True))


def _summary(db):
    duration, = db.execute('SELECT duration_ns FROM bag').fetchone()
    topics = {topic: (count, reason) for topic, count, reason in db.execute('SELECT topic,messages,reason FROM topics')}
    warnings = []
    for topic, label in TOPICS.items():
        count, reason = topics.get(topic, (0, ''))
        if reason:
            warnings.append(f'{label}（{topic}）: {reason}')
        elif not count and topic != '/vehicle/pico/telemetry':
            warnings.append(f'{label} は未収録です（{topic}）')
    joy_count, joy_issue = topics.get('/teleop/command', (0, ''))
    board_count, board_issue = topics.get('/vehicle/joy_drive/status', (0, ''))
    if joy_count and not joy_issue and not db.execute('SELECT 1 FROM joy LIMIT 1').fetchone():
        warnings.append('この記録に有効な DualSense の操舵・スロットルはありません')
    if board_count and not board_issue and not db.execute(
            "SELECT 1 FROM board WHERE source='cdc0' LIMIT 1").fetchone():
        warnings.append('この記録に有効な Pico CDC0 の基板報告はありません')
    return duration, warnings


def _status(src, jobs):
    result = dict(bag=src.bag, index_id=src.index.stem, ready=src.index.is_file(), duration_s=None, max_window_s=MAX_WINDOW_S,
                  warnings=[], job_id=None, vo_job_id=None, can_generate_vo=False, vo_reason='', imu_reason='',
                  speed_reason='', default_speed_source=None, speed_sources=[])
    history = jobs.list()
    if result['ready']:
        with _open(src.index) as db:
            duration, result['warnings'] = _summary(db)
            result['duration_s'] = duration / NS
            stereo, = db.execute('SELECT stereo_ready FROM bag').fetchone()
            recorded = bool(db.execute('SELECT 1 FROM speed LIMIT 1').fetchone())
            issues = dict(db.execute("SELECT topic,reason FROM topics WHERE reason != ''"))
            imu_issues = [issues[key] for key in ('/camera/camera/accel/sample', '/camera/camera/imu', '/tf_static')
                          if key in issues]
            result['imu_reason'] = ' '.join(dict.fromkeys(imu_issues))
            if not result['imu_reason'] and not db.execute('SELECT 1 FROM imu LIMIT 1').fetchone():
                result['imu_reason'] = 'この bag に車体前方へ変換できる IMU 加速度がありません'
        sources = manual_motion_sources.sources(jobs.store, src.bag)
        if recorded:
            sources = [dict(id='recorded', label='走行中の VSLAM（記録済み）'), *sources]
        result['speed_sources'] = sources
        if recorded or len(sources) == 1:
            result['default_speed_source'] = sources[0]['id']
        elif not sources:
            result['speed_reason'] = issues.get('/localization/status') or 'この走行の有効な VSLAM 速度はまだありません'
        else:
            result['speed_reason'] = '比較に使う VSLAM の結果を選択してください'
        result['can_generate_vo'] = bool(stereo)
        if not stereo:
            result['vo_reason'] = 'ステレオからの生成には左右 IR 画像・CameraInfo と base_link から両光学フレームへの静的 TF の記録が必要です'
        job = next((j for j in history if _vo_resource(src) in j['resources']), None)
        if job:
            result['vo_job_id'] = job['id']
            if job['status'] == 'succeeded':
                generated = (job.get('result') or {}).get('speed_source')
                if any(s['id'] == generated for s in sources):
                    result['can_generate_vo'] = False
    else:
        job = next((j for j in history if src.resource in j['resources']), None)
        if job:
            result['job_id'] = job['id']
    return result


@router.get('')
def status(request: Request, bag: str):
    return _status(recording(request.app.state.store, bag), request.app.state.jobs)


@router.delete('')
def delete_timeline(request: Request, choice: TimelineRemoval):
    store, jobs = request.app.state.store, request.app.state.jobs
    src = recording(store, choice.bag)
    # 同じ走行から時系列を準備する処理・その時系列を読む VO ジョブと直列化する。
    with store.lock(src.resource), store.lock(_vo_resource(src)), jobs.registry():
        jobs.require_available([src.resource, _vo_resource(src)])
        if src.index.stem != choice.index_id:
            raise ValueError('選択後に走行データが更新されました。時系列を再読み込みしてください')
        if not src.index.is_file():
            raise ValueError('選択した走行に抽出済みの時系列がありません')
        src.index.unlink()
    return dict(bag=src.bag)


@router.post('')
def start(request: Request, choice: Selection):
    store, jobs = request.app.state.store, request.app.state.jobs
    src = recording(store, choice.bag)
    with store.lock(src.resource):
        result = _status(src, jobs)
        if result['ready'] or (result['job_id'] and jobs.get(result['job_id'])['status'] in ACTIVE | {'orphaned'}):
            return result
        job = jobs.start('manual_drive_replay', dict(bag=src.bag, resource=src.resource),
                         [src.resource, 'record:' + src.bag], '手動走行記録の読込: ' + src.bag)
        return dict(result, job_id=job['id'])


def _vo_resource(src):
    return 'manual-vo:' + src.index.stem


@router.post('/vo')
def start_vo(request: Request, choice: Selection):
    store, jobs = request.app.state.store, request.app.state.jobs
    src = recording(store, choice.bag)
    resource = _vo_resource(src)
    with store.lock(resource):
        status = _status(src, jobs)
        if not status['ready']:
            raise ValueError('先に時系列を読み出してください')
        if status['vo_job_id'] and jobs.get(status['vo_job_id'])['status'] in ACTIVE | {'orphaned'}:
            return status
        if not status['can_generate_vo']:
            if status['vo_reason']:
                raise ValueError(status['vo_reason'])
            return status
        key = 'drive-vo-' + uuid4().hex
        job = jobs.start('manual_drive_vo', dict(bag=src.bag, resource=resource, map_id=key),
                         [resource, 'record:' + src.bag, 'map:' + key, 'ros-domain:93', 'offline-mapping'],
                         'ステレオから前後速度を推定: ' + src.bag)
        return dict(status, vo_job_id=job['id'])


def execute_vo(store, task, payload):
    src = recording(store, payload['bag'])
    if _vo_resource(src) != payload['resource'] or not src.index.is_file():
        raise ValueError('選択後に bag が変わりました。時系列を読み直してください')
    with _open(src.index) as db:
        start_ns, duration, stereo = db.execute('SELECT start_ns,duration_ns,stereo_ready FROM bag').fetchone()
    if not stereo:
        raise ValueError('左右 IR 画像・CameraInfo と静的 TF を含む bag を選択してください')

    def verify(source):
        if recording(store, src.bag).resource != src.resource:
            raise ValueError('姿勢の生成中に bag が変わりました。再度選択してください')
        if source.get('odom_poses', {}).get('frame') != 'base_link':
            raise ValueError('cuVSLAM の出力に base_link の連続オドメトリがありません')

    key = payload['map_id']
    task.log('Notebook の GPU で左右 IR 画像から連続オドメトリを生成しています')
    offline.map_build(store, task, dict(bag=src.bag, id=key), validate_source=verify)
    task.log('後輪軸中心の符号付き前後速度を計算しています')
    speed_source = 'odom:' + key
    manual_motion_sources.prepare(store, src.bag, speed_source, start_ns, duration)
    task.check()
    return dict(bag=src.bag, map=key, speed_source=speed_source)


def execute(store, task, payload):
    src = recording(store, payload['bag'])
    if src.resource != payload['resource']:
        raise ValueError('選択後に bag が変わりました。選び直してください')
    src.index.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.reading-', dir=src.index.parent) as working:
        path = Path(working) / 'timeline.sqlite'
        task.run(['ros2', 'run', 'atd_web_bridge', 'extract_drive_timeline', '--bag', str(src.directory),
                  '--output', str(path)])
        try:
            with _open(path) as db:
                _summary(db)
                db.execute('SELECT stereo_ready FROM bag LIMIT 0')
                db.execute('SELECT t,until_ns,speed_mps,segment FROM speed LIMIT 0')
                db.execute('SELECT t,until_ns,accel_mps2 FROM imu LIMIT 0')
                db.execute('SELECT t,until_ns,throttle_raw FROM propo LIMIT 0')
                db.execute('SELECT t,until_ns,steering,throttle FROM joy LIMIT 0')
                db.execute('SELECT t,until_ns,ch3,control_source,rx_mode_us,ch3_raw,servo_us,esc_us,source,pwm_enabled '
                           'FROM board LIMIT 0')
        except sqlite3.Error as error:
            raise ValueError('時系列の抽出形式を読み取れません。Notebook コンテナで '
                             'atd-build --packages-up-to atd_web_bridge を実行してください') from error
        if recording(store, src.bag).resource != src.resource:
            raise ValueError('読込中に bag が変更されました。再度選択してください')
        task.check()
        os.replace(path, src.index)
    return dict(bag=src.bag)


def _rows(db, table, fields, start, stop):
    # 区間は重ならない。開始時点をまたぐ直前の1行も取得する。
    sql = (f'SELECT t,until_ns,{fields} FROM {table} WHERE '
           f't >= coalesce((SELECT max(t) FROM {table} WHERE t<=?),0) AND t<? AND until_ns>? ORDER BY t')
    result = []
    for row in db.execute(sql, (start, stop, start)):
        item = dict(row)
        item['start_s'] = max(item.pop('t'), start) / NS
        item['end_s'] = min(item.pop('until_ns'), stop) / NS
        if table == 'board' and item['pwm_enabled'] is not None:
            item['pwm_enabled'] = bool(item['pwm_enabled'])
        result.append(item)
    return result


@router.get('/window')
def window(request: Request, bag: str, start_s: float = Query(ge=0, allow_inf_nan=False),
           end_s: float = Query(ge=0, allow_inf_nan=False), speed_source: str | None = None):
    if end_s < start_s or end_s - start_s > MAX_WINDOW_S + 1e-9:
        raise ValueError(f'表示区間は {MAX_WINDOW_S} 秒以内で指定してください')
    src = recording(request.app.state.store, bag)
    if not src.index.is_file():
        raise ValueError('この bag の手動走行記録を先に読み込んでください')
    with _open(src.index) as db:
        bag_start, duration = db.execute('SELECT start_ns,duration_ns FROM bag').fetchone()
        if end_s > duration / NS + 0.5 / NS:
            raise ValueError('表示区間が bag の記録範囲外です')
        start, end = round(start_s * NS), round(end_s * NS)
        if start == end and end != duration:
            raise ValueError('表示区間の終了は開始より後に指定してください')
        stop = end + int(end == duration)
        db.row_factory = sqlite3.Row
        joy = _rows(db, 'joy', 'steering,throttle', start, stop)
        board = _rows(db, 'board', 'ch3,control_source,rx_mode_us,ch3_raw,servo_us,esc_us,source,pwm_enabled',
                      start, stop)
        imu = _rows(db, 'imu', 'accel_mps2', start, stop)
        propo = _rows(db, 'propo', 'throttle_raw', start, stop)
        speed = _rows(db, 'speed', 'speed_mps,segment', start, stop) if speed_source == 'recorded' else []
    speed_issue = ''
    if speed_source and speed_source != 'recorded':
        try:
            speed = manual_motion_sources.window(request.app.state.store, src.bag, speed_source, bag_start, start, stop)
        except (OSError, ValueError) as error:
            speed_issue = f'選択した速度を読み取れません。{error}'
    return dict(bag=src.bag, start_s=start / NS, end_s=end / NS, duration_s=duration / NS, joy=joy, board=board,
                imu=imu, propo=propo, speed=speed, speed_source=speed_source, speed_issue=speed_issue)


CONFIRM_NS = 150_000_000


def _transitions(rows, threshold):
    """連続した速度推定で確認できた方向変化。欠測・追跡世代をまたがない。"""
    changes = []
    last_end = last_segment = candidate = confirmed = None
    candidate_start = confirmed_start = 0
    for start, end, speed, segment in rows:
        if segment != last_segment or start != last_end:
            candidate = confirmed = None
        last_end, last_segment = end, segment
        state = 'forward' if speed > threshold else 'reverse' if speed < -threshold else 'near_zero'
        if candidate != state:
            candidate, candidate_start = state, start
        if end - candidate_start >= CONFIRM_NS and confirmed != state:
            if confirmed is not None:
                changes.append({'time_s': candidate_start / NS, 'from': confirmed, 'to': state,
                                'near_zero_start_s': confirmed_start / NS if confirmed == 'near_zero' else None})
            confirmed, confirmed_start = state, candidate_start
    return changes


@router.get('/transitions')
def transitions(request: Request, bag: str, speed_source: str,
                threshold_mps: float = Query(default=.05, gt=0, allow_inf_nan=False)):
    store = request.app.state.store
    src = recording(store, bag)
    if not src.index.is_file():
        raise ValueError('この bag の時系列を先に読み出してください')
    issue, changes = '', []
    with _open(src.index) as db:
        if speed_source == 'recorded':
            rows = db.execute('SELECT t,until_ns,speed_mps,segment FROM speed ORDER BY t')
            changes = _transitions(rows, threshold_mps)
        else:
            start, duration = db.execute('SELECT start_ns,duration_ns FROM bag').fetchone()
            try:
                rows = manual_motion_sources.spans(store, src.bag, speed_source, start, duration)
                changes = _transitions(rows, threshold_mps)
            except (OSError, ValueError) as error:
                issue = f'選択した速度を読み取れません。{error}'
    return dict(bag=src.bag, speed_source=speed_source, threshold_mps=threshold_mps,
                confirm_s=CONFIRM_NS / NS, transitions=changes, issue=issue)
