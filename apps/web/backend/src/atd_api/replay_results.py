"""atd replay が保存した Gazebo 正解軌跡と画像時刻ごとの推定誤差を読む。"""
import fcntl
from itertools import islice
import json
import math
from pathlib import Path

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse

from .store import confined, identifier, read_json


router = APIRouter(prefix='/replay')
RUNS = Path('runs/replay')
# Monza の 1/10 GP 走行線。距離は計時線から走行方向に測る。
MONZA_SECTIONS = (
    dict(id='post_ascari', label='アスカリ後の直線', start_m=390, end_m=480),
    dict(id='home', label='ホームストレート', start_m=524, end_m=60),
)


def _finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def _summary(folder):
    path = confined(folder, 'summary.json')
    value = read_json(path)
    if isinstance(value, dict) and value.get('status') == 'running' and value.get('writer_lock') == 'samples.jsonl':
        # プロセスが kill されても確定済みのチェックポイントを保持する。
        # 通常終了と重なった場合は、排他ロック解放前に公開された最終状態を読む。
        with confined(folder, 'samples.jsonl').open('rb') as samples:
            try:
                fcntl.flock(samples, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError:
                pass
            else:
                value = read_json(path)
                if isinstance(value, dict) and value.get('status') == 'running':
                    value.update(status='partial', reason='保存処理が終了したため、最後のチェックポイントまで表示します')
    if (not isinstance(value, dict) or value.get('version') != 1 or value.get('id') != folder.name
            or value.get('status') not in ('running', 'complete', 'partial', 'failed')
            or not isinstance(value.get('bag'), str) or not isinstance(value.get('created_at'), str)
            or type(value.get('sample_count')) is not int or value['sample_count'] < 0
            or not _finite(value.get('rate')) or value['rate'] <= 0):
        raise ValueError(f'{folder.name} の replay 集計ファイルが不正です')
    laps, length = value.get('laps'), value.get('lap_length_m')
    if ((laps is not None and (type(laps) is not int or laps < 1))
            or (length is not None and (not _finite(length) or length <= 0))
            or (value.get('circuit') is not None and not isinstance(value['circuit'], str))
            or (value.get('reason') is not None and not isinstance(value['reason'], str))):
        raise ValueError(f'{folder.name} の replay 周回情報または状態理由が不正です')
    keys = ('version', 'id', 'bag', 'created_at', 'status', 'sample_count', 'rate', 'circuit', 'laps',
            'lap_length_m', 'reason')
    summary = {key: value.get(key) for key in keys}
    if 'tracking' in value:
        if value['tracking'] not in ('vo', 'vio'):
            raise ValueError('replay の追跡方式が不正です')
        summary['tracking'] = value['tracking']
        if value['tracking'] == 'vio':
            if not _finite(value.get('imu_hz')) or value['imu_hz'] <= 0:
                raise ValueError('VIO の IMU 周波数が不正です')
            summary['imu_hz'] = value['imu_hz']
    if 'slam_max_map_size' in value:
        limit = value['slam_max_map_size']
        if type(limit) is not int or limit < 0:
            raise ValueError('replay の地図姿勢数の上限が不正です')
        summary['slam_max_map_size'] = limit
    if 'gap_threshold_s' in value:
        if not _finite(value['gap_threshold_s']) or value['gap_threshold_s'] <= 0:
            raise ValueError('replay のサンプル間隔しきい値が不正です')
        summary['gap_threshold_s'] = value['gap_threshold_s']
    if 'anchor_stamp_ns' in value:
        stamp = value['anchor_stamp_ns']
        if stamp is not None and (not isinstance(stamp, str) or not stamp.isascii() or not stamp.isdecimal()):
            raise ValueError('replay の初期合わせの時刻が不正です')
        summary['anchor_stamp_ns'] = stamp
    return summary


def list_runs(store):
    root = confined(store.result_root, RUNS, exists=False)
    runs = []
    if root.is_dir():
        for folder in root.iterdir():
            if folder.is_symlink() or not folder.is_dir() or folder.name.startswith('.'):
                continue
            try:
                identifier(folder.name)
                runs.append(_summary(folder))
            except (ValueError, OSError):
                continue
    return sorted(runs, key=lambda run: (run['created_at'], run['id']), reverse=True)


def _point(row, *, estimate=False):
    fields = ('distance_m', 'time_s', 'gt_x_m', 'gt_y_m')
    if estimate:
        fields += ('est_x_m', 'est_y_m', 'along_m', 'lateral_m', 'heading_deg')
    if not isinstance(row, dict) or not all(_finite(row.get(k)) for k in fields):
        raise ValueError('replay の位置・時刻・誤差に有限の数値が必要です')
    lap, station = row['lap'], row['station_m']
    if ((lap is not None and (type(lap) is not int or lap < 1))
            or (station is not None and not _finite(station))):
        raise ValueError('replay の周回番号または周回内距離が不正です')
    point = {key: row[key] for key in fields}
    point.update(lap=lap, station_m=row['distance_m'] if station is None else station)
    if estimate:
        stamp, segment = row.get('stamp_ns'), row.get('segment')
        if (not isinstance(stamp, str) or not stamp.isascii() or not stamp.isdecimal()
                or type(segment) is not int or segment < 0):
            raise ValueError('replay の推定時刻または連続区間番号が不正です')
        point.update(stamp_ns=stamp, segment=segment)
    else:
        point['x_m'] = point.pop('gt_x_m')
        point['y_m'] = point.pop('gt_y_m')
    return point


def load_run(store, run_id):
    folder = confined(store.result_root, RUNS / identifier(run_id), exists=False)
    if not folder.is_dir():
        raise HTTPException(status_code=404, detail='指定された replay の記録が見つかりません')
    summary = _summary(folder)
    reference = read_json(confined(folder, 'reference.json'))
    if not isinstance(reference, list):
        raise ValueError('replay の正解軌跡が点列ではありません')
    try:
        reference = [_point(row) for row in reference]
        # sample_count は書き手が flush 後に atomic rename した時点の行数。
        # 取得中の追記や、書きかけの最終行を応答に混ぜない。
        with confined(folder, 'samples.jsonl').open('rb') as stream:
            samples = [_point(json.loads(line), estimate=True) for line in islice(stream, summary['sample_count'])]
    except KeyError as error:
        raise ValueError(f'replay の必須列 {error} が欠けています') from error
    if len(samples) != summary['sample_count']:
        raise ValueError('replay の保存済みサンプル数より推定データが短くなっています')
    length = summary['lap_length_m']
    is_monza_gp = (summary['circuit'] == 'monza' and summary['laps'] is not None and length is not None
                   and math.isclose(length, 579.20, abs_tol=0.1))
    return dict(**summary, reference=reference, samples=samples, sections=MONZA_SECTIONS if is_monza_gp else [])


@router.get('/runs')
def runs(request: Request):
    return dict(items=list_runs(request.app.state.store))


@router.get('/runs/{run_id}')
def run(request: Request, run_id: str):
    return JSONResponse(load_run(request.app.state.store, run_id))
