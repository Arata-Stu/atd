"""保存済み ORB 固定地図 run の姿勢途絶を、収録ジャイロでオフライン評価する。"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile

from .orb_gap_imu import read_bag_gyro
from .orb_gap_motion import evaluate_gaps


def run_dir(results_root, kind, run_id):
    if not isinstance(run_id, str) or not run_id or Path(run_id).name != run_id or run_id == '..':
        raise ValueError(f'{kind} の run ID が不正です: {run_id!r}')
    return results_root / 'runs' / kind / run_id


def evaluate_run(results_root, run_id, bag_override=None):
    folder = run_dir(results_root, 'orbslam3', run_id)
    orb = json.loads((folder / 'summary.json').read_bytes())
    if orb.get('mode') != 'localization' or orb.get('backend') != 'orbslam3' or not orb.get('gt_replay_id'):
        raise ValueError('Gazebo 正解軌跡と照合した ORB-SLAM3 固定地図 run を指定してください')
    replay = run_dir(results_root, 'replay', orb['gt_replay_id'])
    gt = json.loads((replay / 'summary.json').read_bytes())
    if (gt.get('id') != replay.name or gt.get('alignment_source') != 'saved_map'
            or any(gt.get(key) != orb[key] for key in ('bag', 'backend', 'mode', 'map_id', 'rate', 'start_offset_s'))):
        raise ValueError('ORB run と Gazebo 比較 run の地図、bag、再生条件が一致しません')
    if 'running' in (orb.get('status'), gt.get('status')):
        raise ValueError('記録中の run は終了後に評価してください')
    bag = bag_override or Path(orb['bag'])
    if hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest() != orb['metadata_sha256']:
        raise ValueError('bag の metadata.yaml が ORB run の収録元と一致しません')

    with (folder / 'trajectory.csv').open(newline='') as stream:
        stamps = [int(row['image_stamp_ns']) for row in csv.DictReader(stream)]
    epochs = {}
    with (folder / 'status.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row['state'] == 'tracking':
                stamp, epoch = int(row['stamp_ns']), int(row['epoch'])
                # 同じ撮影時刻が別 epoch に属す場合、その姿勢から速度を推定しない。
                if epochs.setdefault(stamp, epoch) != epoch:
                    epochs[stamp] = None
    samples = {}
    with (replay / 'samples.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            stamp = int(row['stamp_ns'])
            if stamp in samples:
                raise ValueError(f'Gazebo 比較に撮影時刻の重複があります: {stamp}')
            samples[stamp] = row

    gyro_stamps, rates, imu = read_bag_gyro(bag)
    result = evaluate_gaps(stamps, epochs, samples, gyro_stamps, rates)
    result.update(schema='atd.orbslam3.imu_gap_report.v1', orb_run=run_id, orb_status=orb['status'],
                  gt_replay_id=replay.name, gt_status=gt['status'], map_id=orb['map_id'], bag=str(bag),
                  metadata_sha256=orb['metadata_sha256'], imu=imu, trajectory_rows=len(stamps),
                  trajectory_samples_reported=orb.get('trajectory_samples'))
    path = folder / 'imu_gap_report.json'
    payload = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
    with NamedTemporaryFile(dir=folder, mode='w', encoding='utf-8', delete=False) as stream:
        temp = Path(stream.name)
        try:
            stream.write(payload)
            stream.flush()
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
    return path, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_id', help='runs/orbslam3 内の固定地図 run ID')
    parser.add_argument('--results-root', type=Path, default=Path(os.environ.get('ATD_REPLAY_RESULTS_ROOT', '/data/results')))
    parser.add_argument('--bag', type=Path, help='同じ bag を移動した場合のディレクトリ')
    args = parser.parse_args(argv)
    try:
        path, result = evaluate_run(args.results_root, args.run_id, args.bag)
    except (OSError, KeyError, ValueError, RuntimeError, ImportError, csv.Error) as error:
        print(f'ジャイロによる途絶評価に失敗しました: {error}', file=sys.stderr)
        return 1

    reported = result['trajectory_samples_reported']
    if reported is not None and result['trajectory_rows'] != reported:
        print(f"記録済み CSV の {result['trajectory_rows']} 姿勢を評価します（run 集計 {reported} 姿勢）。")
    print(f"0.25 秒を超える姿勢出力の間隔 {result['gap_count']} 区間、4方式で比較できたのは {result['paired_count']} 区間")
    if result['paired_count']:
        for method, label in (('hold', '位置保持'), ('straight', '等速直進'), ('turn', '定速旋回'), ('gyro', '収録ジャイロ')):
            metrics = result['paired_metrics'][method]
            print(f"  {label}　終点位置誤差の中央値 {metrics['median_position_error_m']:.3f} m")
    longest = result['longest_gap']
    if longest:
        print(f"最長の途絶 {longest['duration_s']:.3f} 秒", end='　')
        if longest['status'] == 'evaluated':
            print(f"ジャイロ使用時の終点位置誤差 {longest['errors']['gyro']['position_error_m']:.3f} m")
        else:
            print(f"比較対象外 ({longest['status']})")
    print(f'評価結果: {path}')
    return 0
