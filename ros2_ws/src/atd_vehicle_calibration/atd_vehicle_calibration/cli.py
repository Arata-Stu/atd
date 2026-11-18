"""Notebook で校正用 bag とローカル VO 軌跡を解析する。"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from math import isfinite
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from uuid import uuid4


def source_info(bag, poses, pose_frame):
    if poses.name in {'slam_poses.tum', 'keyframe_pose.tum', 'keyframe_pose_optimized.tum', 'fully_optimized_poses.tum'}:
        raise ValueError('速度には連続した odom_poses.tum を指定してください')
    with (bag / 'metadata.yaml').open('rb') as stream:
        bag_digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    with poses.open('rb') as stream:
        pose_digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    source = {'bag': str(bag), 'metadata_sha256': bag_digest, 'poses': str(poses),
              'poses_sha256': pose_digest, 'pose_frame': pose_frame, 'body_frame': 'base_link',
              'mapping_manifest': None, 'association': 'user_supplied'}
    for parent in poses.parents:
        manifest = parent.parent / 'mapping.json'
        if parent.name != 'nvidia' or not manifest.is_file():
            continue
        document = json.loads(manifest.read_text())
        if not isinstance(document, dict) or not isinstance(document.get('source'), dict):
            raise ValueError(f'姿勢の生成元を読めません: {manifest}')
        if document['source'].get('metadata_sha256') != bag_digest:
            raise ValueError(f'姿勢の生成元と指定bagのmetadataが一致しません: {manifest}')
        source.update(mapping_manifest=str(manifest), association='mapping_metadata_sha256')
        break
    return source


def analyze_run(args):
    from .analysis import analyze
    from .inputs import read_bag
    from .motion import estimate_motion, load_tum
    from .report import plot_response

    bag, poses = args.bag.resolve(), args.poses.resolve()
    if args.output is None:
        run = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid4().hex[:8]
        output = Path('/data/results/runs/vehicle_calibration') / run
    else:
        output = args.output.resolve()
    if output.exists() or output.is_symlink():
        raise ValueError(f'出力先は新規ディレクトリを指定してください: {output}')
    if output.is_relative_to(bag):
        raise ValueError('解析の保存先は入力bagの外に指定してください')
    source = source_info(bag, poses, args.pose_frame)
    trajectory = load_tum(poses, world_frame='odom', pose_frame=args.pose_frame)
    reference = None
    if args.reference_summary:
        reference_path = args.reference_summary.resolve()
        saved = json.loads(reference_path.read_text())
        if (not isinstance(saved, dict) or saved.get('schema_version') != 1
                or not isinstance(saved.get('source'), dict) or not isinstance(saved.get('analysis'), dict)
                or not saved['source'].get('metadata_sha256')):
            raise ValueError('比較元に校正解析の summary.json を指定してください')
        if saved['source']['metadata_sha256'] == source['metadata_sha256']:
            raise ValueError('検証には応答表の作成に使ったbagと異なる走行bagを指定してください')
        reference = saved['analysis']
        source['reference_summary'] = str(reference_path)
    data = read_bag(bag, max_gap_s=args.max_gap_s)
    motion = estimate_motion(trajectory, pose_frame=args.pose_frame,
                             static_transforms=data['static_transforms'], world_frame='odom',
                             window_s=args.window_s, max_gap_s=args.max_gap_s)
    if not motion['samples']:
        raise ValueError('有効な姿勢区間から速度を推定できません。時刻、TF、推定窓幅を確認してください')
    rows, result = analyze(data, motion, wheelbase_m=args.wheelbase_m, max_sync_s=args.max_sync_s,
                           max_gap_s=args.max_gap_s, min_speed_mps=args.min_speed_mps,
                           gravity_world_mps2=args.gravity_world_mps2, reference_summary=reference)
    if not rows:
        raise ValueError('解析できる時系列がありません。姿勢とbagの時刻系を確認してください')
    eligible = sum(row['valid'] for row in rows)
    summary = {'schema_version': 1, 'source': source, 'eligible_samples': eligible,
               'timeseries_samples': len(rows), 'inputs': data['provenance'], 'motion': motion['provenance'],
               'input_rejections': data['rejected'], 'motion_rejections': motion['rejected'],
               'analysis': result}
    # Finish rendering before reserving the destination. mkdir prevents two
    # concurrent runs from replacing each other. Publish the summary last.
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.analysis-', dir=output.parent) as temporary:
        stage = Path(temporary)
        with (stage / 'timeseries.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        plot_response(rows, stage / 'response.png', bag.name)
        with (stage / 'summary.json').open('w', encoding='utf-8') as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        output.mkdir()
        for name in ('timeseries.csv', 'response.png', 'summary.json'):
            (stage / name).rename(output / name)
    print(f'{output}\n解析 {len(rows)} 点、プロポ入力と対応した有効試料 {eligible} 点')
    if not eligible:
        print('有効なプロポ手動区間がありません。除外理由を summary.json で確認してください。', file=sys.stderr)
    return 0 if eligible else 2


def positive(text):
    value = float(text)
    if not isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('正の有限値を指定してください')
    return value


def check_bag(argv=None):
    """Validate cheap inputs before the Web worker starts offline VO."""
    from .inputs import inspect_metadata
    import yaml
    parser = argparse.ArgumentParser(description='校正に必要な bag の記録トピック・件数・データファイルを確認')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--require-stereo', action='store_true', help='新規 VO の左右 Infra 画像も検査')
    args = parser.parse_args(argv)
    try:
        bag = args.bag.resolve(strict=True)
        metadata, _ = inspect_metadata(bag, require_stereo=args.require_stereo)
        paths = metadata.get('relative_file_paths')
        if not isinstance(paths, list) or not paths:
            raise ValueError('bag の relative_file_paths が空または不正です')
        files = metadata.get('files', [])
        if not isinstance(files, list) or any(not isinstance(file, dict) for file in files):
            raise ValueError('bag の files が不正です')
        for name in [*paths, *(file.get('path') for file in files)]:
            if not isinstance(name, str) or not name or Path(name).is_absolute() or '..' in Path(name).parts:
                raise ValueError('bag データファイルの相対パスが不正です')
            path = bag / name
            if path.resolve() != path or not path.is_file() or not path.stat().st_size:
                raise ValueError('bag 内に非空の通常ファイルがありません: ' + name)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'校正 bag の事前検査エラー: {error}', file=sys.stderr, flush=True)
        return 3
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True, help='metadata.yaml を含む手動走行の bag')
    parser.add_argument('--poses', type=Path, required=True, help='同じ走行から生成した odom_poses.tum')
    parser.add_argument('--pose-frame', required=True, help='軌跡が表す剛体の ROS frame（導入版・実行設定で確認）')
    parser.add_argument('--wheelbase-m', type=positive, help='実測ホイールベース [m]。指定時に等価舵角を推定')
    parser.add_argument('--gravity-world-mps2', type=float, nargs=3, metavar=('GX', 'GY', 'GZ'),
                        help='軌跡の世界座標で確認した重力 [m/s²]。指定時にIMUの重力補正値を追加')
    parser.add_argument('--window-s', type=positive, default=0.3, help='局所微分の窓幅 [s]')
    parser.add_argument('--max-gap-s', type=positive, default=0.2, help='連続区間を分ける時間差 [s]')
    parser.add_argument('--max-sync-s', type=positive, default=0.05, help='入力を対応付ける時刻差の上限 [s]')
    parser.add_argument('--min-speed-mps', type=positive, default=0.15, help='曲率の計算に使う最低速度 [m/s]')
    parser.add_argument('--output', type=Path, help='新規の保存先。既定は /data/results/runs/vehicle_calibration/<run>')
    parser.add_argument('--reference-summary', type=Path, help='別の走行で作った応答表（summary.json）で今回の走行を評価')
    args = parser.parse_args(argv)
    if args.gravity_world_mps2 is not None and not all(isfinite(x) for x in args.gravity_world_mps2):
        parser.error('重力ベクトルには有限値を指定してください')
    try:
        return analyze_run(args)
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as exc:
        print(f'校正解析エラー: {exc}', file=sys.stderr)
        return 1
