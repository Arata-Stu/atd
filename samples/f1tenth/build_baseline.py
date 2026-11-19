#!/usr/bin/env python3
"""同梱 HDMap を UI と同じ方法で評価し、全コースが終わってから JSON を公開する。"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'apps/web/backend/src'), str(ROOT / 'apps/web/processing')]

from atd_api.simulation_benchmark import Batch, evaluate_row, included, make_report, prepare
from atd_api.store import atomic_json


def evaluate(key, payload):
    # 配布用基準は同梱サンプルを読む。編集用の地図や走行 bag は参照しない。
    return evaluate_row(None, key, payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='結果 JSON の保存先')
    parser.add_argument('--workers', type=int, default=2, help='並列評価プロセス数（既定 2）')
    args = parser.parse_args()
    if args.workers < 1:
        parser.error('--workers は 1 以上を指定してください')
    keys = list(included())
    selection = Batch(course_ids=keys)
    options = prepare(None, selection)
    version = make_report('baseline', options, [])['implementation_sha256']
    rows = []
    with ProcessPoolExecutor(max_workers=min(args.workers, len(keys))) as pool:
        for row in pool.map(evaluate, keys, repeat(options)):
            rows.append(row)
            print(f"{len(rows)}/{len(keys)} {row['course_id']}", flush=True)
    if prepare(None, selection) != options:
        raise RuntimeError('評価中にコースが更新されました。再実行してください')
    report = make_report('baseline', options, rows)
    if report['implementation_sha256'] != version:
        raise RuntimeError('評価中に実装が変更されました。再実行してください')
    atomic_json(args.output, report)
    print(args.output)


if __name__ == '__main__':
    main()
