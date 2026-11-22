#!/usr/bin/env python3
"""Gazebo お台場の舗装境界から既定の HDMap と走行ラインを生成する。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/gazebo_monza'))
sys.path.insert(0, str(ROOT / 'apps/web/processing'))

from circuits import load_lap, reference_files, track_data
from atd_processing.geometry import corridor, generate


def boundaries():
    edges, _, _ = load_lap('odaiba')
    track, _ = reference_files('odaiba')
    source = track_data('odaiba')
    document = dict(id='odaiba', title='お台場・シティサーキット東京ベイ（1/5）',
                    closed=True, frame='map', left=edges['left'], right=edges['right'],
                    custom=[], lines={}, revision=1, updated=0,
                    source=dict(kind='gazebo_sample', circuit='odaiba',
                                file=track.relative_to(ROOT).as_posix(),
                                sha256=hashlib.sha256(track.read_bytes()).hexdigest(),
                                horizontal_scale=source['horizontal_scale'], axes=source['axes'],
                                official_reference=source['official_reference'],
                                attribution=source['attribution']))
    corridor(document)
    return document


def write_json(path, data):
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(data, separators=(',', ':'), ensure_ascii=False, allow_nan=False) + '\n',
                    encoding='utf-8')


def regenerate(output):
    if output.exists():
        raise ValueError(f'出力先は未作成のディレクトリを指定してください: {output}')
    document = boundaries()
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.odaiba-', dir=output.parent) as tmp:
        bundle = Path(tmp) / 'odaiba'
        bundle.mkdir()
        for kind in ('centerline', 'raceline'):
            line = generate(document, kind, {})
            document['lines'][kind] = line
            document['revision'] += 1
            write_json(bundle / 'lines' / f'{kind}.json',
                       dict(map_id='odaiba', revision=document['revision'], **line))
            print(f'{kind} {line["length"]:.3f} m ({len(line["points"])} 点)', flush=True)
        write_json(bundle / 'hdmap/map.json', document)
        bundle.rename(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='新しく生成する地図ディレクトリ')
    regenerate(parser.parse_args().output)
