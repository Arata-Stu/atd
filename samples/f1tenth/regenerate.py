#!/usr/bin/env python3
"""元の CSV から RC 用 HDMap を作り、ATD の既定 region・mincurv で生成する（Notebook Web 環境）。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

import numpy as np
from shapely import hausdorff_distance
from shapely.geometry import LinearRing, LineString

SAMPLE = Path(__file__).resolve().parent
sys.path.insert(0, str(SAMPLE.parents[1] / 'apps/web/processing'))
from atd_processing.geometry import corridor, generate


# A modest increase in layout size lets these hairpins pass the standard ATD
# interpolation/curvature checks while retaining the 0.76 m road width.
EXTRA_SCALE = dict(austin=1.075, budapest=1.075, catalunya=1.075, hockenheim=1.075,
                   montreal=1.075, shanghai=1.04)
ROAD_WIDTH_M = .76


def catalog():
    return json.loads((SAMPLE / 'catalog.json').read_text())


def input_file(track):
    source = SAMPLE / 'sources' / (track['id'] + '.csv')
    records = (SAMPLE / 'sources/SHA256SUMS').read_text().splitlines()
    digests = {}
    for record in records:
        checksum, filename = record.split('  ', 1)
        digests[filename] = checksum
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digests.get(source.name) != digest:
        raise ValueError(f'{source.name} の SHA-256 が一致しません')
    return source, digest


def road_boundaries(center):
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    norms = np.linalg.norm(tangent, axis=1)
    if np.min(norms) < 1e-6:
        raise ValueError('設計中心に長さゼロの接線があります')
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0])) / norms[:, None]

    def simplify(wall):
        closed = LineString(np.vstack((wall, wall[0])))
        return [list(xy) for xy in closed.simplify(.006).coords[:-1]]

    half_width = ROAD_WIDTH_M / 2
    return simplify(center + half_width * normal), simplify(center - half_width * normal)


def document(track, upstream, source_path, digest, fit):
    extra = EXTRA_SCALE.get(track['id'], 1.)
    scale = float(fit['scale']) * extra
    center = fit['dense'] * extra
    reference = fit['orig'] * extra
    left, right = road_boundaries(center)
    raw = np.loadtxt(source_path, delimiter=',')[:, :2]
    paired_offset = np.linalg.norm(center[::4] - reference, axis=1)
    worst = int(np.argmax(paired_offset))
    source = dict(kind='f1tenth_benchmark', repository=upstream['url'], commit=upstream['revision'],
                  file=f"{track['folder']}/{track['folder'].replace(' ', '')}_centerline.csv", sha256=digest,
                  upstream_lap_m=LinearRing(raw).length, xy_scale_from_upstream=scale,
                  width_m=ROAD_WIDTH_M, curve_fit='compact_rc_v1',
                  reference_hausdorff_m=float(hausdorff_distance(LinearRing(raw*scale), LinearRing(center))),
                  corresponding_rms_offset_m=float(np.sqrt(np.mean(paired_offset**2))),
                  max_corresponding_offset_m=float(paired_offset[worst]),
                  max_offset_source_xy_m=reference[worst].tolist(), max_offset_rc_xy_m=center[4*worst].tolist())
    doc = dict(id=track['id'], title=track['name'] + ' (RC)', closed=True, frame='map',
               left=left, right=right, custom=[], lines={}, revision=2, source=source, updated=0)
    corridor(doc)
    return doc


def write_bundle(folder, doc):
    # A complete folder is published in one rename. Never modify existing edited maps.
    folder.parent.mkdir(parents=True, exist_ok=True)
    if folder.exists():
        raise FileExistsError(f'出力先が既に存在します: {folder}')
    with TemporaryDirectory(prefix='.f1tenth-', dir=folder.parent) as tmp:
        staged = Path(tmp) / doc['id']
        files = {'hdmap/map.json': doc}
        for revision, kind in enumerate(('centerline', 'raceline'), start=3):
            files[f'lines/{kind}.json'] = dict(map_id=doc['id'], revision=revision, **doc['lines'][kind])
        for name, content in files.items():
            path = staged / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(content, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
        staged.rename(folder)


def regenerate(track, upstream, root, helper):
    folder = root / track['id']
    if folder.exists():
        raise FileExistsError(f'出力先が既に存在します: {folder}')
    source, digest = input_file(track)
    with TemporaryDirectory(prefix='atd-f1-outline-') as tmp:
        shape = Path(tmp) / 'shape.npz'
        command = [helper, str(SAMPLE / 'prepare_outline.py'), str(source), str(shape)]
        if track['id'] == 'shanghai':
            command.append('--source-seed')
        subprocess.run(command, check=True)
        with np.load(shape) as fit:
            doc = document(track, upstream, source, digest, fit)
    for kind in ('centerline', 'raceline'):
        doc['lines'][kind] = generate(doc, kind, {})
        doc['revision'] += 1
    write_bundle(folder, doc)
    print(f"{track['id']}  Centerline {doc['lines']['centerline']['length']:.2f} m  "
          f"Raceline {doc['lines']['raceline']['length']:.2f} m", flush=True)


if __name__ == '__main__':
    data = catalog()
    tracks = {t['id']: t for t in data['tracks'] if t['included']}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='生成する地図の親ディレクトリ')
    parser.add_argument('--course', choices=tracks, action='append', help='コースID（複数可、省略時は全件）')
    args = parser.parse_args()
    helper = os.environ.get('ATD_RACELINE_PYTHON')
    if not helper or not Path(helper).is_file():
        parser.error('ATD_RACELINE_PYTHON に準備済み raceline 環境の Python を指定してください')
    for name in args.course or tracks:
        regenerate(tracks[name], data['upstream'], args.output, helper)
