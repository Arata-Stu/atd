"""同一 revision の HDMap JSON/YAML とライン CSV を明示的に書き出す。"""
import csv
import os
import shutil
import tempfile

from .store import atomic_json


def export_map(store, key):
    with store.lock('map-' + key):
        document = store.load_map(key)
        folder = store.folder(key)
        root = folder / 'exports'
        root.mkdir(exist_ok=True)
        target = root / ('r' + str(document['revision']))
        if target.exists():
            return dict(map=key, revision=document['revision'], relative=str(target.relative_to(store.result_root)))
        stage = tempfile.mkdtemp(prefix='.export-', dir=root)
        from pathlib import Path
        stage = Path(stage)
        try:
            # JSON は YAML 1.2 のサブセット。追加 YAML 依存を要求しない。
            atomic_json(stage / 'hdmap.yaml', document)
            for kind, line in document['lines'].items():
                with (stage / (kind + '.csv')).open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['x_m', 'y_m', 'speed_mps'])
                    writer.writerows([*point, speed] for point, speed in zip(line['points'], line['speeds']))
            atomic_json(stage / 'manifest.json', dict(schema='atd.hdmap.export.v1', map_id=key,
                revision=document['revision'], frame=document['frame'], line_kinds=sorted(document['lines'])))
            os.rename(stage, target)
        finally:
            if stage.exists():
                shutil.rmtree(stage)
        return dict(map=key, revision=document['revision'], relative=str(target.relative_to(store.result_root)))
