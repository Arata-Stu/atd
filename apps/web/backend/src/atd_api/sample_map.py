"""同梱したお台場の HDMap と、Notebook への編集用コピー。"""
from pathlib import Path
import tempfile
import time

from .map_library import destination
from .store import atomic_json, read_json


# 保存地図の ID に使えない文字を使い、同じ名前のユーザー地図と区別する。
ODAIBA_COURSE = 'sample:odaiba'
ODAIBA_MAP = Path(__file__).resolve().parents[5] / 'samples/odaiba/hdmap/map.json'


def odaiba_document():
    return read_json(ODAIBA_MAP)


def import_odaiba(store, key, title):
    destination(store, key)
    doc = odaiba_document()
    doc.update(id=key, title=title or key, revision=1, updated=time.time())
    with tempfile.TemporaryDirectory(dir=store.maps_root, prefix='.sample-') as tmp:
        stage = Path(tmp) / key
        atomic_json(stage / 'hdmap/map.json', doc)
        for kind in ('centerline', 'raceline'):
            atomic_json(stage / 'lines' / (kind + '.json'), dict(doc['lines'][kind], map_id=key, revision=1))
        with store.lock('maps'):
            target = destination(store, key)
            stage.rename(target)
    return doc
