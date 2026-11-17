"""Use the lane's published Centerline as the Raceline starting geometry."""

import math

from .geometry import Corridor, PathValidationError, edges, points, resample
import hashlib
import json

def fingerprint(data):
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate(line, corridor, clearance, info, stage="raceline_reference"):
    try:
        corridor.validate(line, clearance)
    except PathValidationError as exc:
        label = "保存済みCenterline" if info["source"] == "saved_centerline" else "領域中心線"
        exc.diagnostic.update(stage=stage, reference_source=info["source"])
        phase = "平滑化後の初期経路" if stage == "raceline_smoothed_reference" else "初期経路"
        exc.args = (f"Raceline{phase}（{label}）の検査: {exc}",)
        raise


def build(document, spacing):
    closed = document["closed"]
    saved = document.get("lines", {}).get("centerline")
    if saved is None:
        raise ValueError('先に領域方式の Centerline を生成・保存してください。境界中点への fallback は行いません')
    if saved.get('method') != 'region_equal_distance_v1':
        raise ValueError('領域方式で Centerline を再生成・保存してください。旧方式の中心線を最適化の初期経路には使いません')
    else:
        minimum = 3 if closed else 2
        line = points(saved["points"], minimum)
        corridor = Corridor(points(document["left"], minimum), points(document["right"], minimum), closed)
        info = dict(source="saved_centerline", method=saved.get("method", "unknown"),
                    source_points_hash=fingerprint(line))
        # Honor the requested optimization resolution, even if the saved Center
        # was generated at another spacing. Never edit its stored points.
        length = sum(math.dist(a, b) for a, b in edges(line, closed))
        count = max(12 if closed else 3, min(600, math.ceil(length / spacing)))
        line = resample(line, count, closed)
    validate(line, corridor, 0, info)
    return line, corridor, info
