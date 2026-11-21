"""同じ Gazebo 周回記録の計時線、周回長と走行開始時刻を復元する。"""
import json
import math
from pathlib import Path

from .replay_gt_source import SIDECAR, TRACE, matching_run


def _validated(course):
    unknown = dict(circuit=None, laps=None, lap_length_m=None, lap_duration_s=None, start_ns=None)
    if not isinstance(course, dict):
        return unknown
    circuit, laps, start = (course.get(k) for k in ('circuit', 'laps', 'start_ns'))
    length, duration = (course.get(k) for k in ('lap_length_m', 'lap_duration_s'))
    if (not isinstance(circuit, str) or not circuit or type(laps) is not int or laps < 1
            or type(start) is not int or start < 0
            or not all(type(n) in (int, float) and math.isfinite(n) and n > 0 for n in (length, duration))):
        return unknown
    return dict(circuit=circuit, laps=laps, lap_length_m=length, lap_duration_s=duration, start_ns=start)


def recorded_course(report):
    """カメラ台は閉じた走行線を定速で進む。周回長は生成時の値を優先する。"""
    duration, speed = report.get('lap_duration_s'), report.get('speed_mps')
    length = report.get('lap_length_m')
    if length is None and type(duration) in (int, float) and type(speed) in (int, float):
        length = duration * speed
    audit = report.get('bag_audit') or {}
    return _validated(dict(circuit=report.get('circuit'), laps=report.get('laps_requested'),
                           lap_length_m=length, lap_duration_s=duration, start_ns=audit.get('lap_start_sim_ns')))


def load_course(bag: Path, trace_path: Path, results_root: Path = Path('/data/results')):
    """find_ground_truth で照合済みの trace に対してのみ周回情報を付与する。"""
    if trace_path.parent == bag / SIDECAR:
        manifest = json.loads((trace_path.parent / 'manifest.json').read_text())
        if 'course' in manifest:
            return _validated(manifest['course'])
    # 旧 bag は同じ試行の完了レポートが残っていれば復元できる。
    # レポートが欠けた場合でも、同梱済みの正解による距離軸の解析は可能。
    try:
        match = matching_run(bag, results_root)
        if match:
            directory, report = match
            original = directory / 'scene' / TRACE
            if trace_path == original or (original.is_file() and original.read_bytes() == trace_path.read_bytes()):
                return recorded_course(report)
    except (OSError, ValueError):
        pass
    return _validated(None)
