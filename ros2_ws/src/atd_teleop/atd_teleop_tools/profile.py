"""操作割り当ての厳密な検査と atomic 保存。物理入力の校正とは別管理。"""
from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile

import yaml

AXES = ("left_x", "left_y", "right_x", "right_y", "l2", "r2", "dpad_x", "dpad_y")
BUTTONS = ("cross", "circle", "triangle", "square", "l1", "r1", "l2", "r2",
           "create", "options", "ps", "l3", "r3", "dpad_up", "dpad_down", "dpad_left", "dpad_right")


def default_profile():
    return {"version": 1, "axes": {
        role: {"axis": "", "subtract_axis": "", "scale": 1.0}
        for role in ("steering", "throttle")},
        "buttons": {role: "" for role in ("enable", "record_start", "record_stop")}}


def _keys(value, keys):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError("必要なキー: " + ", ".join(keys))


def _name(value, names):
    if type(value) is not str or value not in ("", *names):
        raise ValueError("不明な入力名: " + repr(value))
    return value


def validate(profile):
    _keys(profile, ("version", "axes", "buttons"))
    if type(profile["version"]) is not int or profile["version"] != 1:
        raise ValueError("操作割り当ての version は整数 1 を指定してください")
    _keys(profile["axes"], ("steering", "throttle"))
    _keys(profile["buttons"], ("enable", "record_start", "record_stop"))
    result = default_profile()
    for role, binding in profile["axes"].items():
        _keys(binding, ("axis", "subtract_axis", "scale"))
        scale = binding["scale"]
        if type(scale) not in (int, float) or not math.isfinite(scale) or not -1 <= scale <= 1:
            raise ValueError("scale は -1〜1 の有限数を指定してください")
        result["axes"][role] = {
            "axis": _name(binding["axis"], AXES),
            "subtract_axis": _name(binding["subtract_axis"], AXES), "scale": float(scale)}
    used = set()
    for role, button in profile["buttons"].items():
        name = _name(button, BUTTONS)
        if name and name in used:
            raise ValueError("同じボタンに複数の役割は割り当てられません: " + name)
        used.add(name)
        result["buttons"][role] = name
    return result


class _UniqueLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if type(key) is not str:
            raise ValueError("YAML のキーは文字列で指定してください")
        if key in result:
            raise ValueError("YAML の重複キー: " + key)
        result[key] = loader.construct_object(value_node, deep=True)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def load(path):
    path = Path(path)
    if path.stat().st_size > 65536:
        raise ValueError("操作割り当ては 64 KiB 以下にしてください")
    return validate(yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueLoader))


def save(path, profile):
    value = validate(profile)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(value, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
