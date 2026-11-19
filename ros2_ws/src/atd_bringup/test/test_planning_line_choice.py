"""地図選択後に生成済みラインだけを提示する bringup の入口。"""
import json
from pathlib import Path
import runpy

import pytest


@pytest.mark.parametrize(('available', 'requested', 'choice', 'expected'), [
    (['centerline', 'custom'], None, 'custom', 'custom'),
    (['centerline', 'custom'], None, None, None),
    (['centerline'], None, None, 'centerline'),
    ([], None, None, ''),
    (['raceline', 'custom'], 'raceline', None, 'raceline'),
])
@pytest.mark.parametrize('autonomous', [False, True])
def test_map_then_saved_line(tmp_path, monkeypatch, available, requested, choice, expected, autonomous):
    package = Path(__file__).resolve().parents[1]
    for dependency in ('atd_localization', 'atd_mapping', 'atd_planning'):
        monkeypatch.syspath_prepend(str(package.parent / dependency))
    from atd_localization import assets
    folder = tmp_path / 'maps' / 'track'
    folder.mkdir(parents=True)
    coords = [[0, 0], [1, 0], [1, 1]]
    saved = dict(points=coords, profile=[[i, x, y, 0, 0, 1, 0] for i, (x, y) in enumerate(coords)])
    doc = dict(frame='map', closed=True, custom=coords, lines={key: saved for key in available})
    monkeypatch.setattr(assets, 'map_metadata', lambda root, key: {'map_id': key})
    monkeypatch.setattr(assets, 'verified_hdmap', lambda root, metadata: json.dumps(doc).encode())
    entry = runpy.run_path(str(package / 'scripts/choose_localization_map'))['main']
    def select(label, options):
        if label == '自己位置推定の地図':
            assert options == ['track']
            return 'track'
        assert requested is None and len(available) > 1 and options == available
        return choice
    monkeypatch.setitem(entry.__globals__, 'choose', select)
    launched = []
    monkeypatch.setattr('os.execvp', lambda _program, args: launched.extend(args))
    args = (['--autonomous'] if autonomous else []) + [f'results_root:={tmp_path}']
    if requested:
        args.append('line:=' + requested)
    if autonomous and not available:
        with pytest.raises(ValueError, match='速度付きの保存ライン'):
            entry(args)
        assert not launched
        return
    entry(args)
    if expected is None:
        assert not launched
    else:
        assert launched[:4] == ['ros2', 'launch', 'atd_bringup',
                                'autonomous.launch.py' if autonomous else 'localization.launch.py']
        assert 'map_id:=track' in launched
        assert [arg for arg in launched if arg.startswith('line:=')] == (['line:=' + expected] if expected else [])
