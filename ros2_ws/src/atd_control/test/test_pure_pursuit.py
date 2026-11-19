import math

import pytest

from atd_control.core import LookaheadConfig, pure_pursuit
from atd_control.vehicle import load_measured_wheelbase
from atd_vehicle_layout.profile import default_profile, save


def follow(path, **kwargs):
    return pure_pursuit(path, **(dict(x=0., y=0., yaw=0., speed_mps=1., wheelbase_m=.25) | kwargs))


@pytest.mark.parametrize(('speed', 'expected'), [(0., .35), (1., .75), (3., 1.5), (20., 1.5)])
def test_lookahead_uses_estimated_speed_and_interpolates_sparse_trajectory(speed, expected):
    data = follow([(0., 0.), (5., 0.)], speed_mps=speed)
    assert data['valid'] and data['controller'] == 'pure_pursuit'
    assert data['requested_lookahead_m'] == pytest.approx(expected)
    assert data['actual_lookahead_m'] == pytest.approx(expected)
    assert data['target'] == pytest.approx([expected, 0.])
    assert data['curvature_per_m'] == data['steering_rad'] == 0.
    assert not data['preview_limited']


def test_auto_standstill_band_preserves_forward_lookahead_and_excludes_reverse():
    path = [[0., 0.], [2., 0.]]
    for noise in (-.05, -.005, .05):
        auto = follow(path, speed_mps=noise, low_speed_drive=True)
        assert auto['valid'] and auto['requested_lookahead_m'] == .35
    assert not follow(path, speed_mps=-.005)['valid']
    assert not follow(path, speed_mps=-.051, low_speed_drive=True)['valid']


@pytest.mark.parametrize('turn', [-1., 1.])
def test_exact_symmetric_circle_curvature_and_wheel_angle_in_rotated_frame(turn):
    # 前進方向は map の +y。円上の既知の点を通る短い直線近似から曲率を復元する。
    angle, radius = .42, 2.
    forward, left = radius * math.sin(angle), turn * radius * (1. - math.cos(angle))
    goal = [3. - left, 4. + forward]
    distance = math.hypot(forward, left)
    data = follow([(3., 4.), goal, (3. - 2 * left, 4. + 2 * forward)], x=3., y=4., yaw=math.pi / 2,
                  config=LookaheadConfig(distance, 0., distance))
    assert data['valid'] and not data['preview_limited']
    assert data['target'] == pytest.approx(goal)
    assert data['curvature_per_m'] == pytest.approx(turn / radius)
    assert data['steering_rad'] == pytest.approx(math.atan(.25 * turn / radius))


def test_uses_chord_distance_from_offset_rear_axle_not_distance_along_path():
    data = follow([(0., 0.), (4., 0.)], y=.2)
    assert data['target'] == pytest.approx([math.sqrt(.75 ** 2 - .2 ** 2), 0.])
    assert data['curvature_per_m'] == pytest.approx(-.4 / .75 ** 2)


def test_uses_actual_chord_for_truncated_preview_and_unknown_wheelbase():
    data = follow([(0., .02), (.2, .1)], wheelbase_m=None)
    assert data['valid'] and data['preview_limited']
    assert data['actual_lookahead_m'] == pytest.approx(math.hypot(.2, .1))
    assert data['requested_lookahead_m'] == pytest.approx(.75)
    assert data['curvature_per_m'] == pytest.approx(4.) and data['steering_rad'] is None


def test_never_chooses_rear_half_of_circle_or_later_intersection():
    data = follow([(-2., 0.), (2., 0.), (0., 2.)])
    assert data['target'] == pytest.approx([.75, 0.])


@pytest.mark.parametrize('path,kwargs', [
    ([], {}), ([(0, 0)], {}), ([(.2, 0), (.2, 0)], {}),
    ([(0, 0), (-1, 0)], {}), ([(0, 1), (2, 1)], {}), ([(0, 0), (2, 0)], {'speed_mps': -.5}),
    ([(0, 0), (1, math.nan)], {}), ([(0, 0), (1, 0)], {'wheelbase_m': 0.}),
])
def test_invalid_inputs_clear_all_geometry(path, kwargs):
    result = follow(path, **kwargs)
    assert not result['valid'] and result['reason'] and result['target'] is None
    assert result['curvature_per_m'] is result['steering_rad'] is result['actual_lookahead_m'] is None


@pytest.mark.parametrize('minimum,time,maximum', [(0., 1., 1.), (.3, -1., 1.), (3., 1., 1.), (.3, math.nan, 1.)])
def test_invalid_config_rejected(minimum, time, maximum):
    with pytest.raises(ValueError):
        LookaheadConfig(minimum, time, maximum)


def test_only_measured_wheelbase_used(tmp_path):
    path = tmp_path / 'layout.yaml'
    assert load_measured_wheelbase(path) is None
    doc = default_profile()
    for source, value in [('unmeasured', None), ('cad_candidate', .22), ('manual', .25)]:
        doc['dimensions']['wheelbase'] = dict(value_m=value, source=source)
        save(path, doc)
        assert load_measured_wheelbase(path) == (value if source == 'manual' else None)
    path.write_text('invalid: [')
    with pytest.raises(ValueError, match='車両配置'):
        load_measured_wheelbase(path)
