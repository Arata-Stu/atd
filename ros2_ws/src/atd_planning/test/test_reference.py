import math

import pytest

from atd_planning.core import Planner, ReferenceLine, yaw_from_quaternion


def document(points, speeds=None, closed=False, key='custom'):
    speeds = speeds or [1.] * len(points)
    s, profile = 0., []
    for i, (x, y) in enumerate(points):
        if i:
            s += math.dist(points[i - 1], points[i])
        profile.append([s, x, y, 0., 0., speeds[i], 0.])
    return dict(frame='map', closed=closed, lines={key: dict(points=points, profile=profile, speeds=speeds)})


def observe(planner, x, y, yaw, t=1., v=1., epoch=0, session='run', **kwargs):
    return planner.update(x=x, y=y, yaw=yaw, speed_mps=v, epoch=epoch, session_id=session,
                          stamp_ns=round(t * 1e9), **kwargs)


@pytest.mark.parametrize('key', ['raceline', 'centerline', 'custom'])
def test_saved_profiles_are_selected_by_key(key):
    doc = document([[0, 0], [1, 0], [1, 2]], [0, 2, 0], key=key)
    doc['custom'] = [[9, 9], [10, 10]]
    route = ReferenceLine.from_document(doc, key)
    planner = Planner(route, horizon_m=1.)
    result = observe(planner, .5, .1, 0., v=-.2)
    assert result['valid'] and result['progress_m'] == pytest.approx(.5)
    assert result['estimated_speed_mps'] == -.2 and result['lateral_error_m'] == pytest.approx(.1)
    assert result['target_speed_mps'] == pytest.approx(math.sqrt(2.))
    assert result['trajectory'] == [[.5, 0.], [1, 0], [1., .5]]
    assert result['trajectory_speeds_mps'] == pytest.approx([math.sqrt(2.), 2., math.sqrt(3.)])
    assert all(type(value) is float for value in result['trajectory_speeds_mps'])


def test_closed_seam_and_open_endpoint_do_not_draw_shortcut():
    rect = document([[0, 0], [2, 0], [2, 2], [0, 2]], closed=True)
    route = ReferenceLine.from_document(rect, 'custom')
    planner = Planner(route, horizon_m=.5)
    result = observe(planner, 0, .1, -math.pi / 2)
    assert result['progress_m'] == pytest.approx(7.9)
    assert len(result['trajectory']) == 3
    assert result['trajectory'][1] == [0, 0]
    assert result['trajectory'][-1] == pytest.approx([.4, 0])
    assert observe(planner, .12, .01, 0, t=1.1)['progress_m'] == pytest.approx(.12)
    open_route = ReferenceLine.from_document(document([[0, 0], [1, 0], [2, 0]], [0, 1, 0]), 'custom')
    p, v = open_route.preview(1.9)
    assert p == [[1.9, 0.], [2., 0.]] and v[-1] == 0
    assert open_route.preview(2.) == ([[2., 0.]], [0.])


def test_close_parallel_lanes_progress_gate_and_reacquisition():
    # The first and last legs have opposing headings at a hairpin.
    points = [[0, 0], [3, 0], [3, .2], [0, .2]]
    planner = Planner(ReferenceLine.from_document(document(points), 'custom'))
    assert observe(planner, 1, .18, 0)['progress_m'] == pytest.approx(1.)
    other = observe(planner, 1, .19, math.pi, t=1.1)
    assert not other['valid'] and not other['trajectory'] and other['target_speed_mps'] is None
    assert observe(planner, 1, .19, math.pi, t=1.2, epoch=1)['progress_m'] == pytest.approx(5.2)
    far = observe(planner, 10, 10, 0, t=1.3)
    assert not far['valid']
    assert observe(planner, 1, .19, math.pi, t=1.4, epoch=1, include_trajectory=False)['trajectory'] == []


def test_parallel_same_direction_lanes_require_clear_branch_at_acquisition():
    points = [[0, 0], [2, 0], [3, .5], [-1, .5], [0, .1], [2, .1]]
    planner = Planner(ReferenceLine.from_document(document(points), 'custom'))
    assert not observe(planner, 1, .05, 0)['valid']
    assert observe(planner, 1, 0, 0, t=1.1)['progress_m'] == pytest.approx(1.)
    near = observe(planner, 1.03, .08, 0, t=1.2)
    assert near['progress_m'] == pytest.approx(1.03)
    assert not observe(planner, 1, 0, 0, t=1.1)['valid']
    assert observe(planner, 1, .1, 0, t=3.)['progress_m'] > 6.


@pytest.mark.parametrize('corrupt', ['missing', 'extra', 'speed', 'position', 'arc', 'duplicate'])
def test_wrong_saved_profile_is_rejected(corrupt):
    doc = document([[0, 0], [1, 0], [2, 1]])
    entry = doc['lines']['custom']
    if corrupt == 'missing':
        doc['custom'] = entry['points']
        del doc['lines']['custom']
    elif corrupt == 'extra':
        entry['profile'].append(entry['profile'][-1])
    elif corrupt == 'speed':
        entry['profile'][0][5] = float('nan')
    elif corrupt == 'position':
        entry['profile'][0][1] = 11
    elif corrupt == 'arc':
        entry['profile'][1][0] = .5
    else:
        entry['points'][1] = [0, 0]
        entry['profile'][1][1:3] = [0, 0]
    with pytest.raises(ValueError):
        ReferenceLine.from_document(doc, 'custom')


def test_yaw_preserves_tilt_and_rejects_missing_pose():
    assert yaw_from_quaternion(0., 0., .5, math.sqrt(.75)) == pytest.approx(math.pi / 3)
    with pytest.raises(ValueError):
        yaw_from_quaternion(0., 0., 0., 0.)
