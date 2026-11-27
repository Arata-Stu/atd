import math

import pytest

from atd_localization.rpm_comparison import FIELDS, aligned_rows
from atd_localization.state import RigidPose


def pose(x, y=0., yaw=0.):
    return RigidPose((x, y, 0.), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))


def test_jump_is_separated_from_motion_and_not_applied_to_ekf():
    t = 1_000_000_000
    d = 10_000_000
    wheel = {t + i*d: ('segment', (float(i), 0., 0.)) for i in (0, 2, 4)}
    vo = {t+d: pose(1.), t+3*d: pose(3.)}
    origin = pose(8., 2., math.pi/2)
    # 2 m of actual forward travel and a 0.25 m left map correction.
    statuses = [(t+d, ('a', 0, 'course', 'hash'), origin.compose(pose(1.))),
                (t+3*d, ('a', 0, 'course', 'hash'), origin.compose(pose(3., .25)))]
    first, second = [dict(zip(FIELDS, row, strict=True)) for row in aligned_rows(wheel, vo, statuses)]
    assert first['correction_left_m'] == ''
    assert second['correction_forward_m'] == pytest.approx(0., abs=1e-9)
    assert second['correction_left_m'] == pytest.approx(.25)
    assert second['aligned_ekf_x_m'] == pytest.approx(8.)
    assert second['aligned_vo_y_m'] == pytest.approx(5.)


def test_restart_and_tracking_loss_do_not_bridge():
    ts = [1_000_000_000 + i*10_000_000 for i in range(5)]
    wheel = {ts[0]: ('a', (0., 0., 0.)), ts[2]: ('b', (0., 0., 0.)), ts[4]: ('b', (2., 0., 0.))}
    statuses = [(t, ('A', 0, 'course', 'hash'), None if t == ts[3] else pose(10.)) for t in ts]
    rows = list(aligned_rows(wheel, dict.fromkeys(ts, pose(0.)), statuses))
    assert [r[0] for r in rows] == [ts[0], ts[2], ts[4]]
    assert all(r[-1] == '' for r in rows)


def test_relocalization_keeps_ekf_anchor_on_same_map():
    t = 1_000_000_000
    d = 10_000_000
    wheel = {t+i*d: ('segment', (float(i), 0., 0.)) for i in range(3)}
    visual = {t: pose(0.), t+2*d: pose(0.)}
    statuses = [(t, ('a', 0, 'course', 'hash'), pose(10.)),
                (t+d, ('a', 1, 'course', 'hash'), None),
                (t+2*d, ('a', 1, 'course', 'hash'), pose(14.))]
    row = dict(zip(FIELDS, list(aligned_rows(wheel, visual, statuses))[-1], strict=True))
    assert row['aligned_ekf_x_m'] == pytest.approx(12.)
    assert row['map_x_m'] == 14.
    assert row['correction_left_m'] == ''
