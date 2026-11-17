"""領域中心線と pinned 最小曲率 helper の契約・失敗境界。ROS は不要。"""
from copy import deepcopy
import math
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from atd_processing.geometry import DEFAULTS, generate, settings
from atd_processing.planning.geometry import PathValidationError, edges, simple
from atd_processing.planning.optimizer import optimize
from atd_processing.planning.optimizer_worker import _noncrossing_output
from atd_processing.planning.raceline_reference import build
from atd_processing.planning.iqp import monitored_iqp
from atd_processing.planning.lines import validate_raceline_curvature


def circle(radius, count):
    return [[radius * math.cos(i * math.tau / count), radius * math.sin(i * math.tau / count)] for i in range(count)]


def track():
    return dict(left=circle(3, 30), right=circle(5, 50), custom=circle(4, 40), closed=True)


def traced_course():
    # Kart の synthetic fixture。実地図・実測寸法・実車検証ではない。
    inner = [[596,391],[633,357],[793,357],[824,369],[802,378],[670,378],[609,420],
             [597,512],[656,570],[820,584],[849,605],[835,622],[627,617],[583,570]]
    outer = [[518,318],[617,262],[896,272],[969,334],[966,402],[878,471],[735,468],
             [720,476],[731,486],[881,494],[962,561],[959,653],[875,709],[610,706],[519,631]]
    return dict(left=[[x/100, -y/100] for x,y in inner],
                right=[[x/100, -y/100] for x,y in outer], closed=True, custom=[])


def test_only_requested_methods_and_pinned_defaults():
    assert settings({})['centerline_method'] == 'region'
    assert settings({})['optimizer'] == 'mincurv'
    assert settings({})['centerline_max_points'] == 800
    for options in ({'centerline_method': 'paired'}, {'optimizer': 'local'}, {'optimizer': 'smoothing'}):
        with pytest.raises(ValueError, match='廃止'):
            settings(options)
    for value in (True, 12.5, 11, 1501):
        with pytest.raises(ValueError):
            settings({'centerline_max_points': value})


def test_region_unequal_counts_winding_phase_and_immutability():
    original = track()
    for reverse in (False, True):
        doc = deepcopy(original)
        doc['right'] = doc['right'][7:] + doc['right'][:7]
        if reverse:
            doc['right'].reverse()
        before = deepcopy(doc)
        result = generate(doc, 'centerline', {})
        assert result['method'] == 'region_equal_distance_v1'
        assert result['region_info']['max_relative_imbalance'] < .02
        assert all(3.95 < math.hypot(*p) < 4.02 for p in result['points'])
        assert doc == before
        area = lambda points: sum(a[0]*b[1]-a[1]*b[0] for a,b in edges(points, True))
        assert area(result['points']) * area(doc['left']) > 0
        assert result['collision_model'] == 'rear_axle_rectangle_v1'


def test_region_concave_equal_distance_and_point_budget():
    result = generate(traced_course(), 'centerline', {})
    assert result['region_info']['max_relative_imbalance'] < .02
    assert max(abs(a-b)/(a+b) for a,b in result['widths']) < .02
    with pytest.raises(ValueError, match='設定上限'):
        generate(track(), 'centerline', {'spacing': .05, 'centerline_max_points': 32})


def test_region_open_memory_missing_numpy_and_cancel_never_fallback(monkeypatch):
    from atd_processing.planning import region_centerline
    with pytest.raises(ValueError, match='閉路専用'):
        generate(dict(track(), closed=False), 'centerline', {})
    monkeypatch.setattr(region_centerline, 'MAX_CELLS', 10)
    with pytest.raises(ValueError, match='格子が大きすぎ'):
        generate(track(), 'centerline', {})
    monkeypatch.undo()
    monkeypatch.setitem(__import__('sys').modules, 'numpy', None)
    with pytest.raises(ValueError, match='NumPy'):
        generate(track(), 'centerline', {})
    monkeypatch.undo()
    check = Mock(side_effect=RuntimeError('cancel marker'))
    with pytest.raises(RuntimeError, match='cancel marker'):
        generate(track(), 'centerline', {}, check)


def test_race_requires_saved_reference_and_missing_env_does_not_fallback(monkeypatch):
    doc = track()
    with pytest.raises(ValueError, match='先に.*Centerline'):
        generate(doc, 'raceline', {})
    doc['lines'] = {'centerline': generate(doc, 'centerline', {})}
    original = deepcopy(doc)
    monkeypatch.delenv('ATD_RACELINE_PYTHON', raising=False)
    with pytest.raises(ValueError, match='専用環境'):
        generate(doc, 'raceline', {})
    assert doc == original
    doc['lines']['centerline']['points'] = [[9,9], [10,9], [9,10]]
    with pytest.raises(PathValidationError, match='保存済みCenterline'):
        build(doc, .1)


def test_long_rectangle_is_checked_in_addition_to_centerline_clearance():
    doc = dict(left=[[0,.5],[2,.5],[2,2],[1,2],[1,1],[0,1]],
               right=[[-1,-.5],[3,-.5],[3,3],[0,3],[0,2],[-1,2]],
               custom=[[0,0],[2,0],[2,2],[.5,2],[.5,1.5]], closed=False)
    # Independent simple corridor: a narrow open straight admits the axle line,
    # but a long, oblique body intersects its edges.
    doc = dict(left=[[0,.3],[10,.3]], right=[[0,-.3],[10,-.3]],
               custom=[[1,0],[2,.1],[3,0]], closed=False)
    with pytest.raises(PathValidationError, match='長方形車体'):
        generate(doc, 'custom', {'vehicle_length': 4, 'rear_axle_to_rear': 1})


def test_topology_backtracking_keeps_order_and_curvature_checks():
    bowtie = [[-1,-1],[1,1],[-1,1],[1,-1]]
    with pytest.raises(PathValidationError) as caught:
        simple(bowtie, True)
    assert caught.value.diagnostic['reason'] == 'self_intersection'
    create = Mock(return_value=bowtie)
    with pytest.raises(PathValidationError):
        _noncrossing_output(create)
    assert create.call_count == 9
    create = lambda scale: circle(.1 / scale, 20)
    line, scale, rejected = _noncrossing_output(create, lambda line: validate_raceline_curvature(line, {'curvature_limit': 6}))
    assert scale == .5 and rejected[0]['diagnostic']['reason'] == 'curvature'
    validate_raceline_curvature(line, {'curvature_limit': 6})


def test_iqp_keeps_upstream_convergence_and_restores_solver():
    solver = Mock(side_effect=[([0], error) for error in [.5]*25+[.005]])
    tph = SimpleNamespace(opt_min_curv=SimpleNamespace(opt_min_curv=solver))
    def loop(**args):
        while True:
            result = tph.opt_min_curv.opt_min_curv(reftrack=[[0,0,1,1]])
            if result[1] <= args['curv_error_allowed']:
                return result
    tph.iqp_handler = SimpleNamespace(iqp_handler=loop)
    history = []
    monitored_iqp(tph, dict(curv_error_allowed=.01), history=history)
    assert solver.call_count == 26 and len(history[0]) == 26
    assert tph.opt_min_curv.opt_min_curv is solver


@pytest.mark.parametrize('method', ['mincurv', 'mincurv_iqp'])
def test_real_pinned_helper(method):
    python = os.environ.get('ATD_TEST_RACELINE_PYTHON')
    if not python or not Path(python).is_file():
        pytest.skip('ATD_TEST_RACELINE_PYTHON に専用環境を指定したとき実 helper を検証')
    os.environ['ATD_RACELINE_PYTHON'] = python
    doc = track()
    doc['lines'] = {'centerline': generate(doc, 'centerline', {'spacing': .25})}
    before = deepcopy(doc)
    result = generate(doc, 'raceline', {'optimizer': method, 'spacing': .25})
    assert result['method'] == 'tph_' + method
    assert result['optimizer_info']['package'] == 'trajectory-planning-helpers'
    assert result['reference_info']['source'] == 'saved_centerline'
    assert result['optimizer_info']['reference_info'] == result['reference_info']
    assert result['optimizer_info']['version'] == '0.80'
    assert result['settings']['optimizer'] == method
    assert max(abs(row[4]) for row in result['profile']) <= 1.01
    assert len(result['points']) == len(result['speeds'])
    assert doc == before


def test_helper_wrong_fork_is_rejected_before_numerical_import(monkeypatch):
    import importlib.metadata
    import json
    from atd_processing.planning.optimizer_worker import pinned_installation, NUMERICAL_PINS
    def installed(name):
        return SimpleNamespace(version=NUMERICAL_PINS[name], read_text=lambda _file: json.dumps({
            'url': 'https://github.com/other/fork', 'vcs_info': {'commit_id': 'wrong'}}))
    monkeypatch.setattr(importlib.metadata, 'distribution', installed)
    with pytest.raises(ValueError, match='固定 fork / commit'):
        pinned_installation()


def test_old_paired_saved_center_is_display_only_for_new_optimization():
    doc = track()
    doc['lines'] = {'centerline': {'points': circle(4, 40), 'method': 'paired_boundary_midpoints'}}
    before = deepcopy(doc)
    with pytest.raises(ValueError, match='領域方式.*再生成'):
        generate(doc, 'raceline', {})
    assert doc == before


def test_optimizer_cancel_stops_dedicated_process(monkeypatch):
    import subprocess
    import sys
    from atd_processing.planning import optimizer as module
    original = subprocess.Popen
    processes = []
    def slow_helper(_argv, **kwargs):
        process = original([sys.executable, '-c', 'import time; time.sleep(20)'], **kwargs)
        processes.append(process)
        return process
    monkeypatch.setenv('ATD_RACELINE_PYTHON', sys.executable)
    monkeypatch.setattr(module.subprocess, 'Popen', slow_helper)
    with pytest.raises(RuntimeError, match='cancel helper'):
        optimize({'closed': True}, {}, Mock(side_effect=RuntimeError('cancel helper')))
    assert processes and all(process.poll() is not None for process in processes)


def test_curvature_arc_metrics_are_sample_count_independent_and_not_signed_turning():
    from atd_processing.planning.lines import curvature_metrics
    coarse = curvature_metrics(circle(4, 48))
    dense = curvature_metrics(circle(4, 384))
    inner, outer = curvature_metrics(circle(3.2, 384)), curvature_metrics(circle(4.8, 384))
    assert coarse['mean_squared_curvature_1_per_m2'] == pytest.approx(1/16, rel=1e-8)
    assert dense['mean_squared_curvature_1_per_m2'] == pytest.approx(1/16, rel=1e-8)
    assert coarse['squared_curvature_integral_1_per_m'] == pytest.approx(2*math.pi/4, rel=.001)
    assert dense['squared_curvature_integral_1_per_m'] == pytest.approx(coarse['squared_curvature_integral_1_per_m'], rel=.001)
    assert inner['squared_curvature_integral_1_per_m'] > outer['squared_curvature_integral_1_per_m']
    assert inner['signed_curvature_integral_rad'] == pytest.approx(2*math.pi, rel=.001)
    assert outer['signed_curvature_integral_rad'] == pytest.approx(2*math.pi, rel=.001)


def test_real_curvature_jacobian_matches_independent_tph_finite_difference():
    import subprocess
    import json
    python = os.environ.get('ATD_TEST_RACELINE_PYTHON')
    if not python or not Path(python).is_file():
        pytest.skip('専用 helper 環境が必要です')
    code = r'''
import json,numpy as np
import trajectory_planning_helpers as tph
from atd_processing.planning.curvature_qp import linearization
n=16; theta=np.arange(n)*2*np.pi/n
ref=np.column_stack((4*np.cos(theta),2.8*np.sin(theta)))
cx,cy,A,normal=tph.calc_splines.calc_splines(np.vstack((ref,ref[0])),use_dist_scaling=False)
k,J,_=linearization(np.column_stack((ref,np.ones((n,2)))),normal,A)
def independently_recompute(alpha):
    moved=ref+normal*alpha[:,None]
    x,y,_,_=tph.calc_splines.calc_splines(np.vstack((moved,moved[0])),use_dist_scaling=False)
    return tph.calc_head_curv_an.calc_head_curv_an(coeffs_x=x,coeffs_y=y,ind_spls=np.arange(n),t_spls=np.zeros(n))[1]
finite=np.empty_like(J)
for j in range(n):
    delta=np.zeros(n);delta[j]=1e-6
    finite[:,j]=(independently_recompute(delta)-independently_recompute(-delta))/(2e-6)
print(json.dumps({'max_abs_error':float(np.max(np.abs(finite-J))),'max_gradient':float(np.max(np.abs(finite)))}))
'''
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[2] / 'processing'), OPENBLAS_NUM_THREADS='1')
    done = subprocess.run([python, '-c', code], env=env, text=True, capture_output=True, timeout=30)
    assert done.returncode == 0, done.stderr
    result = json.loads(done.stdout)
    assert result['max_abs_error'] < 1e-6
    assert result['max_gradient'] > .1


@pytest.mark.parametrize('method', ['mincurv', 'mincurv_iqp'])
@pytest.mark.parametrize('clockwise', [False, True])
def test_real_annulus_moves_outward_and_improves_true_curvature(method, clockwise, monkeypatch):
    python = os.environ.get('ATD_TEST_RACELINE_PYTHON')
    if not python or not Path(python).is_file():
        pytest.skip('専用 helper 環境が必要です')
    monkeypatch.setenv('ATD_RACELINE_PYTHON', python)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '1')
    from atd_processing.planning.lines import curvature_metrics
    doc = dict(left=circle(5, 48), right=circle(3, 36), closed=True, custom=[])
    if clockwise:
        doc['left'].reverse(); doc['right'].reverse()
    doc['lines'] = {'centerline': generate(doc, 'centerline', {})}
    before = deepcopy(doc)
    result = generate(doc, 'raceline', {'optimizer': method, 'spacing': .2})
    radius = sum(math.hypot(*point) for point in result['points']) / len(result['points'])
    assert radius > 4.3  # 元の frozen-tangent QP は CW/CCW とも約3.15へ縮んだ。
    metrics = curvature_metrics(result['points'])
    assert metrics['squared_curvature_integral_1_per_m'] < .95 * (2*math.pi/4)
    assert metrics['signed_curvature_integral_rad'] == pytest.approx((-1 if clockwise else 1)*2*math.pi, abs=.02)
    info = result['optimizer_info']
    assert info['curvature_linearization'] == 'tph_full_curvature_jacobian_v1'
    assert info['actual_objective_verified'] is True
    assert info['output_curvature_metrics'] == metrics
    assert info['output_curvature_metrics']['squared_curvature_integral_1_per_m'] <= info['reference_curvature_metrics']['squared_curvature_integral_1_per_m'] * 1.0001
    if method == 'mincurv_iqp':
        assert info['step_backtracking'] == 'cumulative_from_initial_reference_spline'
    assert doc == before


@pytest.mark.parametrize('method', ['mincurv', 'mincurv_iqp'])
def test_real_concave_fixture_keeps_all_geometry_checks_and_improves_objective(method, monkeypatch):
    python = os.environ.get('ATD_TEST_RACELINE_PYTHON')
    if not python or not Path(python).is_file():
        pytest.skip('専用 helper 環境が必要です')
    monkeypatch.setenv('ATD_RACELINE_PYTHON', python)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '1')
    doc = traced_course()
    doc['lines'] = {'centerline': generate(doc, 'centerline', {})}
    before = deepcopy(doc)
    result = generate(doc, 'raceline', {'optimizer': method, 'spacing': .2, 'curvature_limit': 20})
    info = result['optimizer_info']
    assert info['actual_objective_verified'] is True
    assert info['output_curvature_metrics']['squared_curvature_integral_1_per_m'] < info['reference_curvature_metrics']['squared_curvature_integral_1_per_m']
    assert result['collision_model'] == 'rear_axle_rectangle_v1'
    simple(result['points'], True)
    assert max(abs(row[4]) for row in result['profile']) <= 20*1.01
    assert doc == before
