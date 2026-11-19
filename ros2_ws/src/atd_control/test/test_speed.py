import json

import pytest

from atd_control.speed import DriveFeedback, FeedforwardTable, PIDConfig, SpeedController, limited_target, manual_reading
from atd_control.speed_table_file import SpeedTableFile


def document(decreasing=False):
    raw = [1400, 1200, 1100] if decreasing else [1100, 1200, 1400]
    return dict(schema='atd.throttle_feedforward.v1', usable=True, direction='decreasing' if decreasing else 'increasing',
                input_unit='m/s', output_unit='sbus_raw_ch2', interpolation='linear', extrapolation='none',
                run_id='day-a', proposal_sha256='a' * 64, speed_min_mps=.5, speed_max_mps=2.,
                points=[dict(speed_mps=v, throttle_raw=t) for v, t in zip((.5, 1., 2.), raw)])


def test_low_speed_limit_applies_to_ff_and_pid_and_preserves_invalid_targets():
    ctl = SpeedController(FeedforwardTable.from_document(document()), PIDConfig(kp=20., ki=5., kd=0.),
                          max_target_speed_mps=.75)
    limited = ctl.update(1, 2., .4)
    assert limited['target_speed_mps'] == .75 and limited['feedforward_raw'] == 1150
    assert limited['p_raw'] == pytest.approx(7.) and limited['calculated_raw'] == 1150 and limited['saturated']
    assert ctl.update(10_000_001, 2., .4)['i_raw'] == 0.
    assert ctl.update(20_000_001, .6, .5)['calculated_raw'] > 1120
    decreasing = SpeedController(FeedforwardTable.from_document(document(True)), max_target_speed_mps=.75)
    assert decreasing.update(1, 2., .4)['calculated_raw'] == 1300.
    assert limited_target(2., 0.) == 2. and limited_target(.2, .5) == .2
    for value in (float('nan'), float('inf'), -1.):
        assert not ctl.update(25_000_001, value, .4)['valid']
    for value in (float('nan'), -1., 1.01):
        with pytest.raises(ValueError):
            limited_target(.2, value)


def test_low_speed_vo_standstill_noise_keeps_original_measurement():
    table = FeedforwardTable.from_document(document())
    ctl = SpeedController(table, PIDConfig(10., 0., 100.), max_target_speed_mps=.75)
    for t, measured in [(1, -.005), (100_000_001, .02), (200_000_001, -.05)]:
        sample = ctl.update(t, .6, measured)
        assert sample['valid'] and sample['estimated_speed_mps'] == measured
        assert sample['speed_error_mps'] == pytest.approx(.6 - measured)
        assert sample['p_raw'] == 6. and sample['d_raw'] == 0.
    assert not ctl.update(300_000_001, .6, -.051)['valid']
    assert not SpeedController(table).update(1, .6, -.005)['valid']


def test_pid_history_requires_a_continuous_fresh_active_source_and_route():
    gate = DriveFeedback()
    doc = dict(active=True, braking=False, map_id='map1', line='custom', session_id='node-boot-1')
    route = ('map1', 'custom')

    def sample(t, **fields):
        return gate.update(doc | dict(stamp_ns=t) | fields, t, t)

    assert not gate.matches(route, 1, 1)
    assert not sample(10) and gate.matches(route, 10, 10)  # 初回 active は履歴初期化。
    assert sample(20) and not gate.matches(('map2', 'custom'), 20, 20)
    assert not gate.matches(route, 300_000_020, 300_000_020)
    assert not gate.matches(route, 20, 300_000_020)
    assert not gate.update(doc | dict(stamp_ns=20), 300_000_020, 300_000_020)
    assert not sample(300_000_021) and sample(300_000_022)
    assert not sample(300_000_023, active=False) and not gate.matches(route, 300_000_023, 300_000_023)
    assert not sample(300_000_024) and sample(300_000_025)
    assert not sample(300_000_026, braking=True)
    assert not sample(300_000_027) and sample(300_000_028)
    assert not sample(300_000_029, session_id='node-boot-2')
    assert not sample(300_000_030, line='raceline', session_id='node-boot-2')
    assert not gate.update(None, 300_000_031, 300_000_031)
    assert not gate.update(doc | dict(stamp_ns=310_000_000), 300_000_031, 300_000_031)


@pytest.mark.parametrize('decreasing', [True, False])
def test_signed_pid_and_interpolation_in_counts(decreasing):
    table = FeedforwardTable.from_document(document(decreasing))
    speed = SpeedController(table, PIDConfig(kp=20., ki=10., kd=2.))
    sign = table.direction
    ff = table.interpolate(1.5)
    assert ff == (1150. if decreasing else 1300.)
    first = speed.update(1_000_000_000, 1.5, 1., 'session')
    assert first['feedforward_raw'] == ff and first['p_raw'] == 10. * sign
    assert first['i_raw'] == first['d_raw'] == 0. and first['calculated_raw'] == ff + 10. * sign
    next_ = speed.update(1_200_000_000, 1.5, 1.2, 'session')
    assert next_['p_raw'] == pytest.approx(6. * sign)
    assert next_['i_raw'] == pytest.approx(.8 * sign)
    assert next_['d_raw'] == pytest.approx(-2. * sign)
    assert next_['calculated_raw'] == pytest.approx(ff + 4.8 * sign) and not next_['saturated']
    assert next_ == speed.update(1_200_000_000, 1.5, 1.2, 'session')
    # A target step should not kick the derivative when the measured velocity stays constant.
    assert speed.update(1_250_000_000, 1.4, 1.2, 'session')['d_raw'] == 0.


def test_no_unmeasured_stop_brake_reverse_or_extrapolation_and_pid_resets():
    controller = SpeedController(FeedforwardTable.from_document(document()))
    def update(t, v=1., estimate=.9, epoch=1):
        return controller.update(round(t * 1e9), v, estimate, epoch)
    assert update(1.)['valid'] and update(1.1)['i_raw'] > 0.
    assert update(2.)['i_raw'] == update(2.1, epoch=2)['d_raw'] == 0.
    for i, (goal, measured) in enumerate([(0., .9), (.49, .9), (2.01, .9), (1., -.01)]):
        t = 3 + i
        data = update(t, goal, measured)
        assert not data['valid'] and data['calculated_raw'] is None
        assert (data['target_speed_mps'], data['estimated_speed_mps']) == (goal, measured)
        assert update(t + .03)['i_raw'] == 0.
    assert not update(5.)['valid']
    assert update(8.)['valid']
    assert not update(8., v=1.1)['valid']
    assert not update(9., estimate=float('nan'))['valid']


def test_integral_stops_at_observed_raw_limit_and_unwinds_within_interval():
    controller = SpeedController(FeedforwardTable.from_document(document()), PIDConfig(0., 1500., 0.))
    for tick in range(301):
        result = controller.update(1_000_000_000 + tick * 100_000_000, 1., .5)
    assert result['calculated_raw'] == 1400 and result['saturated'] and result['i_raw'] == 200
    controller.update(31_100_000_000, 1., 1.5)
    recovered = controller.update(31_200_000_000, 1., 1.5)
    assert recovered['i_raw'] == 125 and recovered['calculated_raw'] == 1325 and not recovered['saturated']


@pytest.mark.parametrize('field,value', [
    ('input_unit','km/h'), ('output_unit','us'), ('direction','decreasing'), ('interpolation','nearest'),
    ('usable',False), ('speed_min_mps',0.), ('proposal_sha256','x'), ('extrapolation','clamp'),
    ('points',[dict(speed_mps=.5, throttle_raw=990),dict(speed_mps=.5, throttle_raw=1200)]),
    ('points',[dict(speed_mps=.5, throttle_raw=990),dict(speed_mps=2., throttle_raw=2048)]),
])
def test_calibration_contract(field, value):
    doc = document()
    doc[field] = value
    with pytest.raises(ValueError):
        FeedforwardTable.from_document(doc)


@pytest.mark.parametrize('gains', [(-1., 0., 0.), (0., float('nan'), 0.), (0., 0., float('inf'))])
def test_invalid_gains(gains):
    with pytest.raises(ValueError):
        PIDConfig(*gains)


def test_table_file_can_be_selected_replaced_or_lost_without_restarting(tmp_path):
    path = tmp_path / 'vehicle' / 'active.json'
    tables = SpeedTableFile(path)
    assert tables.refresh() and tables.table is None and not tables.refresh()
    selected = tables.save(document())
    assert tables.refresh() and tables.table == selected and not tables.refresh()
    assert tables.save(document()) == selected and not tables.refresh()
    invalid = document() | dict(output_unit='us')
    with pytest.raises(ValueError):
        tables.save(invalid)
    assert json.loads(path.read_text()) == document() and not tables.refresh()
    replacement = tmp_path / 'other'
    replacement.write_text(json.dumps(document(True)))
    replacement.replace(path)
    assert tables.refresh() and tables.table.direction == -1
    path.write_text('{')
    assert tables.refresh() and tables.table is None and tables.issue
    path.unlink()
    assert tables.refresh() and tables.table is None


def test_only_healthy_manual_receiver_readings_are_valid():
    msg = dict(header=dict(frame_id='pico'), serial='rev6', instance_id='a', rc_valid=True,
               power_ready=True, active_path=1, sbus_fresh=True, fault_bits=0, sbus_age_ms=50,
               sbus_flags=0, channels=[990, 1400] + [990] * 14)
    assert manual_reading(msg) == (1400, '')
    for key, value in [('rc_valid', False), ('active_path', 2), ('sbus_age_ms', 51), ('power_ready', False),
                       ('fault_bits', 1), ('sbus_flags', 4), ('sbus_flags', 8), ('sbus_fresh', False),
                       ('channels', []), ('serial', ''), ('header', {})]:
        raw, reason = manual_reading(msg | {key: value})
        assert raw is None and reason
