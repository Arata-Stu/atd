import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from atd_control.speed import FeedforwardTable
from atd_vehicle_interface.auto_calibration import AutoCalibration, load_calibration


def feedforward(increasing=False):
    return dict(schema='atd.throttle_feedforward.v1', usable=True, input_unit='m/s', output_unit='sbus_raw_ch2',
                interpolation='linear', extrapolation='none', direction='increasing' if increasing else 'decreasing',
                run_id='measured', proposal_sha256='a' * 64, speed_min_mps=.3, speed_max_mps=.7,
                points=[dict(speed_mps=v, throttle_raw=count) for v, count in
                        zip((.3, .7), (1070, 1240) if increasing else (930, 800))])


def profile(increasing=False):
    raw = (992, 1270) if increasing else (712, 992)
    esc = (1500, 1360) if increasing else (1360, 1500)
    return dict(schema='atd.auto_drive.v1', brake_level=.13, brake_hold_s=.4,
                steering=[dict(angle_deg=-18., servo_us=1780), dict(angle_deg=0., servo_us=1510),
                          dict(angle_deg=21., servo_us=1200)],
                forward_throttle=[dict(ch2_raw=r, esc_us=t) for r, t in zip(raw, esc)])


def status(table, **change):
    ff = table.interpolate(.5)
    return SimpleNamespace(**(dict(valid=True, has_speeds=True, calibration_run_id=table.run_id,
        proposal_sha256=table.proposal_sha256, target_speed_mps=.5, feedforward_raw=ff, calculated_raw=ff)
        | change))


@pytest.mark.parametrize('increasing', [True, False])
def test_measured_raw_polarity_and_asymmetric_steering(increasing, tmp_path):
    doc = feedforward(increasing)
    ff_path, cal_path = tmp_path / 'throttle.json', tmp_path / 'auto.yaml'
    ff_path.write_text(json.dumps(doc))
    cal_path.write_text(yaml.safe_dump(profile(increasing)))
    cal = load_calibration(cal_path, ff_path)
    msg = status(cal.table)
    for angle, servo in [(-18., 1780.), (0., 1510.), (10.5, 1355.), (21., 1200.)]:
        steer, throttle = cal.commands(math.radians(angle), msg, max_speed_mps=.5)
        assert 1500 + 500 * steer == pytest.approx(servo)
        frac = ((msg.calculated_raw - 992) / (1270-992) if increasing
                else (992 - msg.calculated_raw) / (992-712))
        assert 1500 - 500 * throttle == pytest.approx(1500 - frac * 140)
    assert (cal.brake_level, cal.brake_hold_s) == (.13, .4)
    for bad in [dict(valid=False), dict(has_speeds=False), dict(calibration_run_id='other'),
                dict(proposal_sha256='b'*64), dict(feedforward_raw=0.), dict(target_speed_mps=-1.),
                dict(calculated_raw=float('nan')), dict(calculated_raw=-1)]:
        with pytest.raises(ValueError):
            cal.commands(0., status(cal.table, **bad), max_speed_mps=.5)
    with pytest.raises(ValueError):
        cal.commands(math.radians(22), msg, max_speed_mps=.5)
    # 前進側に大きい PID 補正があっても、上限速度での定常スロットルまで。
    assert cal.commands(0., status(cal.table, calculated_raw=cal.table.raw[-1]), max_speed_mps=.5) == (
        cal.commands(0., msg, max_speed_mps=.5))
    with pytest.raises(ValueError):
        cal.commands(0., msg, max_speed_mps=.4)


def test_incomplete_ranges_wrong_polarity_and_unmeasured_brake_are_rejected():
    ff = FeedforwardTable.from_document(feedforward())
    for changes in [dict(forward_throttle=profile(True)['forward_throttle']),
                    dict(steering=[dict(angle_deg=0, servo_us=1500), dict(angle_deg=15, servo_us=1800)]),
                    dict(brake_level=.001), dict(brake_level=2), dict(brake_hold_s=float('nan')),
                    dict(brake_hold_s=.09), dict(brake_hold_s=2.1)]:
        with pytest.raises(ValueError):
            AutoCalibration.from_document(profile() | changes, ff)
    example = Path(__file__).resolve().parents[1] / 'config/auto_drive.example.yaml'
    with pytest.raises(ValueError):
        AutoCalibration.from_document(yaml.safe_load(example.read_text()), ff)
    partial = [dict(ch2_raw=850, esc_us=1400), dict(ch2_raw=992, esc_us=1500)]
    cal = AutoCalibration.from_document(profile() | dict(forward_throttle=partial), ff)
    with pytest.raises(ValueError):
        cal.commands(0., status(ff, calculated_raw=800.), max_speed_mps=.7)
