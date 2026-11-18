"""実際の校正 CSV の文字列・列の形で候補と根拠を検査する。"""
import csv
import io

import pytest

from atd_processing.vehicle_limits import derive_candidates


class Recording:
    def __init__(self):
        self.rows = []
        self.stamp = 1000
        self.episode = 0

    def hold(self, speed=0.8, ax=0.0, yaw=0.0, throttle=1200, *, count=7, dt=0.1,
             valid=True, sustained=True, segment=0):
        for i in range(count):
            vx = speed[i] if isinstance(speed, list) else speed
            ch2 = throttle[i] if isinstance(throttle, list) else throttle
            self.rows.append(dict(stamp=self.stamp, valid=valid, motion_valid=valid, sustained=sustained,
                                  segment=segment, episode_id=self.episode, steering_raw=1000, throttle_raw=ch2,
                                  vo_vx_mps=vx, vo_ax_mps2=ax, vo_yaw_rate_rps=yaw,
                                  turn_regime='left' if yaw > .1 else 'right' if yaw < -.1 else 'straight',
                                  longitudinal_regime='accelerating' if ax > .2 else 'decelerating' if ax < -.2 else 'steady'))
            self.stamp += dt
        self.episode += 1

    def derive(self, **counts):
        stream = io.StringIO()
        writer = csv.DictWriter(stream, self.rows[0].keys())
        writer.writeheader()
        writer.writerows(self.rows)
        stream.seek(0)
        summary = dict(schema_version=1, analysis=dict(thresholds=dict(min_speed_mps=.15, max_gap_s=.2,
            min_regime_duration_s=.5, raw_stability_tolerance=8, steady_accel_mps2=.2, turn_rate_rps=.1),
            binning=dict(raw_bin_width=64)), timeseries_samples=len(self.rows),
            eligible_samples=sum(row['valid'] for row in self.rows))
        return derive_candidates(summary | counts, csv.DictReader(stream))


def test_repeatable_limits_from_maneuvers_and_weaker_turn():
    data = Recording()
    for left, right, acceleration, deceleration in [(1.6, -.8, 1.2, -.7), (1.2, -.64, 1.0, -.9)]:
        data.hold(yaw=left)
        data.hold(yaw=right)
        data.hold(ax=acceleration)
        data.hold(ax=deceleration)
    # Faster steady turns contribute to lateral acceleration, not low-speed turning radius.
    data.hold(speed=2, yaw=1.5)
    data.hold(speed=2, yaw=-1.0)
    data.hold(speed=2, yaw=1.2)
    data.hold(speed=2, yaw=-.9)
    limits = data.derive()['limits']
    assert limits['curvature_limit']['value'] == pytest.approx(.8 * (.64 / .8))
    assert limits['lateral_accel']['value'] == pytest.approx(.8 * (2 * .9))
    assert limits['accel']['value'] == pytest.approx(.8)
    assert limits['decel']['value'] == pytest.approx(.8 * .7)
    assert {e['side'] for e in limits['curvature_limit']['evidence']} == {'left', 'right'}
    assert [(ep['episode_id'], ep['samples']) for ep in limits['accel']['evidence']] == [(2, 7), (6, 7)]
    assert all(e['end_s'] > e['start_s'] for e in limits['accel']['evidence'])
    assert data.derive()['method']['limit_fraction'] == .8


def test_command_jitter_and_invalid_gaps_do_not_invent_repeat_trials():
    data = Recording()
    data.hold(ax=1, count=100)
    data.hold(ax=2, throttle=1250)
    data.hold(yaw=.5, count=1, sustained=False)
    data.hold(valid=False)
    data.hold(ax=1.1, segment=1)
    data.hold(yaw=-1)
    data.hold(ax=-1, dt=.4)
    data.hold(ax=-1, count=3)
    data.hold(speed=-1, yaw=1)
    data.hold(speed=-1, ax=1)
    result = data.derive()
    assert result['limits']['accel']['value'] is None
    assert result['limits']['accel']['episodes'] == 1
    assert result['limits']['curvature_limit']['value'] is None
    assert result['limits']['decel']['episodes'] == 0
    assert not result['feedforward']['usable']


def test_feedforward_inverse_raw_observed_forward_steady_only():
    data = Recording()
    data.hold(speed=1, throttle=1600)
    data.hold(speed=1, throttle=1600, count=40)
    data.hold(speed=1.4, throttle=1450)
    data.hold(speed=1.4, yaw=-1.1, throttle=1100)
    data.hold(speed=-4, throttle=900)
    data.hold(speed=.02, throttle=990)
    data.hold(speed=2, ax=1, throttle=1200)
    table = data.derive()['feedforward']
    assert table['usable'] and table['direction'] == 'decreasing'
    assert [(p['speed_mps'], p['throttle_raw']) for p in table['points']] == [(1, 1600), (1.4, 1450)]
    assert table['input_unit'] == 'm/s' and table['output_unit'] == 'sbus_raw_ch2'
    assert (table['speed_min_mps'], table['speed_max_mps'], table['extrapolation']) == (1, 1.4, 'none')
    assert table['points'][0]['samples'] == 47


@pytest.mark.parametrize('levels', [
    [(1, 1000), (2, 1200), (1.2, 1350)],
    [([1., 1., 1., 1., 1.01, 1.02, 1.03], 1000), (1.02, 1200)],
    [(1, [1020, 1021, 1021, 1022, 1023, 1024, 1025]), (2, [1023, 1024, 1025, 1026, 1027, 1027, 1028])],
    [(1, 1000)],
    [([.8, .84, .88, .92, .96, 1.0, 1.04], 1000), (2., 1200)],
])
def test_insufficient_or_ambiguous_ff_keeps_observations_but_no_interpolation_range(levels):
    data = Recording()
    for speed, throttle in levels:
        data.hold(speed=speed, throttle=throttle)
    table = data.derive()['feedforward']
    assert table['points'] and not table['usable']
    assert table['speed_min_mps'] is None and table['speed_max_mps'] is None and table['reason']


def test_reversed_sweeps_and_sample_counts_do_not_weight_ff_by_fps():
    data = Recording()
    data.hold(speed=1, throttle=1000, count=70)
    data.hold(speed=1.1, throttle=1002)
    data.hold(speed=2, throttle=1200)
    table = data.derive()['feedforward']
    assert table['direction'] == 'increasing'
    assert table['points'][0]['speed_mps'] == pytest.approx(1.05)
    assert table['points'][0]['throttle_raw'] == 1001


def test_zero_valid_samples_do_not_produce_limits_and_tampered_csv_is_rejected():
    data = Recording()
    data.hold(valid=False)
    result = data.derive()
    assert all(limit['value'] is None and limit['reason'] for limit in result['limits'].values())
    assert result['feedforward']['points'] == []
    with pytest.raises(ValueError, match='件数'):
        data.derive(eligible_samples=1)
    data.hold()
    data.rows[-1]['vo_vx_mps'] = 'nan'
    with pytest.raises(ValueError, match='有効試料'):
        data.derive()
