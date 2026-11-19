import os
from types import SimpleNamespace

import pytest

from atd_esc_trials.model import validate_steps
from atd_esc_trials.runner import confirm_trial, execute, run_reserved, TrialCancelled
from atd_esc_trials.store import TrialStore
from atd_esc_trials.stop import StopEstimate


def step(action, level=None, seconds=.08):
    item = dict(action=action, duration_s=seconds)
    if level is not None:
        item['level'] = level
    return item


def plan():
    return [step('forward', .1), step('brake', .2), step('wait_stopped', seconds=.5),
            step('neutral'), step('reverse', .3), step('forward', .4)]


class Clock:
    now = 0.

    def sleep(self, seconds):
        self.now += seconds


class Board:
    def __init__(self):
        self.writes = []
        self.sequence = 0
        self.fault = False
        self.stuck_brake = False

    def command(self, throttle, enabled, *, reverse=False):
        self.writes.append((throttle, enabled, reverse))
        if enabled and self.fault:
            raise RuntimeError('操作権が変化')

    def feedback(self):
        self.sequence += 1
        pulse = 1500 - round(500 * self.writes[-1][0])
        return {'sequence': self.sequence, 'esc_us': 1600 if self.stuck_brake else pulse}


class Store:
    def __init__(self):
        self.events = []
        self.state = 'running'

    def status(self):
        return {'trial': {'id': 'a', 'status': self.state}}

    def update(self, key, **fields):
        self.events.append(fields)
        return self.status()


class Observer:
    def __init__(self, observe):
        self.observe = observe
        self.began = False

    def begin(self):
        self.began = True

    def evidence(self):
        assert self.began
        return self.observe()


def drive(board, store, observe, steps=None):
    clock = Clock()
    execute(store, 'a', validate_steps(steps or plan()), board, Observer(observe),
            clock=lambda: clock.now, sleep=clock.sleep)


def test_forward_brake_stop_neutral_reverse_forward():
    board, store = Board(), Store()
    drive(board, store, lambda: '目視停止')
    changes = [record for i, record in enumerate(board.writes) if not i or record != board.writes[i - 1]]
    assert changes == [(0., True, False), (.1, True, False), (-.2, True, False),
                       (0., True, False), (-.3, True, True), (.4, True, False), (0., False, False)]
    states = [event['esc_state'] for event in store.events]
    assert [state for i, state in enumerate(states) if not i or states[i-1] != state] == [
        'forward', 'braking', 'wait_stopped', 'stopped_braking', 'neutral_after_stop', 'reverse', 'forward']
    assert any(event.get('stop_evidence') == '目視停止' for event in store.events)


@pytest.mark.parametrize('invalid', [
    [step('reverse', .1)], [step('brake', .1)], plan()[:2] + [step('neutral'), step('reverse', .1)],
    plan()[:3] + [step('reverse', .1)], [step('wait_stopped')], [step('forward', True)],
    [step('neutral', .1)], [step('forward', 1.1)], [step('forward', .00001)],
    [step('neutral', seconds=float('nan'))], [step('neutral', seconds=0)], []])
def test_reject_invalid_plan(invalid):
    with pytest.raises(ValueError):
        validate_steps(invalid)


@pytest.mark.parametrize('fault', ['timeout', 'takeover', 'cancel', 'no_neutral'])
def test_stop_failure_never_reverses(fault):
    board, store = Board(), Store()
    def observe():
        if fault == 'takeover':
            board.fault = True
        if fault == 'cancel':
            store.state = 'cancel_requested'
        if fault == 'no_neutral':
            board.stuck_brake = True
            return '目視停止'
        return None
    with pytest.raises(TrialCancelled if fault == 'cancel' else RuntimeError):
        drive(board, store, observe)
    assert not any(reverse for _, _, reverse in board.writes)
    assert board.writes[-1] == (0., False, False)
    assert board.writes.count((-.2, True, False)) > 2


def sample(tick, speed=.01, epoch=1, state='tracking'):
    stamp = SimpleNamespace(sec=10 + tick // 10, nanosec=(tick % 10) * 100_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), session_id='s', map_id='m',
                           epoch=epoch, speed_mps=speed, yaw_rate_rps=.01, state=state)


def test_stop_estimate_requires_fresh_consecutive_tracking():
    estimator = StopEstimate()
    estimator.begin(10_000_000_000)
    def push(tick, **fields):
        return estimator.observe(sample(tick, **fields), 10_000_000_000 + tick * 100_000_000, tick / 10)
    assert not push(0)
    assert not push(1)
    assert not push(2, state='lost')
    assert not push(3)
    assert not push(4, speed=1.0)
    assert not push(5)
    assert not push(6, epoch=2)
    assert not push(7, epoch=2)
    assert not push(7, epoch=2)  # 同じ速度・stamp の再掲
    for tick in (12, 13, 14, 15, 16):
        assert not push(tick, epoch=2)
    assert '停止を推定' in push(17, epoch=2)
    assert not estimator.observe(sample(17), 20_000_000_000, 10)  # 古い停止位置
    estimator.begin(20_000_000_000)
    assert not estimator.observe(sample(99), 20_000_000_000, 10)  # wait 開始前


def test_no_report_for_short_command_fails_before_next_step(tmp_path):
    class UnobservedBoard(Board):
        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def feedback(self):
            return {'sequence': 1, 'esc_us': 1500}
    from contextlib import nullcontext
    board = UnobservedBoard()
    store = TrialStore(tmp_path)
    key = store.prepare([step('forward', .2, .02), step('brake', .1)])['trial']['id']
    trial = run_reserved(store, key, '', nullcontext(), board_factory=lambda _: board)
    assert trial['status'] == 'failed' and '1400' in trial['message']
    assert trial['step_index'] == 0 and trial['pwm_observed'] is None
    assert board.writes[-1] == (0., False, False)
    assert not any(throttle < 0 for throttle, *_ in board.writes)


def test_cancel_confirmation_without_typing_old_id(tmp_path):
    import threading
    store = TrialStore(tmp_path)
    key = store.prepare([step('neutral')])['trial']['id']
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as reader, os.fdopen(write_fd, 'w'):
        def replace():
            store.cancel(key)
            store.prepare([step('forward', .1)])
        timer = threading.Timer(.03, replace)
        timer.start()
        try:
            assert not confirm_trial(store, key, reader)
        finally:
            timer.join(1)
