from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import sys

import pytest

from atd_esc_trials.store import TrialStore


PLAN = [{'action': 'forward', 'level': .1, 'duration_s': .2}]


def test_cannot_start_cancelled_or_replaced_plan(tmp_path):
    store = TrialStore(tmp_path)
    def prepare(_):
        try:
            return TrialStore(tmp_path).prepare(PLAN)['trial']['id']
        except ValueError:
            return None
    with ThreadPoolExecutor(2) as pool:
        ids = [key for key in pool.map(prepare, (0, 1)) if key]
    assert len(ids) == 1
    first = ids[0]
    assert store.cancel(first)['trial']['status'] == 'cancelled'
    with pytest.raises(ValueError):
        store.claim(first)
    second = store.prepare(PLAN)['trial']['id']
    with pytest.raises(ValueError):
        store.cancel(first)
    assert store.status()['trial']['id'] == second
    assert json.loads((tmp_path/(first+'.json')).read_text())['status'] == 'cancelled'


def test_owner_and_cancel_during_final_step(tmp_path, monkeypatch):
    runner, ui = TrialStore(tmp_path), TrialStore(tmp_path)
    key = ui.prepare(PLAN)['trial']['id']
    runner.claim(key)
    assert ui.status()['trial']['status'] == 'running'
    with pytest.raises(ValueError):
        ui.update(key, pwm_us=1500)
    with pytest.raises(ValueError):
        runner.update(key, steps=[])
    runner.update(key, step_index=0, pwm_us=1450, throttle=.1, feedback_sequence=10)
    active = ui.status()['trial']
    assert active['feedback_stale'] is False
    with monkeypatch.context() as patch:
        patch.setattr('atd_esc_trials.store.time.time', lambda: active['feedback_at'] + 3)
        assert ui.status()['trial']['feedback_stale'] is True
    assert ui.cancel(key)['trial']['status'] == 'cancel_requested'
    assert runner.update(key, throttle=0)['trial']['status'] == 'cancel_requested'
    assert runner.finish(key, 'succeeded')['trial']['status'] == 'cancelled'
    history = ui.status()['trial']['state_history']
    assert history[-1]['status'] == 'cancelled' and any(row['pwm_us'] == 1450 for row in history)
    events = [json.loads(line) for line in (tmp_path/(key+'.jsonl')).read_text().splitlines()]
    assert any(event.get('pwm_us') == 1450 for event in events)
    assert events[-1]['status'] == 'cancelled'


def test_abrupt_process_exit_never_resumes_previous_trial(tmp_path):
    import atd_esc_trials.store as module
    from pathlib import Path
    store = TrialStore(tmp_path)
    key = store.prepare(PLAN)['trial']['id']
    env = dict(os.environ, PYTHONPATH=str(Path(module.__file__).resolve().parents[1]))
    subprocess.run([sys.executable, '-c', 'import os,sys; from atd_esc_trials.store import TrialStore; '
        'r=TrialStore(sys.argv[1]); r.claim(sys.argv[2]); os._exit(0)', str(tmp_path), key], check=True, env=env)
    assert store.status()['trial']['status'] == 'failed'
    assert store.status()['trial']['esc_state'] == 'unknown'
    assert store.prepare(PLAN)['trial']['id'] != key
