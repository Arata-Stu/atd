"""API から渡された固定 action のみを処理する独立 supervisor。"""
from __future__ import annotations

import fcntl
import signal
import sys
import time

from atd_processing.geometry import generate
from .jobs import ACTIONS, Cancelled, Jobs, Task
from .store import Store, read_json


def execute(store, task, action, payload):
    if action == 'map_duplicate':
        from .map_library import duplicate_map
        return duplicate_map(store, task, payload)
    if action == 'simulation_benchmark':
        from .simulation_benchmark import execute as benchmark
        return benchmark(store, task, payload)
    if action == 'generate':
        document = store.load_map(payload['id'])
        if document['revision'] != payload['revision']:
            raise ValueError('地図 revision が変わりました')
        line = generate(document, payload['kind'], payload.get('settings', {}), task.check)
        task.check()
        document = store.save_line(payload['id'], payload['revision'], payload['kind'], line)
        return dict(map=document['id'], revision=document['revision'], kind=payload['kind'])
    if action == 'map_build':
        from .offline import map_build
        return map_build(store, task, payload)
    if action == 'localization_validation':
        from .localization_validation import execute as validate
        return validate(store, task, payload)
    if action == 'planning_replay':
        from .planning_replay import execute as plan
        return plan(store, task, payload)
    if action == 'speed_replay':
        from .speed_replay import execute as compare_speed
        return compare_speed(store, task, payload)
    if action == 'manual_drive_replay':
        from .manual_drive_replay import execute as read_manual_drive
        return read_manual_drive(store, task, payload)
    if action == 'stereo_bag_diagnostics':
        from .stereo_bag_diagnostics import execute as diagnose_stereo
        return diagnose_stereo(store, task, payload)
    if action == 'manual_drive_vo':
        from .manual_drive_replay import execute_vo
        return execute_vo(store, task, payload)
    if action == 'calibration':
        from .calibration import execute as calibrate
        return calibrate(store, task, payload)
    if action == 'vgl':
        from .offline import vgl_build
        return vgl_build(store, task, payload)
    if action == 'capture':
        from .capture import capture
        return capture(store, task, payload)
    if action == 'pull':
        from .transfers import pull
        return pull(store, task, payload['connection'], payload['relative'])
    if action == 'push':
        from .transfers import push
        return push(store, task, payload['connection'], payload['id'])
    raise ValueError('許可されていない処理です')


def main():
    from pathlib import Path
    request = read_json(sys.argv[1])
    if request['action'] not in ACTIONS:
        raise ValueError('許可されていない処理です')
    store = Store(request['record_root'], request['result_root'])
    jobs = Jobs(store)
    key = request['job_id']
    if Path(sys.argv[1]).resolve() != jobs.root / (key + '.request.json'):
        raise ValueError('worker request のパスが一致しません')
    with (jobs.root / (key + '.lease')).open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        task = Task(jobs, key)
        def stop(_signum, _frame):
            raise Cancelled('worker に停止要求が届きました')
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            task.check()
            result = execute(store, task, request['action'], request['payload'])
            task.state.update(status='succeeded', result=result, message='完了しました')
        except Cancelled as error:
            task.state.update(status='cancelled', message=str(error))
        except BaseException as error:
            task.state.update(status='failed', message=str(error), diagnostic=getattr(error, 'diagnostic', None))
            import traceback
            traceback.print_exc()
        finally:
            task.state['finished'] = time.time()
            task.persist()


if __name__ == '__main__':
    main()
