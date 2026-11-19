"""予約をローカル端末で承認し、Pico の操作権を持つ間だけ手順を実行する。"""
import argparse
import select
import signal
import sys
import termios
import time

from .board import Board
from .model import EscState, validate_steps
from .stop import LocalizationStop, VisualStop
from .store import TrialStore


class TrialCancelled(RuntimeError):
    pass


def _check_owner(reply, key):
    trial = reply['trial']
    if not trial or trial['id'] != key or trial['status'] != 'running':
        raise TrialCancelled('試験が中止されました')


def execute(store, key, steps, board, stop, *, clock=time.monotonic, sleep=time.sleep):
    """board は排他取得済み。各送信時にも Board が CH3・異常・時刻・経路を検査する。"""
    machine = EscState()
    _check_owner(store.status(), key)
    try:
        board.command(0.0, True)  # HOST 経路かつ実際の基板報告が中立になるまで待つ
        for index, step in enumerate(steps):
            _check_owner(store.status(), key)
            machine.apply(step)
            action = step['action']
            if action == 'wait_stopped':
                stop.begin()
            sample = board.feedback()
            start_sequence = sample['sequence']
            milli = round(machine.throttle * 1000)
            target_pwm = 1500 - int(milli / 2)  # firmware と同じ整数演算（0 方向への丸め）
            start = clock()
            deadline = start + step['duration_s']
            next_log = next_cancel_check = start
            last_logged = None
            observed = False
            while True:
                now = clock()
                if now >= next_cancel_check:
                    _check_owner(store.status(), key)
                    now = clock()
                    next_cancel_check = now + 0.05
                if now < deadline:
                    board.command(machine.throttle, True, reverse=machine.state == 'reverse')
                sample = board.feedback()
                now = clock()
                # S は C の ACK ではない。手順開始後に届いた HOST 経路の実出力値を照合する。
                matched = sample['sequence'] != start_sequence and sample['esc_us'] == target_pwm
                expired = now >= deadline
                if now > deadline and not observed:
                    matched = False
                evidence = stop.evidence() if action == 'wait_stopped' and matched and not expired else None
                changed = (sample['esc_us'], matched) != last_logged
                if changed or expired or evidence or now >= next_log:
                    _check_owner(store.update(key, step_index=index, esc_state=machine.state,
                                             throttle=machine.throttle, pwm_us=sample['esc_us'],
                                             requested_pwm_us=target_pwm, pwm_observed=matched,
                                             feedback_sequence=sample['sequence'], step_elapsed_s=now - start,
                                             message=f'手順 {index + 1}/{len(steps)}　{action}'), key)
                    next_log = now + 0.2
                    last_logged = (sample['esc_us'], matched)
                if observed and not matched:
                    raise RuntimeError(f'手順 {index + 1} で基板の ESC 出力が指令から変化しました')
                observed = matched
                if evidence:
                    machine.confirm_stopped()
                    _check_owner(store.update(key, esc_state=machine.state, stop_evidence=evidence,
                                              message=evidence), key)
                    break
                if expired:
                    if not observed:
                        raise RuntimeError(f'手順 {index + 1} の {target_pwm} µs を期限内に基板状態で確認できません')
                    if action == 'wait_stopped':
                        raise RuntimeError('停止確認が期限に達しました')
                    break
                sleep(min(0.02, max(0., deadline - clock())))
    finally:
        board.command(0.0, False)


def run_reserved(store, key, port, stop, *, board_factory=Board):
    trial = store.claim(key)['trial']
    try:
        steps = validate_steps(trial['steps'])
        with stop:
            with board_factory(port) as board:
                execute(store, key, steps, board, stop)
    except (KeyboardInterrupt, TrialCancelled):
        status, message = 'cancelled', '試験を中止し、HOST 出力の解除処理を行いました'
    except Exception as exc:
        status, message = 'failed', f'試験を終了しました。{exc}'
    else:
        status, message = 'succeeded', '手順完了。中立の解除指令を送信しました'
    # 操作権を返した後の PWM・走行状態はこのセッションでは観測できない。
    return store.finish(key, status, esc_state='unknown', throttle=None, pwm_us=None,
                        requested_pwm_us=None, pwm_observed=None, message=message)['trial']


def confirm_trial(store, key, stream):
    print('開始する試験 ID を入力してください > ', end='', flush=True)
    while True:
        trial = store.status()['trial']
        if not trial or trial['id'] != key or trial['status'] != 'queued':
            return False
        if select.select([stream], [], [], 0.2)[0]:
            return stream.readline().strip() == key


def main(argv=None):
    parser = argparse.ArgumentParser(description='Jetson 端末で予約済みの ESC 試験を確認・実行します')
    parser.add_argument('--port', required=True, help='Pico Rev6 CDC0 (if00) のデバイスパス')
    parser.add_argument('--root', help='Jetson の試験保存先')
    parser.add_argument('--stop-source', choices=('visual', 'localization'), default='visual',
                        help='停止確認の方法。既定は車両を目視して STOP を入力')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.exit(2, '開始確認には Jetson の対話端末が必要です\n')
    store = TrialStore(args.root)
    try:
        trial = store.status()['trial']
        if not trial or trial['status'] != 'queued':
            raise ValueError('予約された試験がありません')
        steps = validate_steps(trial['steps'])
        print(f"ESC 試験 {trial['id']}  ポート {args.port}  停止確認 {args.stop_source}")
        for index, step in enumerate(steps, 1):
            level = f" {step['level']:.3g}" if 'level' in step else ''
            seconds = '期限' if step['action'] == 'wait_stopped' else '保持'
            print(f"  {index:2d}. {step['action']}{level}　{seconds} {step['duration_s']:g} 秒")
        print('車両の静止、ESC の中立、プロポの HOST 許可を確認してください。Ctrl+C で中止できます。')
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
        if not confirm_trial(store, trial['id'], sys.stdin):
            print('試験は開始していません')
            return 1
        stop = VisualStop() if args.stop_source == 'visual' else LocalizationStop()
        previous = signal.getsignal(signal.SIGTERM)
        def terminate(*_):
            raise TrialCancelled('試験を中止しました')
        signal.signal(signal.SIGTERM, terminate)
        try:
            result = run_reserved(store, trial['id'], args.port, stop)
        finally:
            signal.signal(signal.SIGTERM, previous)
        print(result['message'])
        return 0 if result['status'] == 'succeeded' else 1
    except (KeyboardInterrupt, EOFError, OSError, ValueError, RuntimeError) as exc:
        print(f'試験を終了しました。{exc}', file=sys.stderr)
        return 1
