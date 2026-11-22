"""Follow post-physics Gazebo observations and score a finite closed-loop run."""
import json
import time

from circuits import load_lap
from closed_loop_eval import CourseEvaluation


def read_samples(stream):
    """A writer may still be appending the last line."""
    while True:
        start = stream.tell()
        line = stream.readline()
        if not line.endswith("\n"):
            stream.seek(start)
            return
        yield json.loads(line)


class Trial:
    def __init__(self, evaluator):
        self.evaluator = evaluator
        self.last_sim_s = None
        self.last_progress_s = None
        self.previous_distance = 0.
        self.last_gate = (0, 0)
        self.last_gate_s = None

    def sample(self, row):
        t = self.last_sim_s = row["sim_time_s"]
        if row["first_active_ns"] is None:
            if row["actuator_state"] == "stopped":
                return self.evaluator.report("control_stopped", t, row["actuator_reason"])
            return None
        result = self.evaluator.observe(t, row["x_m"], row["y_m"], row["yaw_rad"])
        # Keep the actual coast-down in the trace following loss of authority.
        if row["actuator_state"] == "stopped":
            if row["speed_mps"] < .01 or result is not None:
                reason = row["actuator_reason"]
                state = row.get("control_state") or {}
                details = state.get("reason") if state.get("state") == "stopped" else None
                status = "command_timeout" if reason == "command_timeout" else "control_stopped"
                failure = self.evaluator.report(status, t, details or reason)
                if result is not None:
                    failure["course_outcome_during_stop"] = result["status"]
                return failure
            return None
        if result is not None:
            return result
        # Distinguish immobility from circling without reaching another sector gate.
        if self.last_progress_s is None or self.evaluator.distance >= self.previous_distance + .05:
            self.last_progress_s = t
            self.previous_distance = self.evaluator.distance
        gate = len(self.evaluator.lap_times), self.evaluator.next_gate
        if self.last_gate_s is None or gate != self.last_gate:
            self.last_gate, self.last_gate_s = gate, t
        if t - self.last_progress_s >= 30.:
            return self.evaluator.report("no_progress", t, "30シミュレーション秒で5 cmの移動を観測できません")
        if t - self.last_gate_s >= 600.:
            return self.evaluator.report("no_progress", t, "600シミュレーション秒で次の区間ゲートを通過していません")
        return None


def run_trial(children, out, scene, report, start_stream, wait_for, check_children):
    boundaries, samples, _ = load_lap(scene["course"])
    evaluator = CourseEvaluation(boundaries, samples, scene["trajectory"]["laps"])
    trial = Trial(evaluator)
    cancel = out / "closed_loop.cancel"
    trace = out / "scene/rig_motion.jsonl"
    result = None

    def check_cancelled():
        if cancel.is_file():
            raise InterruptedError(cancel.read_text().strip() or "閉ループ走行の中断が要求されました")

    try:
        start_stream(children, False, scene["course"], check_cancelled, imu_hz=report["imu_hz"])
        wait_for(lambda: trace.is_file() and trace.stat().st_size > 0, children, 45,
                 "指令走行プラグインの初期静止姿勢", check_cancelled)
        (out / "gazebo.ready").touch(exist_ok=False)
        print("計時線で停止中。保存地図での追跡と最初の有効な制御指令を待っています。", flush=True)
        with trace.open() as stream:
            last_sample_wall = time.monotonic()
            while result is None:
                check_cancelled()
                check_children(children)
                for row in read_samples(stream):
                    last_sample_wall = time.monotonic()
                    result = trial.sample(row)
                    if result is not None:
                        break
                if result is None:
                    if time.monotonic() - last_sample_wall >= 90:
                        raise TimeoutError("90実時間秒にわたりGazeboの車体姿勢が届きません")
                    time.sleep(.05)
    except (KeyboardInterrupt, InterruptedError) as exc:
        result = evaluator.report("cancelled", trial.last_sim_s, str(exc) or "割り込み")
    except (OSError, RuntimeError, ValueError) as exc:
        result = evaluator.report("error", trial.last_sim_s, str(exc))
    report["closed_loop"] = result
    target = out / "closed_loop.result.json"
    pending = target.with_suffix(".tmp")
    pending.write_text(json.dumps(result, allow_nan=False, indent=2, ensure_ascii=False) + "\n")
    pending.replace(target)
    print(f"閉ループ走行 {result['status']}  {result['laps_completed']}/{result['laps_requested']} 周  "
          f"{result['reason']}", flush=True)
    return 0 if result["status"] == "completed" else 1
