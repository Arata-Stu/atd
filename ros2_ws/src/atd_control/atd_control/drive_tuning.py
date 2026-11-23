"""実車の planner・横制御・速度制御で共有する走行中の設定。"""
from dataclasses import asdict, dataclass
import math

from atd_planning.core import LINE_KEYS
from .core import LookaheadConfig
from .speed import PIDConfig


@dataclass(frozen=True)
class DriveTuning:
    line: str
    max_speed_mps: float
    lookahead: LookaheadConfig
    pid: PIDConfig

    @classmethod
    def from_dict(cls, data):
        line, speed = data['line'], data['max_speed_mps']
        if line not in LINE_KEYS or type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            raise ValueError('保存した走行ラインと正の速度上限 [m/s] を指定してください')
        lookahead = LookaheadConfig(**{k: data[k] for k in asdict(LookaheadConfig())})
        pid = PIDConfig(**{k: data['speed_' + k] for k in asdict(PIDConfig())})
        return cls(line, speed, lookahead, pid)

    def values(self):
        return dict(line=self.line, max_speed_mps=self.max_speed_mps, **asdict(self.lookahead),
                    **{'speed_' + k: v for k, v in asdict(self.pid).items()})


DEFAULT_DRIVE_TUNING = DriveTuning('raceline', .5, LookaheadConfig(), PIDConfig())
