"""작업 설명을 반복 입력받는 추론 명령의 설정."""

import math
from dataclasses import dataclass, field

from lerobot.rollout.configs import (
    BaseStrategyConfig,
    RolloutConfig,
    RolloutStrategyConfig,
)


@RolloutStrategyConfig.register_subclass("edu_interactive")
@dataclass
class InteractiveStrategyConfig(BaseStrategyConfig):
    """작업 제한 시간과 초기 자세 복귀 판정을 설정한다."""

    task_timeout: float = 60.0
    success_detection: bool = True
    success_leave_thresh: float = 15.0
    success_return_thresh: float = 6.0
    success_settle_eps: float = 1.0
    success_settle_ticks: int = 15
    inference_stop_timeout_s: float = 30.0

    def __post_init__(self) -> None:
        for name in (
            "task_timeout",
            "inference_stop_timeout_s",
            "success_leave_thresh",
            "success_return_thresh",
            "success_settle_eps",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name}은 유한한 양수여야 합니다.")
        if self.success_settle_ticks < 1:
            raise ValueError("success_settle_ticks는 1 이상이어야 합니다.")
        if self.success_leave_thresh <= self.success_return_thresh:
            raise ValueError("이탈 기준은 복귀 기준보다 커야 합니다.")


@dataclass
class InteractiveRolloutConfig(RolloutConfig):
    """LeRobot 설정을 재사용하되 녹화 없이 interactive만 실행한다."""

    strategy: RolloutStrategyConfig = field(default_factory=InteractiveStrategyConfig)

    def __post_init__(self) -> None:
        if not isinstance(self.strategy, InteractiveStrategyConfig):
            raise ValueError("교육판 interactive에는 edu_interactive 설정이 필요합니다.")
        if self.dataset is not None or self.teleop is not None:
            raise ValueError("interactive는 dataset·teleop 없이 실행합니다.")
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("fps는 유한한 양수여야 합니다.")
        if not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError("duration은 유한한 0 이상 값이어야 합니다.")
        if self.interpolation_multiplier < 1:
            raise ValueError("interpolation_multiplier는 1 이상이어야 합니다.")
        super().__post_init__()
