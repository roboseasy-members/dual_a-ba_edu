from dataclasses import dataclass

from ..config import TeleoperatorConfig
from ..so102_leader.config_so102_leader import SO102LeaderConfig


@TeleoperatorConfig.register_subclass("bi_so102_leader")
@dataclass
class BiSO102LeaderConfig(TeleoperatorConfig):
    left_arm_config: SO102LeaderConfig
    right_arm_config: SO102LeaderConfig
