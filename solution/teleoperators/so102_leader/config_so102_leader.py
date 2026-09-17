from dataclasses import dataclass

from ..config import TeleoperatorConfig


@dataclass
class SO102LeaderConfig:
    port: str

    use_degrees: bool = True


@TeleoperatorConfig.register_subclass("so102_leader")
@dataclass
class SO102LeaderTeleopConfig(TeleoperatorConfig, SO102LeaderConfig):
    pass
