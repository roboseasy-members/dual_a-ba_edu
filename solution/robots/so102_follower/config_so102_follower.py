from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@dataclass
class SO102FollowerConfig:
    port: str

    disable_torque_on_disconnect: bool = True

    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    use_degrees: bool = True


@RobotConfig.register_subclass("so102_follower")
@dataclass
class SO102FollowerRobotConfig(RobotConfig, SO102FollowerConfig):
    pass
