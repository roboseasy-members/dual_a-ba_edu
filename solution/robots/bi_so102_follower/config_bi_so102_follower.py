from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from ..so102_follower.config_so102_follower import SO102FollowerConfig


@RobotConfig.register_subclass("bi_so102_follower")
@dataclass
class BiSO102FollowerConfig(RobotConfig):
    left_arm_config: SO102FollowerConfig
    right_arm_config: SO102FollowerConfig

    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@dataclass
class BiSO102HostConfig:
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    connection_time_s: int = 0

    watchdog_timeout_ms: int = 500

    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("bi_so102_client")
@dataclass
class BiSO102ClientConfig(RobotConfig):
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
