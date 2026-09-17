import logging
from functools import cached_property

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.bimanual import BimanualMixin
from lerobot.utils.decorators import check_if_not_connected

from ..robot import Robot
from ..so102_follower import SO102Follower, SO102FollowerRobotConfig
from .config_bi_so102_follower import BiSO102FollowerConfig

logger = logging.getLogger(__name__)


class BiSO102Follower(BimanualMixin, Robot):
    config_class = BiSO102FollowerConfig
    name = "bi_so102_follower"

    def __init__(self, config: BiSO102FollowerConfig):
        super().__init__(config)
        self.config = config

        self._top_level_cam_keys = set(config.cameras)
        _collisions = self._top_level_cam_keys & set(
            config.left_arm_config.cameras
        ) | self._top_level_cam_keys & set(config.right_arm_config.cameras)
        if _collisions:
            raise ValueError(
                f"Top-level camera names collide with per-arm camera names: {sorted(_collisions)}"
            )
        left_arm_cameras = {**config.left_arm_config.cameras, **config.cameras}

        left_arm_config = SO102FollowerRobotConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            disable_torque_on_disconnect=config.left_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.left_arm_config.max_relative_target,
            use_degrees=config.left_arm_config.use_degrees,
            cameras=left_arm_cameras,
        )

        right_arm_config = SO102FollowerRobotConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            disable_torque_on_disconnect=config.right_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.right_arm_config.max_relative_target,
            use_degrees=config.right_arm_config.use_degrees,
            cameras=config.right_arm_config.cameras,
        )

        self.left_arm = SO102Follower(left_arm_config)
        self.right_arm = SO102Follower(right_arm_config)

        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    @property
    def _motors_ft(self) -> dict[str, type]:
        left_arm_motors_ft = self.left_arm._motors_ft
        right_arm_motors_ft = self.right_arm._motors_ft

        return {
            **{f"left_{k}": v for k, v in left_arm_motors_ft.items()},
            **{f"right_{k}": v for k, v in right_arm_motors_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        out: dict[str, tuple] = {}
        for k, v in self.left_arm._cameras_ft.items():
            out[k if k in self._top_level_cam_keys else f"left_{k}"] = v
        for k, v in self.right_arm._cameras_ft.items():
            out[f"right_{k}"] = v
        return out

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs_dict: RobotObservation = {}

        for key, value in self.left_arm.get_observation().items():
            obs_dict[key if key in self._top_level_cam_keys else f"left_{key}"] = value

        for key, value in self.right_arm.get_observation().items():
            obs_dict[f"right_{key}"] = value

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        sent_action_left = self.left_arm.send_action(left_action)
        sent_action_right = self.right_arm.send_action(right_action)

        prefixed_sent_action_left = {f"left_{key}": value for key, value in sent_action_left.items()}
        prefixed_sent_action_right = {f"right_{key}": value for key, value in sent_action_right.items()}

        return {**prefixed_sent_action_left, **prefixed_sent_action_right}
