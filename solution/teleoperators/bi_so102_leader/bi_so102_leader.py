import logging
from functools import cached_property

from lerobot.types import RobotAction
from lerobot.utils.bimanual import BimanualMixin
from lerobot.utils.decorators import check_if_not_connected

from ..so102_leader import SO102Leader, SO102LeaderTeleopConfig
from ..teleoperator import Teleoperator
from .config_bi_so102_leader import BiSO102LeaderConfig

logger = logging.getLogger(__name__)


class BiSO102Leader(BimanualMixin, Teleoperator):
    config_class = BiSO102LeaderConfig
    name = "bi_so102_leader"

    def __init__(self, config: BiSO102LeaderConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = SO102LeaderTeleopConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            use_degrees=config.left_arm_config.use_degrees,
        )

        right_arm_config = SO102LeaderTeleopConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            use_degrees=config.right_arm_config.use_degrees,
        )

        self.left_arm = SO102Leader(left_arm_config)
        self.right_arm = SO102Leader(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        left_arm_features = self.left_arm.action_features
        right_arm_features = self.right_arm.action_features

        return {
            **{f"left_{k}": v for k, v in left_arm_features.items()},
            **{f"right_{k}": v for k, v in right_arm_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        action_dict = {}

        left_action = self.left_arm.get_action()
        action_dict.update({f"left_{key}": value for key, value in left_action.items()})

        right_action = self.right_arm.get_action()
        action_dict.update({f"right_{key}": value for key, value in right_action.items()})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError
