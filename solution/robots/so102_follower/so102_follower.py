import logging
import time
from dataclasses import replace
from functools import cached_property
from typing import Final

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_so102_follower import SO102FollowerRobotConfig

logger = logging.getLogger(__name__)

BUS_NUM_RETRY: Final = 2
# 미러 모드에서 부호를 뒤집는 관절 (모터 1, 5, 6)
MIRROR_MOTORS: Final = ("shoulder_pan", "wrist_yaw", "wrist_roll")


class MirrorFeetechMotorsBus(FeetechMotorsBus):
    """도 단위에서도 MIRROR_MOTORS의 drive_mode를 적용하는 Feetech 버스 (미러 모드 전용).

    lerobot은 -100~100, 0~100 단위에서만 drive_mode로 부호를 뒤집고 도 단위에서는 무시한다.
    여기서는 도 단위도 같은 기준(캘리브레이션 범위 중앙)으로 뒤집는다. 대상을 MIRROR_MOTORS로
    한정해, 파일에 다른 관절의 drive_mode가 1로 남아 있어도 미러 모드가 그 관절을 바꾸지 않게 한다.
    """

    def _is_reversed_degrees(self, id_: int) -> bool:
        motor = self._id_to_name(id_)
        calibration = self.calibration.get(motor)
        return (
            motor in MIRROR_MOTORS
            and calibration is not None
            and bool(calibration.drive_mode)
            and self.motors[motor].norm_mode is MotorNormMode.DEGREES
        )

    def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
        normalized_values = super()._normalize(ids_values)
        return {
            id_: -val if self._is_reversed_degrees(id_) else val for id_, val in normalized_values.items()
        }

    def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
        ids_values = {id_: -val if self._is_reversed_degrees(id_) else val for id_, val in ids_values.items()}
        return super()._unnormalize(ids_values)


class SO102Follower(Robot):
    config_class = SO102FollowerRobotConfig
    name = "so102_follower"

    def __init__(self, config: SO102FollowerRobotConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        bus_class = MirrorFeetechMotorsBus if config.mirror_mode else FeetechMotorsBus
        self.bus = bus_class(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_yaw": Motor(5, "sts3215", norm_mode_body),
                "wrist_roll": Motor(6, "sts3215", norm_mode_body),
                "gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam in self.cameras:
            if getattr(self.cameras[cam], "use_rgb", True):
                features[cam] = (self.cameras[cam].height, self.cameras[cam].width, 3)
            if getattr(self.cameras[cam], "use_depth", False):
                features[f"{cam}_depth"] = (self.cameras[cam].height, self.cameras[cam].width, 1)
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        if self.config.mirror_mode:
            self._apply_mirror_drive_mode()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @check_if_not_connected
    def disconnect(self):
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)

                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)

    def _apply_mirror_drive_mode(self) -> None:
        """MIRROR_MOTORS의 drive_mode를 1로 바꾼 복사본으로 버스 캘리브레이션을 교체한다.

        self.calibration(파일에 저장되는 원본)은 건드리지 않으므로 캘리브레이션 파일은 바뀌지 않는다.
        """
        self.bus.calibration = {
            motor: replace(calibration, drive_mode=1) if motor in MIRROR_MOTORS else calibration
            for motor, calibration in self.bus.calibration.items()
        }
        logger.info(
            f"{self} mirror mode: drive_mode=1 on {', '.join(MIRROR_MOTORS)} (calibration file unchanged)"
        )

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.bus.motors if motor != full_turn_motor]
        print(
            f"Move all joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        range_mins[full_turn_motor] = 0
        range_maxes[full_turn_motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position", num_retry=BUS_NUM_RETRY)
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                start = time.perf_counter()
                obs_dict[cam_key] = cam.read_latest()
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

            if getattr(cam, "use_depth", False):
                start = time.perf_counter()
                obs_dict[f"{cam_key}_depth"] = cam.read_latest_depth()
                dt_ms = (time.perf_counter() - start) * 1e3
                logger.debug(f"{self} read {cam_key} depth: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position", num_retry=BUS_NUM_RETRY)
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        self.bus.sync_write("Goal_Position", goal_pos, num_retry=BUS_NUM_RETRY)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
