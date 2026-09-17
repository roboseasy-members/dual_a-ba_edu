import base64
import json
import logging
from functools import cached_property

import cv2
import numpy as np

from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError

from ..robot import Robot
from .config_bi_so102_follower import BiSO102ClientConfig

SO102_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)


class BiSO102Client(Robot):
    config_class = BiSO102ClientConfig
    name = "bi_so102_client"

    def __init__(self, config: BiSO102ClientConfig):
        import zmq

        self._zmq = zmq
        super().__init__(config)
        self.config = config

        self.remote_ip = config.remote_ip
        self.port_zmq_cmd = config.port_zmq_cmd
        self.port_zmq_observations = config.port_zmq_observations
        self.polling_timeout_ms = config.polling_timeout_ms
        self.connect_timeout_s = config.connect_timeout_s

        self.zmq_context = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None

        self.last_frames: dict[str, np.ndarray] = {}
        self.last_state: dict[str, float] = {}
        self._is_connected = False

    @cached_property
    def _state_ft(self) -> dict[str, type]:
        keys = [f"{side}_{joint}.pos" for side in ("left", "right") for joint in SO102_JOINTS]
        return dict.fromkeys(keys, float)

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        zmq = self._zmq
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_cmd_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_cmd}")
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_observation_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_observations}")
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        socks = dict(poller.poll(self.connect_timeout_s * 1000))
        if self.zmq_observation_socket not in socks or socks[self.zmq_observation_socket] != zmq.POLLIN:
            raise DeviceNotConnectedError(
                f"Timeout waiting for BiSO102 host at {self.remote_ip}. Is the host running on the Raspberry Pi?"
            )

        self._is_connected = True
        logging.info(f"{self} connected to host {self.remote_ip}.")

    def _poll_latest_message(self) -> str | None:
        zmq = self._zmq
        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        try:
            socks = dict(poller.poll(self.polling_timeout_ms))
        except zmq.ZMQError as e:
            logging.error(f"ZMQ polling error: {e}")
            return None
        if self.zmq_observation_socket not in socks:
            return None

        last_msg = None
        while True:
            try:
                last_msg = self.zmq_observation_socket.recv_string(zmq.NOBLOCK)
            except zmq.Again:
                break
        return last_msg

    @staticmethod
    def _decode_image(image_b64: str) -> np.ndarray | None:
        if not image_b64:
            return None
        try:
            jpg = base64.b64decode(image_b64)
            return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        except (TypeError, ValueError) as e:
            logging.error(f"Error decoding image: {e}")
            return None

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        msg = self._poll_latest_message()
        if msg is not None:
            try:
                observation = json.loads(msg)
            except json.JSONDecodeError as e:
                logging.error(f"Error decoding observation: {e}")
                observation = None
            if observation is not None:
                self.last_state = {key: float(observation.get(key, 0.0)) for key in self._state_ft}
                for cam_name in self._cameras_ft:
                    frame = self._decode_image(observation.get(cam_name, ""))
                    if frame is not None:
                        self.last_frames[cam_name] = frame

        obs_dict: RobotObservation = dict(self.last_state)
        for cam_name, (height, width, _) in self._cameras_ft.items():
            frame = self.last_frames.get(cam_name)
            if frame is None:
                logging.warning(f"No frame for camera {cam_name} yet")
                frame = np.zeros((height, width, 3), dtype=np.uint8)
            obs_dict[cam_name] = frame
        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        action_sent = {key: float(action[key]) for key in self._state_ft if key in action}
        self.zmq_cmd_socket.send_string(json.dumps(action_sent))
        return action_sent

    @check_if_not_connected
    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()
        self._is_connected = False
        logging.info(f"{self} disconnected.")
