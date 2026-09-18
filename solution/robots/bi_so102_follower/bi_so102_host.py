import base64
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Final

import cv2
import draccus
import numpy as np
import zmq

from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.utils.utils import init_logging

from .bi_so102_follower import BiSO102Follower
from .config_bi_so102_follower import BiSO102FollowerConfig, BiSO102HostConfig

MAX_CONSECUTIVE_READ_FAILURES: Final = 5


@dataclass
class BiSO102ServerConfig:
    robot: BiSO102FollowerConfig
    host: BiSO102HostConfig = field(default_factory=BiSO102HostConfig)


class BiSO102Host:
    def __init__(self, config: BiSO102HostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()


def _encode_observation(observation: dict) -> dict:
    encoded = {}
    for key, value in observation.items():
        if isinstance(value, np.ndarray):
            ret, buffer = cv2.imencode(".jpg", value, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            encoded[key] = base64.b64encode(buffer).decode("utf-8") if ret else ""
        else:
            encoded[key] = float(value)
    return encoded


@draccus.wrap()
def main(cfg: BiSO102ServerConfig):
    init_logging()
    logging.info("Configuring BiSO102Follower")
    robot = BiSO102Follower(cfg.robot)

    logging.info("Connecting BiSO102Follower")
    robot.connect()

    logging.info("Starting host")
    host = BiSO102Host(cfg.host)

    last_cmd_time = time.time()
    watchdog_active = False
    client_connected = True
    consecutive_read_failures = 0
    logging.info("Waiting for commands...")
    try:
        start = time.perf_counter()
        duration = 0
        while host.connection_time_s <= 0 or duration < host.connection_time_s:
            loop_start_time = time.time()
            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                robot.send_action(data)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                pass
            except Exception as e:
                logging.error("Message fetching failed: %s", e)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Holding position."
                )
                watchdog_active = True

            last_observation = None
            try:
                last_observation = _encode_observation(robot.get_observation())
                consecutive_read_failures = 0
            except ConnectionError as e:
                consecutive_read_failures += 1
                logging.warning(
                    "Observation read failed (%d/%d), skipping this cycle: %s",
                    consecutive_read_failures,
                    MAX_CONSECUTIVE_READ_FAILURES,
                    e,
                )
                if consecutive_read_failures >= MAX_CONSECUTIVE_READ_FAILURES:
                    logging.error(
                        "Observation read failed %d times in a row. Stopping host.",
                        consecutive_read_failures,
                    )
                    raise

            if last_observation is not None:
                try:
                    host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
                    if not client_connected:
                        logging.info("Client connected, sending observations")
                        client_connected = True
                except zmq.Again:
                    if client_connected:
                        logging.warning("No client connected, observations are dropped until one connects")
                        client_connected = False

            elapsed = time.time() - loop_start_time
            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
        print("Cycle time reached.")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        print("Shutting down BiSO102 host.")
        robot.disconnect()
        host.disconnect()

    logging.info("Finished BiSO102 host cleanly")


if __name__ == "__main__":
    main()
