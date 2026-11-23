# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""작업 설명을 반복 입력받는 교육판 LeRobot 추론 전략.
모델은 유지하고 작업마다 엔진을 교체하여 RTC 이전 작업 상태를 격리한다.
"""

import math
import signal
import threading
import time
from contextlib import contextmanager

import torch

from lerobot.rollout.inference import create_inference_engine
from lerobot.rollout.interactive_config import InteractiveStrategyConfig
from lerobot.rollout.strategies.core import RolloutStrategy
from lerobot.utils.action_interpolator import ActionInterpolator
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.robot_utils import precise_sleep


def await_task_input(shutdown_event: threading.Event) -> str | None:
    """비어 있지 않은 작업 설명을 기다린다. EOF·Ctrl+C는 전체 종료다."""
    while not shutdown_event.is_set():
        try:
            task = input("작업 설명을 입력하세요 (Ctrl+C: 종료)\n> ").strip()
        except (KeyboardInterrupt, EOFError):
            shutdown_event.set()
            return None
        if task:
            return task
    return None


def prime_interpolator(interpolator, observation, ordered_keys) -> None:
    """현재 자세를 보간 시작점으로 넣되 목표 위치 명령은 보내지 않는다."""
    values = [float(observation[key]) for key in ordered_keys]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("보간 시작 자세는 유한해야 합니다.")
    interpolator.reset()
    interpolator.add(torch.tensor(values, dtype=torch.float32))
    # 시작점은 실제 동작 없이 소비한다. 다음 목표부터 현재 자세와 보간된다.
    while interpolator.get() is not None:
        pass


class HomeReturnDetector:
    """초기 자세 이탈 뒤 복귀·정지를 판정한다. 작업 성공의 증명은 아니다."""

    def __init__(self, config: InteractiveStrategyConfig, home: dict):
        self.config = config
        self.home = {
            key: float(value)
            for key, value in home.items()
            if key.endswith(".pos") and not key.startswith("gripper") and not key.endswith("_gripper.pos")
        }
        self.left_home = False
        self.settle_count = 0
        self.previous = None

    def update(self, observation: dict) -> bool:
        if not self.home or any(key not in observation for key in self.home):
            return False
        current = {key: float(observation[key]) for key in self.home}
        if not all(math.isfinite(value) for value in current.values()):
            raise ValueError("초기 자세 판정에 유효하지 않은 관측이 있습니다.")
        distance = max(abs(current[key] - self.home[key]) for key in current)
        if not self.left_home:
            self.left_home = distance > self.config.success_leave_thresh
            self.previous = current
            return False
        step = max(abs(current[key] - self.previous[key]) for key in current)
        self.previous = current
        if distance <= self.config.success_return_thresh and step <= self.config.success_settle_eps:
            self.settle_count += 1
        else:
            self.settle_count = 0
        return self.settle_count >= self.config.success_settle_ticks


@contextmanager
def task_interrupt_handler(interrupted: threading.Event):
    """실행 중 Ctrl+C를 현재 작업 중단으로 처리하고 원래 핸들러를 복원한다."""
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous)


class InteractiveStrategy(RolloutStrategy):
    """이미 불러온 모델로 설명 입력·추론·복귀를 반복한다."""

    def __init__(self, config: InteractiveStrategyConfig) -> None:
        super().__init__(config)
        self._healthy = True
        self._at_home = True
        self._unsafe_worker = False
        self._warmup_done = False
        self._deadline = math.inf

    def setup(self, ctx) -> None:
        cfg = ctx.runtime.cfg
        self._interpolator = ActionInterpolator(cfg.interpolation_multiplier)
        self._warmup_done = not (cfg.use_torch_compile and cfg.inference.type == "rtc")

    def _start_engine(self, ctx, task: str) -> None:
        cfg = ctx.runtime.cfg
        self._engine = create_inference_engine(
            cfg.inference,
            policy=ctx.policy.policy,
            preprocessor=ctx.policy.preprocessor,
            postprocessor=ctx.policy.postprocessor,
            robot_wrapper=ctx.hardware.robot_wrapper,
            hw_features=ctx.data.hw_features,
            dataset_features=ctx.data.dataset_features,
            ordered_action_keys=ctx.data.ordered_action_keys,
            task=task,
            fps=cfg.fps,
            device=cfg.device,
            use_torch_compile=not self._warmup_done,
            compile_warmup_inferences=cfg.compile_warmup_inferences,
            shutdown_event=ctx.runtime.shutdown_event,
        )
        ctx.policy.inference = self._engine
        # 이전 작업의 추론 스레드가 끝난 뒤에만 상태를 초기화한다.
        self._engine.reset()
        self._engine.pause()
        self._engine.start()
        observation = ctx.hardware.robot_wrapper.get_observation()
        prime_interpolator(self._interpolator, observation, ctx.data.ordered_action_keys)
        self._cached_obs_processed = None
        self._process_observation_and_notify(ctx.processors, observation)
        self._engine.resume()

    def _stop_engine(self) -> None:
        if self._engine is None:
            return
        # LeRobot stop()이 내부 스레드 참조를 지우기 전에 보관한다.
        worker = getattr(self._engine, "_rtc_thread", None)
        self._engine.pause()
        self._engine.stop()
        if worker is not None and worker.is_alive():
            worker.join(timeout=self.config.inference_stop_timeout_s)
            if worker.is_alive():
                self._unsafe_worker = True
                raise RuntimeError("이전 RTC 추론이 끝나지 않아 작업 전환을 중단합니다.")
        self._engine = None

    def run(self, ctx) -> None:
        cfg = ctx.runtime.cfg
        if cfg.duration > 0:
            self._deadline = time.monotonic() + cfg.duration
        try:
            while not ctx.runtime.shutdown_event.is_set() and time.monotonic() < self._deadline:
                task = await_task_input(ctx.runtime.shutdown_event)
                if task is None or time.monotonic() >= self._deadline:
                    break
                self._at_home = False
                try:
                    self._start_engine(ctx, task)
                    self._run_task(ctx, task)
                finally:
                    self._stop_engine()
                if not ctx.runtime.shutdown_event.is_set():
                    self._return_home(ctx)
        except BaseException:
            self._healthy = False
            raise

    def _run_task(self, ctx, task: str) -> None:
        cfg = ctx.runtime.cfg
        interrupted = threading.Event()
        deadline = min(self._deadline, time.monotonic() + self.config.task_timeout)
        detector = (
            HomeReturnDetector(self.config, ctx.hardware.initial_position or {})
            if self.config.success_detection
            else None
        )
        interval = self._interpolator.get_control_interval(cfg.fps)

        def should_stop():
            return interrupted.is_set() or ctx.runtime.shutdown_event.is_set() or time.monotonic() >= deadline

        print(f"작업 시작: {task} (Ctrl+C: 현재 작업 중단)", flush=True)
        with task_interrupt_handler(interrupted):
            while not should_stop():
                started = time.monotonic()
                if self._engine.failed:
                    raise RuntimeError("추론 엔진에서 복구할 수 없는 오류가 발생했습니다.")
                observation = ctx.hardware.robot_wrapper.get_observation()
                processed = self._process_observation_and_notify(ctx.processors, observation)
                if not self._warmup_done:
                    if not self._engine.ready:
                        # 가득 찬 큐가 다음 워밍업 추론을 막지 않게 비운다.
                        self._engine.get_action(None)
                    else:
                        # 추론과 reset을 겹치지 않고 워밍업 결과도 폐기한다.
                        self._stop_engine()
                        self._warmup_done = True
                        if not should_stop():
                            self._start_engine(ctx, task)
                    precise_sleep(max(0.0, interval - (time.monotonic() - started)))
                    continue
                action = self._dispatch_action(ctx, processed, observation, should_stop)
                self._log_telemetry(processed, action, ctx.runtime)
                if detector is not None and detector.update(observation):
                    break
                precise_sleep(max(0.0, interval - (time.monotonic() - started)))
        if self._engine is not None and self._engine.failed:
            raise RuntimeError("추론 엔진 오류로 작업을 중단했습니다.")

    def _dispatch_action(self, ctx, processed, observation, should_stop):
        if self._interpolator.needs_new_action():
            frame = build_dataset_frame(ctx.data.dataset_features, processed, prefix="observation")
            action = self._engine.get_action(frame)
            if should_stop():
                return None
            if action is not None:
                if not torch.isfinite(action).all():
                    raise ValueError("추론 action은 유한해야 합니다.")
                self._interpolator.add(action.cpu())
        action = self._interpolator.get()
        if action is None or should_stop():
            return None
        keys = ctx.data.ordered_action_keys
        if len(action) != len(keys):
            raise ValueError("추론 action 길이가 로봇 관절 수와 다릅니다.")
        values = {key: action[index].item() for index, key in enumerate(keys)}
        command = ctx.processors.robot_action_processor((values, observation))
        if should_stop():
            return None
        ctx.hardware.robot_wrapper.send_action(command)
        return values

    def _return_home(self, ctx) -> None:
        if (
            not ctx.runtime.cfg.return_to_initial_position
            or self._at_home
            or not ctx.hardware.initial_position
        ):
            return
        robot = ctx.hardware.robot_wrapper
        target = ctx.hardware.initial_position
        observation = robot.get_observation()
        current = {key: float(observation[key]) for key in target}
        if not all(math.isfinite(value) for value in (*current.values(), *target.values())):
            raise ValueError("복귀 위치는 유한해야 합니다.")
        print("시작할 때의 위치로 복귀합니다.", flush=True)
        for step in range(1, 151):
            if ctx.runtime.shutdown_event.is_set():
                return
            ratio = step / 150
            robot.send_action({key: current[key] + ratio * (target[key] - current[key]) for key in target})
            precise_sleep(1 / 50)
        self._at_home = True

    def teardown(self, ctx) -> None:
        try:
            if not self._unsafe_worker:
                self._stop_engine()
            if self._healthy and not ctx.runtime.shutdown_event.is_set():
                self._return_home(ctx)
        finally:
            try:
                robot = ctx.hardware.robot_wrapper.inner
                if robot.is_connected:
                    robot.disconnect()
            finally:
                teleop = ctx.hardware.teleop
                if teleop is not None and teleop.is_connected:
                    teleop.disconnect()
