"""LeRobot v0.6.0에 solution/을 복사한 환경에서 실행하는 양팔 interactive 검사."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.rollout.interactive_config import InteractiveStrategyConfig
from lerobot.rollout.strategies.edu_interactive import HomeReturnDetector
from lerobot.scripts import lerobot_interactive


class InteractiveEducationTest(unittest.TestCase):
    def test_cli_accepts_remote_so102_without_opening_hardware(self):
        args = [
            "lerobot_interactive",
            "--robot.type=bi_so102_client",
            "--robot.remote_ip=127.0.0.1",
            "--policy.path=local-checkpoint",
            "--device=cpu",
            "--strategy.task_timeout=12",
        ]
        with (
            patch.object(sys, "argv", args),
            patch.object(PreTrainedConfig, "from_pretrained", return_value=SimpleNamespace(device="cpu")),
            patch.object(lerobot_interactive, "run_interactive") as run,
        ):
            lerobot_interactive.main()

        (cfg,) = run.call_args.args
        self.assertEqual(cfg.robot.type, "bi_so102_client")
        self.assertEqual(cfg.robot.remote_ip, "127.0.0.1")
        self.assertEqual(cfg.policy.pretrained_path, "local-checkpoint")
        self.assertEqual(cfg.strategy.task_timeout, 12)

    def test_home_detection_uses_arm_joints_even_when_grippers_move(self):
        home = dict.fromkeys(
            ("left_elbow_flex.pos", "right_elbow_flex.pos", "left_gripper.pos", "right_gripper.pos"), 0.0
        )
        detector = HomeReturnDetector(InteractiveStrategyConfig(success_settle_ticks=2), home)
        gripping = {**home, "left_gripper.pos": 60.0, "right_gripper.pos": 75.0}

        for observation in (gripping, home, home, home):
            self.assertFalse(detector.update(observation))

        for observation in ({**home, "right_elbow_flex.pos": 30.0}, gripping, home):
            self.assertFalse(detector.update(observation))
        self.assertTrue(detector.update(gripping))


if __name__ == "__main__":
    unittest.main()
