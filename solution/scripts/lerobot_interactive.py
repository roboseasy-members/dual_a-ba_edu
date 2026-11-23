"""한 모델을 유지하며 작업 설명을 반복 입력받는 교육판 interactive 명령."""

import signal
import sys
import threading
from contextlib import contextmanager

from lerobot.configs import parser
from lerobot.rollout import build_rollout_context
from lerobot.rollout.interactive_config import InteractiveRolloutConfig
from lerobot.rollout.strategies.edu_interactive import InteractiveStrategy

# 교육판 rollout과 같은 로봇·카메라 설정 타입을 등록한다.
from lerobot.scripts import lerobot_rollout  # noqa: F401
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import (
    init_visualization,
    shutdown_visualization,
)


@contextmanager
def shutdown_signals(shutdown_event):
    """입력 대기 중에도 종료 신호를 처리하고 기존 핸들러를 복원한다."""

    def shutdown(*_):
        shutdown_event.set()
        raise KeyboardInterrupt

    previous = {}
    try:
        for name in ("SIGINT", "SIGTERM", "SIGHUP", "SIGQUIT"):
            sig = getattr(signal, name, None)
            if sig is not None:
                previous[sig] = signal.getsignal(sig)
                signal.signal(sig, shutdown)
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def run_interactive(cfg: InteractiveRolloutConfig) -> None:
    init_logging()
    shutdown_event = threading.Event()
    strategy = InteractiveStrategy(cfg.strategy)
    ctx = None
    visualization_started = False
    with shutdown_signals(shutdown_event):
        try:
            if cfg.display_data:
                init_visualization(
                    cfg.display_mode, session_name="interactive", ip=cfg.display_ip, port=cfg.display_port
                )
                visualization_started = True
            # 모델 불러오기와 장비 연결은 세션 전체에서 한 번만 수행한다.
            ctx = build_rollout_context(cfg, shutdown_event)
            strategy.setup(ctx)
            strategy.run(ctx)
        except KeyboardInterrupt:
            shutdown_event.set()
        finally:
            try:
                if ctx is not None:
                    strategy.teardown(ctx)
            finally:
                if visualization_started:
                    shutdown_visualization(cfg.display_mode)


@parser.wrap()
def interactive(cfg: InteractiveRolloutConfig) -> None:
    run_interactive(cfg)


def main() -> None:
    register_third_party_plugins()
    # 도움말에서도 interactive 전용 옵션을 표시하도록 기본 타입을 지정한다.
    original_argv = sys.argv
    if not any(arg == "--strategy.type" or arg.startswith("--strategy.type=") for arg in original_argv[1:]):
        sys.argv = [original_argv[0], "--strategy.type=edu_interactive", *original_argv[1:]]
    try:
        interactive()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    main()
