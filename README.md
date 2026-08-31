# dual_a-ba_edu

양팔 SO-101 리더암 텔레옵 + LeRobot 데이터 수집(record) 레포지토리입니다.
리더암을 손으로 움직이면 팔로워암이 관절 단위로 따라갑니다. 그 동작은
LeRobot 데이터셋으로 녹화합니다.

## 타깃 하드웨어

| 구성 | 내용 | 전원 |
|------|------|------|
| 팔로워 (로봇) | 양팔 SO-101, 각 6모터 (id 1~6) | 12V |
| 카메라 헤드 (선택) | pan/tilt 2모터 (id 7/8), 왼팔과 같은 버스 | 12V |
| 리더암 (조종) | SO-101 leader 1~2대, 각 6모터 (id 1~6) | 5V |

- `bus1`(left_arm_port) = 왼팔 6모터 + (장착 시) 머리 2모터.
  `bus2`(right_arm_port) = 오른팔 6모터.
- SO-102(손목 yaw 추가, 7모터)도 지원합니다. 아래 명령에서
  `--robot.type=bi_so102_follower` + `--teleop.type=bi_so102_leader`로
  바꾸면 됩니다. 로봇과 리더는 같은 세대끼리만 조합할 수 있고, 어긋난
  조합은 시작 시점에 에러로 걸러집니다.

## 설치

Python 3.12 기준입니다. conda나 venv 같은 가상환경을 쓰는 편이 좋습니다.

```bash
git clone <레포 URL> && cd dual_a-ba_edu
pip install -r requirements.txt   # lerobot==0.6.0 포함
```

## 포트 확인

USB를 꽂고 어떤 포트가 어느 팔인지부터 확인합니다. 읽기 전용 점검이라
모터는 움직이지 않습니다.

```bash
PYTHONPATH=src python -m leader_teleop.scripts.check_robot
```

포트별로 팔로워 bus1(머리 포함) / 팔로워 팔 / 리더암(5V)을 분류해
실행 플래그를 제안합니다. 포트 번호는 재연결하면 바뀔 수 있습니다.

## 실행 (텔레옵만, 녹화 없음)

```bash
# 리더암 2대로 양팔 조종
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3

# 리더암 1대로 왼팔만 (오른팔은 시작 자세 유지)
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.mode=left --teleop.left_arm_port=/dev/ttyACM2
```

- 첫 실행에서 캘리브레이션이 시작됩니다. 터미널 안내대로 중립 자세,
  전체 가동범위 순서로 움직이면 파일로 저장되고 다음부터 재사용합니다.
- 종료는 `Ctrl+C`. 팔이 홈포즈(수그린 자세)로 복귀한 뒤 토크가
  풀립니다. `--home_return.is_enabled=false`로 끌 수 있습니다.
- 리더암에서 손을 놓으면 팔로워는 그 자리에 멈춥니다.

## 데이터 수집 (record)

텔레옵과 같은 인자 체계에 `--dataset.*`가 더해집니다. 카메라는
`--robot.cameras`로 등록합니다 (lerobot 표준). 아래 예시는 상단 1대 +
양쪽 손목 2대 구성입니다. `index_or_path`는 PC마다 다르니
`v4l2-ctl --list-devices`로 먼저 확인하세요.

```bash
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "left_wrist": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, "right_wrist": {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.single_task='물건을 집어 상자에 넣는다' \
	--dataset.num_episodes=10 --dataset.push_to_hub=false \
	--display_data=true
```

에피소드 제어는 lerobot 표준 키보드를 따릅니다. `→` 다음 에피소드로
조기 종료, `←` 현재 에피소드 재녹화, `Esc` 녹화 전체 종료.
record 중 `Ctrl+C`는 진행 중인 에피소드를 버립니다. 종료는 `Esc`로
하세요.

## 라즈베리파이 분리 구성 (host / client)

팔로워 2대와 카메라를 **라즈베리파이**에, 리더암 2대를 **PC**에 꽂는
구성입니다. 파이에서 `host`를 띄우면 로봇이 네트워크로 노출되고, PC의
`teleoperate` / `record`는 로봇 타입만 `bi_so101_client`로 바꾸면 그대로
동작합니다.

```
PC (teleoperate/record + 리더암 2대)  <-- ZMQ 5555/5556 -->  Pi (host + 팔로워 2대 + 카메라 3대)
```

### 1. 파이 준비 (최초 1회)

Raspberry Pi OS 64-bit, **Python 3.12 이상**이 필요합니다(lerobot 0.6.0
요구 사양). Bookworm 기본 Python은 3.11이라 그대로는 설치가 안 됩니다 —
pyenv/uv로 3.12를 넣거나 3.12 이상이 기본인 배포판을 쓰세요(이 절차는
3.12에서 확인했습니다). 시스템 pip는 막혀 있으므로 venv를 씁니다. torch는
CPU 전용 wheel을 먼저 설치해야 쓸모없는 CUDA 패키지가 딸려오지 않습니다.

```bash
sudo apt update && sudo apt install -y python3-venv git v4l-utils
git clone <레포 URL> && cd dual_a-ba_edu
python3 -m venv ~/.venvs/dual_edu && source ~/.venvs/dual_edu/bin/activate
pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements_host.txt      # lerobot[feetech] + pyzmq만 (mujoco·rerun 제외)
sudo usermod -aG dialout $USER            # 시리얼 권한, 재로그인 필요
```

카메라 인덱스는 파이에서 `v4l2-ctl --list-devices`로 확인합니다.

### 2. 파이에서 host 실행

PC에서 `ssh <사용자>@<파이 IP>`로 들어간 뒤 실행합니다. 첫 실행에서
캘리브레이션 프롬프트가 뜨면 안내대로 진행합니다.

```bash
PYTHONPATH=src python -m leader_teleop.host \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "left_wrist": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, "right_wrist": {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30}}'
```

`Waiting for client...`가 뜨면 준비 완료입니다. 이 터미널은 그대로
둡니다. SSH가 끊기면 host는 정리(홈포즈 복귀·토크 해제)를 시도한 뒤
종료됩니다 — 끊김 상황의 정리는 실기로 검증하지 않았으니, 수업처럼
장시간 운용이면 `tmux` 안에서 띄워 SSH와 host의 수명을 분리하세요.
안전 여유가 필요하면 `--robot.max_relative_target=5`(전송 1회당 관절
이동 상한, deg)를 추가합니다 — 대신 명령마다 현재 위치를 읽어 지연이
조금 늘어납니다.

### 3. PC에서 텔레옵 / 데이터 수집

`--robot.type=bi_so101_client` + `--robot.remote_ip=<파이 IP>`로
바꿉니다. **PC 쪽 `--robot.cameras`는 이름과 해상도만 쓰이므로** host와
같은 값을 주되 `index_or_path`는 아무 값이나 됩니다 (PC가 카메라를 열지
않습니다).

```bash
# 텔레옵만
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so101_client --robot.remote_ip=<파이 IP> \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM0 \
	--teleop.right_arm_port=/dev/ttyACM1

# 데이터 수집
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so101_client --robot.remote_ip=<파이 IP> \
	--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "left_wrist": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "right_wrist": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM0 \
	--teleop.right_arm_port=/dev/ttyACM1 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.single_task='물건을 집어 상자에 넣는다' \
	--dataset.num_episodes=10 --dataset.push_to_hub=false \
	--display_data=true
```

### 동작 규칙

- PC와 파이는 같은 네트워크에 있어야 하고, 파이의 5555/5556 포트가
  열려 있어야 합니다. 포트를 바꾸면 파이 `--host.port_zmq_cmd` /
  `--host.port_zmq_observations`와 PC `--robot.port_zmq_cmd` /
  `--robot.port_zmq_observations`를 **함께** 바꿉니다.
- 인증이 없으므로 같은 LAN의 누구나 명령을 보낼 수 있습니다. 수업용
  폐쇄망에서만 쓰세요.
- 머리 모터가 있는 구성이면 host에 `--camera_head.mode=fixed`, PC에
  원하는 모드(`fixed`/`keyboard`)를 줍니다(host는 장착 여부만 봅니다).
  한쪽만 `none`이면 PC 쪽에서 키 불일치 경고가 납니다.
- PC 프로그램을 끝내도(Ctrl+C/Esc) 팔은 마지막 자세를 유지합니다.
  홈포즈 복귀와 토크 해제는 **파이 host를 끝낼 때**(Ctrl+C, SIGTERM,
  SSH 끊김) 일어납니다.
- host 관측이 1초 이상 끊기면 PC 쪽이 오류로 멈춥니다(`stale_timeout_s`,
  기본 1.0). 멈춘 host의 오래된 프레임이 데이터셋에 기록되는 것을 막기
  위한 동작입니다. 끊긴 원인(host 종료, WiFi)을 해결하고 다시 시작하세요.
- 카메라 3대 640×480 30fps는 약 40Mbps입니다. 유선 또는 5GHz WiFi를
  권장하고, 끊기면 `--host.fps=15`나 `--host.jpeg_quality=60`으로
  전송량을 줄입니다. host 루프가 목표 주기를 못 채우면 경고 로그가
  납니다.
- PC 쪽 카메라 해상도가 host와 다르면 연결 시점에 오류로 알려줍니다.
- host의 카메라 한 대가 잠시 멈추면(읽기 타임아웃) 직전 관측을 재사용해
  최대 약 1초까지 버팁니다. 이 구간의 프레임은 중복이며, 1초를 넘기면
  host가 정리 후 종료됩니다.
- 캘리브레이션 파일은 로봇이 붙은 쪽(파이)에 저장됩니다. PC 쪽
  client는 캘리브레이션을 하지 않습니다.
- 이 구성으로 수집한 데이터셋의 `robot_type`은 `bi_so101_client`로
  기록됩니다. 직접 연결로 만든 데이터셋(`bi_so101_follower`)을 client로
  `--resume`하면 타입 불일치로 거부됩니다.

## 학습한 정책 실행 (추론)

`lerobot-rollout`을 이 레포의 로봇 타입이 등록된 상태로 실행하는
진입점입니다(`lerobot-rollout` 명령을 직접 쓰면 `bi_so101_client` 같은
타입을 모릅니다). 파이 host 구성이면 `--robot.type=bi_so10x_client`, 직접
연결이면 `bi_so10x_follower`를 씁니다.

```bash
PYTHONPATH=src python -m leader_teleop.rollout \
	--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--policy.path=outputs/<학습 출력 폴더>/checkpoints/last/pretrained_model \
	--rename_map='{"observation.images.cam_top": "observation.images.camera1", "observation.images.cam_wrist_left": "observation.images.camera2", "observation.images.cam_wrist_right": "observation.images.camera3"}' \
	--task="Put the red die on the yellow cloth." \
	--duration=60 --fps=10 --policy.device=cuda
```

- `--rename_map`은 **학습 때 준 것과 같아야** 합니다. `smolvla_base`에서
  파인튜닝한 정책은 카메라를 `camera1/2/3`으로 기대합니다. 빼면
  `Visual feature mismatch`로 시작하지 않습니다.
- `--task`는 언어 조건부 정책(SmolVLA)이 읽는 문장입니다. ACT는 무시합니다.
- `--policy.path`에는 허브 이름(`rkdals2779/<모델>`)도 됩니다.
- 종료는 `Ctrl+C` 또는 `--duration` 만료. 기본으로 시작 자세로 되돌린 뒤
  끝납니다(`--return_to_initial_position=false`로 끌 수 있음).

## 카메라 헤드 (선택)

머리 pan/tilt 모터가 있는 구성이면 `--camera_head.mode`로 제어합니다.
기본은 `none`. 이때는 머리 모터를 검색하지 않습니다.

| 모드 | 동작 |
|------|------|
| `none` (기본) | 머리 미장착 구성 — 머리 키가 데이터셋에서도 빠짐 |
| `fixed` | 시작 시 고정각(기본 홈각)으로 이동 후 유지 |
| `keyboard` | `a/d`=pan, `w/s`=tilt, `h`=홈 (record 방향키와 안 겹침) |

## 홈포즈 조정

종료 시 복귀하는 홈포즈 관절각은 `src/leader_teleop/config.py`의
`HomeReturnConfig`에 있습니다. 내 로봇에 맞는 값은 이렇게 잽니다:

```bash
PYTHONPATH=src python -m leader_teleop.scripts.capture_home_pose \
	--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1
```

토크가 풀린 상태에서 원하는 자세로 팔을 잡으면, config에 붙여넣을
값이 출력됩니다.

## 코드 구조

| 역할 | 파일 |
|------|------|
| 진입점 (직접 텔레옵 / 데이터 수집 / 추론) | `teleoperate.py` / `record.py` / `rollout.py` |
| 조립·연결·제어 루프·정리 | `app.py` |
| 공유 설정 (홈포즈 등) | `config.py` |
| 양팔+머리 팔로워 로봇 (공통/세대 명세) | `robots/bi_follower_base.py` + `bi_so101_follower.py`/`bi_so102_follower.py` |
| 양팔 리더암 텔레옵 (공통/세대 명세) | `teleoperators/bi_leader_base.py` + `bi_so101_leader.py`/`bi_so102_leader.py` |
| 팔 + 머리 합성 텔레옵 | `teleoperators/combined_teleop.py` |
| 카메라 헤드 모드 | `head/head_modes.py` |
| 연결 재시도 (간헐 통신 글리치 흡수) | `hardware_retry.py` |
| 라즈베리파이 host / PC client / 직렬화 | `host.py` / `robots/bi_follower_client.py` / `remote_codec.py` (파이 의존성: `requirements_host.txt`) |
| 하드웨어 점검 / 홈포즈 캡처 | `scripts/check_robot.py` / `scripts/capture_home_pose.py` |

## 안전 주의

- 실행 전 로봇 주변을 치우세요. `Ctrl+C`를 바로 누를 수 있게 터미널에
  손을 두세요.
- 전압 에러가 반복되면 전원부터 확인하세요. SO-101 리더암은 5V,
  팔로워암은 12V입니다. 데이지체인 커넥터도 다시 꽂아 보세요.
- 리더암과 팔로워암의 좌/우를 바꿔 꽂으면 팔이 교차로 움직입니다.
  실행 직후에는 천천히 움직여 좌/우 대응부터 확인하세요.
