# dual_a-ba_edu

양팔 SO-101/SO-102 로봇으로 텔레오퍼레이션 → 데이터 수집 → 정책 학습 →
추론까지 한 바퀴 도는 교육용 레포지토리입니다. 리더암을 손으로 움직이면
팔로워암이 관절 단위로 따라가고, 그 동작을 LeRobot 데이터셋으로 녹화해
SmolVLA 또는 ACT를 학습합니다.

```
텔레옵 (teleoperate) → 데이터 수집 (record) → 데이터셋 편집 (병합·프롬프트)
    → 학습 (lerobot-train: SmolVLA / ACT) → 추론 (rollout)
```

로봇은 PC에 직접 연결하거나, 라즈베리파이에 팔로워를 두고 PC에서 원격으로
조작하는 두 구성을 지원합니다.

## 목차

1. [타깃 하드웨어](#타깃-하드웨어)
2. [설치](#설치)
3. [포트 확인](#포트-확인)
4. [텔레오퍼레이션](#텔레오퍼레이션)
5. [데이터 수집](#데이터-수집)
6. [라즈베리파이 분리 구성](#라즈베리파이-분리-구성)
7. [데이터셋 편집](#데이터셋-편집)
8. [학습](#학습)
9. [추론](#추론)
10. [카메라 헤드 / 홈포즈 / 코드 구조 / 안전](#카메라-헤드-선택)
11. [문제 해결](#문제-해결)

## 타깃 하드웨어

| 구성 | 내용 | 전원 |
|------|------|------|
| 팔로워 (로봇) | 양팔 SO-101(6모터, id 1~6) 또는 SO-102(7모터, 손목 yaw 추가) | 12V |
| 카메라 헤드 (선택) | pan/tilt 2모터 (id 7/8), 왼팔과 같은 버스 | 12V |
| 리더암 (조종) | SO-101/SO-102 leader 1~2대 | 5V |
| 카메라 | 상단 1대 + 양쪽 손목 2대 (USB 웹캠) | — |

- `bus1`(left_arm_port) = 왼팔 모터 + (장착 시) 머리 2모터,
  `bus2`(right_arm_port) = 오른팔 모터입니다.
- SO-101은 `bi_so101_*`, SO-102는 `bi_so102_*` 타입을 씁니다. 로봇과
  리더는 같은 세대끼리만 조합할 수 있고, 어긋난 조합은 시작 시점에
  오류로 걸러집니다.
- 팔로워 한 대의 소비 전력은 12V 기준 약 24W입니다. 배터리로 구동할
  때는 총 출력이 팔 수 × 24W 이상인지 확인합니다.

## 설치

Python 3.12 기준입니다. conda나 venv 같은 가상환경을 씁니다.

```bash
sudo apt update && sudo apt install -y git v4l-utils ffmpeg  # ffmpeg: 데이터셋 영상 디코딩(TorchCodec)
git clone <레포 URL> && cd dual_a-ba_edu
pip install -r requirements.txt
```

`requirements.txt`에는 lerobot 0.6.0(모터·시각화·영상 인코딩·학습(accelerate/wandb)·
SmolVLA extra), pyzmq(원격 구성), mujoco(시뮬레이션 실습)가 들어 있습니다. 학습을
GPU로 할 PC에는 CUDA용 torch가 설치돼야 하며, PyPI 기본 torch가 CUDA를
포함합니다.

## 포트 확인

USB를 꽂고 어떤 포트가 어느 팔인지부터 확인합니다. 읽기 전용 점검이라
모터는 움직이지 않습니다.

```bash
PYTHONPATH=src python -m leader_teleop.scripts.check_robot
```

포트별로 팔로워 bus1(머리 포함) / 팔로워 팔 / 리더암(5V)을 분류해
실행 플래그를 제안합니다. 포트 번호는 재연결하면 바뀔 수 있습니다.

## 텔레오퍼레이션

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

- 첫 실행에서 캘리브레이션이 시작됩니다. 팔로워는 왼팔 → 오른팔,
  리더는 왼쪽 → 오른쪽 순서로 "가동범위 중간 자세에서 엔터 → 모든
  관절을 끝에서 끝까지 움직이고 엔터"를 반복합니다. 파일로 저장되어
  다음부터 재사용하고, 다시 하려면 프롬프트에서 `c`를 입력합니다.
- 종료는 `Ctrl+C`입니다. 팔이 홈포즈(수그린 자세)로 복귀한 뒤 토크가
  풀립니다. `--home_return.is_enabled=false`로 끌 수 있습니다.
- 리더암에서 손을 놓으면 팔로워는 그 자리에 멈춥니다.

## 데이터 수집

텔레옵과 같은 인자 체계에 `--dataset.*`가 더해집니다. 카메라는
`--robot.cameras`로 등록합니다. 아래 예시는 상단 1대 + 양쪽 손목 2대
구성이며, `index_or_path`는 PC마다 다르니 `v4l2-ctl --list-devices`로
먼저 확인합니다.

```bash
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_left": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, "cam_wrist_right": {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.root=$HOME/lerobot_datasets/<데이터셋이름> \
	--dataset.single_task='red' \
	--dataset.num_episodes=25 --dataset.episode_time_s=1000 --dataset.reset_time_s=5 \
	--dataset.push_to_hub=false \
	--display_data=true
```

- 에피소드 제어는 lerobot 표준 키보드를 따릅니다. `→` 다음 에피소드로
  조기 종료, `←` 현재 에피소드 재녹화, `Esc` 녹화 전체 종료. record 중
  `Ctrl+C`는 진행 중인 에피소드를 버립니다. 종료는 `Esc`로 합니다.
- `--dataset.root`를 명시하면 데이터셋이 그 폴더에 만들어집니다.
  생략하면 `~/.cache/huggingface/lerobot/<repo_id>_<날짜_시각>`처럼 이름에
  타임스탬프가 붙어 이어 찍기 때 경로를 찾기 번거롭습니다.
- `--dataset.single_task`는 이 실행에서 찍는 모든 프레임에 붙는 문장입니다.
  SmolVLA처럼 언어를 읽는 정책을 학습할 계획이면 여기서는 `red`, `blue`
  같은 자리표시자로 행동 종류만 구분해 두고, 진짜 문장은 나중에
  [데이터셋 편집](#데이터셋-편집)에서 에피소드별로 배정합니다.
- 언어 조건부 정책을 위해 여러 행동(빨간 주사위 / 파란 주사위)을
  수집할 때는 매 에피소드에 두 물체를 모두 놓습니다. 한 물체만 놓고 찍으면
  정책이 문장을 읽지 않고 장면만으로 답을 알게 됩니다.

## 라즈베리파이 분리 구성

팔로워 2대와 카메라를 라즈베리파이에, 리더암 2대를 PC에 꽂는 구성입니다.
파이에서 `host`를 띄우면 로봇이 네트워크로 노출되고, PC의 `teleoperate` /
`record` / `rollout`은 로봇 타입만 `bi_so10x_client`로 바꾸면 그대로
동작합니다.

```
PC (teleoperate/record/rollout + 리더암 2대)  <-- ZMQ 5555/5556 -->  Pi (host + 팔로워 2대 + 카메라 3대)
```

### 1. 파이 준비 (최초 1회)

Raspberry Pi OS 64-bit, Python 3.12 이상이 필요합니다(lerobot 0.6.0 요구
사양). Bookworm 기본 Python은 3.11이라 pyenv/uv로 3.12를 넣거나 3.12
이상이 기본인 배포판을 씁니다. 시스템 pip는 막혀 있으므로 venv를 씁니다.
torch는 CPU 전용 wheel을 먼저 설치해야 파이에서 쓸모없는 CUDA 패키지가
딸려오지 않습니다(lerobot이 요구하는 torch 2.11부터 aarch64에도 CUDA
패키지가 붙습니다).

```bash
sudo apt update && sudo apt install -y python3-venv git v4l-utils ffmpeg
git clone <레포 URL> && cd dual_a-ba_edu
python3 -m venv ~/.venvs/dual_edu && source ~/.venvs/dual_edu/bin/activate
pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements_host.txt      # lerobot[feetech,dataset] + pyzmq + pynput
sudo usermod -aG dialout $USER            # 시리얼 권한, 재로그인 필요
```

`dataset` extra는 host 자체에는 필요 없지만, 파이에서 PC 없이 정책을
직접 실행(`leader_teleop.rollout --robot.type=bi_so10x_follower
--policy.device=cpu`)할 때 rollout 모듈이 요구합니다. ACT 1회 추론이
PC CPU 4스레드에서 0.28초였고 파이에서는 몇 배 느리므로, 행동 청크
사이에 멈춤이 생기는 데모용 경로입니다.

### 2. 파이에서 카메라 확인

```bash
v4l2-ctl --list-devices                                   # 장치별 /dev/videoN
v4l2-ctl -d /dev/video0 --list-formats-ext                # MJPG 지원과 해상도
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG \
	--stream-mmap --stream-count=60                       # 실측 fps
```

USB 웹캠은 영상 노드와 메타데이터 노드 두 개를 잡으므로 보통 짝수 번호가
영상입니다. 세 대를 동시에 스트리밍해도 fps가 유지되는지 봅니다.

### 3. 파이에서 host 실행

PC에서 `ssh <사용자>@<파이 IP>`로 들어간 뒤 실행합니다. 카메라는
`"fourcc": "MJPG"`로 엽니다. 무압축(YUYV)으로 열면 640×480 한 대가 약
150Mbps라 USB 2.0 버스에서 두 대째부터 프레임이 끊깁니다. `warmup_s`는
첫 프레임 대기 시간으로, 기본 1초가 짧아 타임아웃이 나는 카메라가 있어
3초로 둡니다.

```bash
PYTHONPATH=src python -m leader_teleop.host \
	--robot.type=bi_so102_follower --robot.id=bi_so102_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_wrist_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}, "cam_wrist_right": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG", "warmup_s": 3}}'
```

`Waiting for client...`가 뜨면 준비 완료입니다. 첫 실행에서 팔로워
캘리브레이션이 진행되며 파일은 파이에 저장됩니다. 수업처럼 장시간
운용하면 `tmux` 안에서 띄워 SSH와 host의 수명을 분리합니다.

`--robot.max_relative_target`(전송 1회당 관절 이동 상한)은 텔레옵·수집에
쓰지 않습니다. 값을 5로 주면 30Hz 루프에서 관절 속도가 150°/s로 잘리고
명령마다 현재 위치를 다시 읽어 루프가 느려져, 팔로워가 리더를 뒤늦게
따라오는 둔한 반응이 됩니다. 첫 연결 순간의 점프는 클라이언트를 붙이기
전에 리더암을 팔로워의 현재 자세(홈포즈)와 비슷하게 잡아 두는 것으로
줄입니다.

### 4. PC에서 텔레옵 / 데이터 수집

`--robot.type=bi_so102_client`와 `--robot.remote_ip=<파이 IP>`로 바꿉니다.
PC 쪽 `--robot.cameras`는 이름과 해상도만 쓰이므로 host와 같은 값을 주되
`index_or_path`는 아무 값이나 됩니다(PC가 카메라를 열지 않습니다).

```bash
# 텔레옵만
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \
	--teleop.type=bi_so102_leader --teleop.id=bi_so102_leader \
	--teleop.left_arm_port=/dev/ttyACM0 \
	--teleop.right_arm_port=/dev/ttyACM1

# 데이터 수집
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so102_leader --teleop.id=bi_so102_leader \
	--teleop.left_arm_port=/dev/ttyACM0 \
	--teleop.right_arm_port=/dev/ttyACM1 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.root=$HOME/lerobot_datasets/<데이터셋이름> \
	--dataset.single_task='red' \
	--dataset.num_episodes=25 --dataset.episode_time_s=1000 --dataset.reset_time_s=5 \
	--dataset.push_to_hub=false \
	--display_data=true
```

### 동작 규칙

- PC와 파이는 같은 네트워크에 있어야 하고, 파이의 5555/5556 포트가
  열려 있어야 합니다. 포트를 바꾸면 파이 `--host.port_zmq_cmd` /
  `--host.port_zmq_observations`와 PC `--robot.port_zmq_cmd` /
  `--robot.port_zmq_observations`를 함께 바꿉니다.
- 인증이 없으므로 같은 LAN의 누구나 명령을 보낼 수 있습니다. 수업용
  폐쇄망에서만 씁니다.
- 머리 모터가 있는 구성이면 host에 `--camera_head.mode=fixed`, PC에
  원하는 모드(`fixed`/`keyboard`)를 줍니다. 한쪽만 `none`이면 PC 쪽에서
  키 불일치 경고가 납니다.
- PC 프로그램을 끝내도(Ctrl+C/Esc) 팔은 마지막 자세를 유지합니다.
  홈포즈 복귀와 토크 해제는 파이 host를 끝낼 때(Ctrl+C, SIGTERM,
  SSH 끊김) 일어납니다.
- host 관측이 1초 이상 끊기면 PC 쪽이 오류로 멈춥니다(`stale_timeout_s`,
  기본 1.0). 멈춘 host의 오래된 프레임이 데이터셋에 기록되는 것을 막기
  위한 동작입니다.
- 카메라 3대 640×480 30fps는 약 40Mbps입니다. 유선 또는 5GHz WiFi를
  권장하고, 끊기면 `--host.fps=15`나 `--host.jpeg_quality=60`으로 전송량을
  줄입니다. host 루프가 목표 주기를 못 채우면 경고 로그가 납니다.
- PC 쪽 카메라 이름·해상도가 host와 다르면 연결 시점에 오류로 알려줍니다.
- host는 카메라 타임아웃이나 모터 버스의 순간 무응답(`no status packet`)을
  일시적 오류로 보고 직전 관측 재사용·명령 건너뛰기로 버팁니다(관측
  연속 10회, 명령 연속 15회까지). 재사용 구간의 프레임은 중복이며, 한도를
  넘기면 장애로 보고 정리 후 종료됩니다. 첫 명령 순간 팔이 크게 점프하며
  전류 피크로 버스가 잠깐 끊기는 경우가 흔하니, 연결 전에 리더 자세를
  팔로워와 비슷하게 맞추고 전원 어댑터의 전류 여유를 확인합니다.
- 이 구성으로 수집한 데이터셋의 `robot_type`은 `bi_so10x_client`로
  기록됩니다. 직접 연결로 만든 데이터셋을 client로 `--resume`하면 타입
  불일치로 거부됩니다.

## 데이터셋 편집

아래 명령은 모두 `lerobot-edit-dataset`을 씁니다. `--root`는 데이터셋이
있는 로컬 폴더이며, 생략하면 `~/.cache/huggingface/lerobot/<repo_id>`를
봅니다.

### 상태 확인

```bash
lerobot-edit-dataset --repo_id=<HF계정>/<데이터셋이름> --root=<로컬경로> --operation.type=info
```

`info`는 `info.json`의 값을 읽을 뿐 메타 파일과 대조하지 않습니다.
편집 뒤에는 메타 행 수까지 확인합니다.

```bash
python -c "
import glob, pandas as pd
df = pd.concat(pd.read_parquet(f) for f in glob.glob('<로컬경로>/meta/episodes/**/*.parquet', recursive=True))
print('메타 행', len(df), '| 고유 에피소드', df.episode_index.nunique(), '| 중복', df.episode_index.duplicated().sum())"
```

### 이어 찍기

`--resume=true`에는 `--dataset.root`가 필수입니다. 나머지 인자(로봇 타입,
카메라 이름·해상도, fps)는 처음과 같아야 합니다.

```bash
PYTHONPATH=src python -m leader_teleop.record \
	... (로봇·카메라·리더 인자 동일) ... \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.root=<로컬경로> --resume=true \
	--dataset.single_task='blue' \
	--dataset.num_episodes=25 --dataset.episode_time_s=1000 --dataset.reset_time_s=5 \
	--dataset.push_to_hub=false --display_data=true
```

`num_episodes`는 이번 실행에서 추가로 찍을 개수입니다.

### 에피소드 삭제

```bash
lerobot-edit-dataset --repo_id=<HF계정>/<데이터셋이름> --root=<로컬경로> \
	--operation.type=delete_episodes --operation.episode_indices='[0]'
```

제자리에서 수정되고 원본은 `<로컬경로>_old`로 백업됩니다. 나머지
에피소드는 0부터 다시 번호가 매겨집니다.

### 병합

`lerobot-train`은 데이터셋 하나만 받습니다(다중 데이터셋 기능이 이 버전에서
꺼져 있음). 여러 데이터셋으로 학습하려면 먼저 합칩니다.

```bash
lerobot-edit-dataset --operation.type=merge \
	--operation.repo_ids="['<HF계정>/<데이터셋A>', '<HF계정>/<데이터셋B>']" \
	--new_repo_id=<HF계정>/<병합이름> --new_root=<병합 로컬경로>
```

`--operation.roots`를 생략하면 각 입력을 `~/.cache/huggingface/lerobot/<repo_id>`에서
찾고, 없으면 허브에서 내려받습니다.

### 프롬프트 배정 (언어 조건부 학습용)

수집 때 자리표시자(`red`/`blue`)로 찍은 에피소드에 실제 문장을
배정합니다. 같은 행동에는 표현이 다른 문장을 여러 개 돌려 넣어 정책이
표현 차이에 강해지게 하고, 다른 행동은 색깔처럼 한 단어가 다른 문장 쌍으로
만들어 언어를 읽어야만 구분되게 합니다.

```bash
python - "<병합 로컬경로>" <<'PYEOF' > /tmp/episode_tasks.json
import glob, json, sys
import pandas as pd
root = sys.argv[1]
red = ['Put the red die on the yellow cloth.', 'Place the red cube on the yellow mat.',
       'Move the red object onto the yellow area.', 'Pick up the red die and set it on the yellow felt.',
       'Put the red block on the yellow cloth.', 'Place the red one on the yellow fabric.',
       'Take the red cube and put it on the yellow mat.', 'Move the red die to the yellow area.']
blue = [s.replace('red', 'blue') for s in red]
files = sorted(glob.glob(f'{root}/meta/episodes/**/*.parquet', recursive=True))
episodes = pd.concat(pd.read_parquet(f) for f in files).sort_values('episode_index')
tasks, counters = {}, {'red': 0, 'blue': 0}
for _, row in episodes.iterrows():
    color = 'red' if str(row['tasks'][0]).startswith('red') else 'blue'
    pool = red if color == 'red' else blue
    tasks[str(int(row['episode_index']))] = pool[counters[color] % len(pool)]
    counters[color] += 1
print(json.dumps(tasks))
print(f'red {counters["red"]}개, blue {counters["blue"]}개', file=sys.stderr)
PYEOF

lerobot-edit-dataset --repo_id=<HF계정>/<병합이름> --root=<병합 로컬경로> \
	--operation.type=modify_tasks \
	--operation.episode_tasks="$(cat /tmp/episode_tasks.json)"
```

적용 후 `info`에서 `Total task`가 문장 개수(예: 16)로 바뀌었는지 봅니다.
문장은 48토큰 안에 들어오게 한 줄로 짧게 씁니다.

### 허브 업로드

새 데이터셋을 처음 올릴 때:

```bash
python -c "from lerobot.datasets.lerobot_dataset import LeRobotDataset; LeRobotDataset('<HF계정>/<이름>', root='<로컬경로>').push_to_hub()"
```

수정한 데이터셋을 다시 올릴 때는 반드시 아래 방식을 씁니다.
`push_to_hub()`는 원격에 남은 옛 파일을 지우지 않아, 삭제·병합 뒤에 그대로
올리면 허브에 옛 메타·영상이 섞여 에피소드 수가 맞지 않게 됩니다.

```bash
hf upload <HF계정>/<이름> <로컬경로> . --repo-type dataset \
	--delete "data/**" --delete "meta/**" --delete "videos/**"
```

허브 데이터셋 삭제는 `hf repos delete <HF계정>/<이름> --repo-type dataset`
입니다. 로컬 폴더는 남으므로 필요하면 따로 지웁니다.

## 학습

### 공통 준비

```bash
hf auth login             # Write 권한 토큰 (모델·데이터셋 업로드). huggingface-cli는 hub 1.x에서 제거됨
wandb login               # https://wandb.ai/authorize 의 API 키
export HF_USER=<HF계정>
export TASK_NAME=<데이터셋이름>
```

`--wandb.enable=true`를 쓰려면 `wandb login`이 돼 있어야 합니다. 정책
설정의 `push_to_hub`가 기본으로 켜져 있어 `--policy.repo_id`가 없으면
시작 시점에 오류가 납니다. 올리지 않으려면 `--policy.push_to_hub=false`를
줍니다.

### SmolVLA (언어 조건부, 사전학습 모델 파인튜닝)

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/${TASK_NAME} \
  --policy.repo_id=${HF_USER}/${TASK_NAME}_smolvla \
  --policy.path=lerobot/smolvla_base \
  --policy.device=cuda \
  --output_dir=outputs/train/bi_so102/smolvla/${TASK_NAME} \
  --job_name=smolvla_bi_so102 \
  --steps=100_000 \
  --save_checkpoint=true \
  --save_freq=10_000 \
  --batch_size=16 \
  --num_workers=16 \
  --wandb.enable=true \
  --rename_map='{
    "observation.images.cam_top": "observation.images.camera1",
    "observation.images.cam_wrist_left": "observation.images.camera2",
    "observation.images.cam_wrist_right": "observation.images.camera3"
  }'
```

- `--rename_map`은 필수입니다. `smolvla_base`는 카메라 입력을
  `camera1/2/3`으로 기대하고, 사전학습 모델을 쓰면 이 이름이 데이터셋
  이름으로 덮어써지지 않습니다. 추론 때도 같은 매핑을 줍니다.
- 양팔 14차원 상태·행동은 SmolVLA가 32차원까지 패딩해 받으므로 그대로
  씁니다.
- 학습률 스케줄은 사전학습 프리셋대로 1,000스텝 워밍업 후 30,000스텝에
  걸쳐 코사인 감쇠하고, 그 뒤로는 바닥값(2.5e-6)으로 고정됩니다. 10만
  스텝 내내 감쇠시키려면 `--policy.scheduler_decay_steps=90_000`을 시작할
  때 줍니다. 이 값은 나중에 이어 학습할 때 바꿀 수 없습니다.
- 이 PC(RTX 5070 Ti Laptop)에서 배치 16 기준 스텝당 0.61초(로그
  `updt_s` 0.605~0.614), 5만 스텝 8시간 37분, `mem_gb` 6.0이었습니다. 10만
  스텝이면 약 17시간입니다.

### ACT (단일 태스크, 처음부터 학습)

ACT는 문장을 읽지 않으므로 행동 종류마다 데이터셋을 따로 두고 각각
학습합니다. 빨강·파랑이 섞인 병합본으로 학습하면 두 물체가 놓인 장면에서
어느 것을 집을지 정할 정보가 없습니다.

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/<빨강_데이터셋> \
  --policy.repo_id=${HF_USER}/<빨강_데이터셋>_act \
  --policy.type=act \
  --policy.device=cuda \
  --output_dir=outputs/train/bi_so102/act/<빨강_데이터셋> \
  --job_name=act_bi_so102_red \
  --steps=100_000 \
  --save_checkpoint=true \
  --save_freq=10_000 \
  --batch_size=8 \
  --num_workers=16 \
  --wandb.enable=true
```

`--policy.path` 대신 `--policy.type=act`이고, 입력 feature를 데이터셋에서
그대로 가져가므로 `--rename_map`이 필요 없습니다. 파랑은 데이터셋 이름만
바꿔 한 번 더 돌립니다.

### 이어 학습

체크포인트에 저장된 설정 파일을 지정하고 `--resume=true`를 줍니다.
바꿀 값(예: 총 스텝)만 덧붙이면 됩니다. 데이터 순서까지 끊긴 지점에서
이어집니다.

```bash
lerobot-train \
  --config_path=outputs/train/bi_so102/smolvla/${TASK_NAME}/checkpoints/last/pretrained_model/train_config.json \
  --resume=true --steps=200_000
```

같은 `--output_dir`에 `--resume=true` 없이 다시 실행하면 덮어쓰기 방지
오류(`Output directory ... already exists`)가 납니다. 실패한 실행이 남긴
빈 폴더면 지우고 다시 시작합니다.

### 학습 인자

공통(`lerobot-train` 최상위):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `--output_dir` | 자동(`outputs/train/날짜/시각_job`) | 체크포인트·로그 저장 폴더. 이미 있으면 `--resume=true` 없이는 시작 안 됨 |
| `--job_name` | 정책 타입 이름 | WandB 실행 이름·기본 출력 폴더명 |
| `--steps` | 100,000 | 총 학습 스텝. `100_000`처럼 밑줄 표기 가능 |
| `--batch_size` | 8 | 스텝당 샘플 수. GPU 메모리에 맞춰 조정 |
| `--num_workers` | 4 | 데이터 로더 프로세스 수(영상 디코딩). CPU 코어 수 이하로 |
| `--save_checkpoint` | true | 체크포인트 저장 여부 |
| `--save_freq` | 20,000 | 체크포인트 저장 간격(스텝). 마지막 스텝은 항상 저장 |
| `--save_checkpoint_to_hub` | false | `save_freq`마다 체크포인트를 허브에도 업로드 (`--policy.repo_id` 필요) |
| `--log_freq` | 200 | 로그 출력 간격(스텝) |
| `--seed` | 1000 | 난수 시드 |
| `--resume` | false | `--config_path`의 체크포인트에서 이어 학습 |
| `--config_path` | — | 불러올 `train_config.json` 경로 |
| `--rename_map` | `{}` | 데이터셋 관측 키 → 정책이 기대하는 키 매핑. 사전학습 정책에만 유효 |
| `--use_policy_training_preset` | true | 정책이 정의한 옵티마이저·스케줄러 프리셋 사용 |
| `--eval_steps` / `--dataset.eval_split` | 0 / 0.0 | 홀드아웃 에피소드로 검증 loss 계산 (둘 다 줘야 동작) |

데이터셋(`--dataset.*`):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `repo_id` | 필수 | 학습 데이터셋 (허브 이름) |
| `root` | 없음 | 로컬 폴더. 주면 허브에서 받지 않음. 없으면 `~/.cache/huggingface/lerobot/<repo_id>`가 있을 때 그것을 사용 |
| `episodes` | 전체 | 사용할 에피소드 인덱스 목록 (예: `'[0,1,2]'`) |
| `image_transforms.enable` | false | 학습 시 이미지 증강(밝기·대비·채도·색조·선명도·아핀) 적용 |
| `video_backend` | 자동 | 영상 디코더 (`torchcodec` / `pyav`) |
| `revision` | 없음 | 허브 데이터셋의 특정 리비전 |
| `streaming` | false | 다운로드 없이 스트리밍 로드 |

정책 공통(`--policy.*`):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `type` | — | 처음부터 학습할 정책 종류 (`act`, `smolvla`, `diffusion`, `pi0` …). `path`와 둘 중 하나 |
| `path` | — | 사전학습 체크포인트 (허브 이름 또는 로컬 `pretrained_model` 폴더). 파인튜닝 시작점 |
| `device` | 자동 | `cuda` / `cpu` / `mps` |
| `repo_id` | 없음 | 학습 결과 모델을 올릴 허브 저장소. `push_to_hub=true`면 필수 |
| `push_to_hub` | true | 학습 종료 시 최종 모델 업로드 |
| `private` | 없음 | 허브 저장소 비공개 여부 |
| `use_amp` | false | 혼합 정밀도 학습 (메모리 절약) |

SmolVLA(`--policy.*`, `smolvla_base` 프리셋 기준):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `chunk_size` / `n_action_steps` | 50 / 50 | 한 번 추론으로 예측하는 행동 개수 / 그중 실행하는 개수 |
| `freeze_vision_encoder` | true | 비전 인코더 고정 |
| `train_expert_only` | true | 액션 전문가(expert)만 학습, VLM 백본 고정 |
| `train_state_proj` | true | 상태 투영층 학습 (로봇 차원이 달라도 됨) |
| `optimizer_lr` | 1e-4 | 최고 학습률 |
| `scheduler_warmup_steps` | 1,000 | 워밍업 스텝 |
| `scheduler_decay_steps` | 30,000 | 코사인 감쇠 구간. 이후 `scheduler_decay_lr`(2.5e-6)로 고정 |
| `tokenizer_max_length` | 48 | 문장 최대 토큰 수 |
| `empty_cameras` | 0 | 카메라가 사전학습보다 적을 때 빈 카메라로 채우는 개수 |
| `max_state_dim` / `max_action_dim` | 32 / 32 | 상태·행동 패딩 차원 |

ACT(`--policy.*`):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `chunk_size` / `n_action_steps` | 100 / 100 | 예측 행동 개수 / 실행 개수 |
| `vision_backbone` | resnet18 | 이미지 인코더 |
| `pretrained_backbone_weights` | ImageNet | 인코더 초기 가중치 (`null`이면 무작위) |
| `use_vae` / `kl_weight` | true / 10.0 | VAE 목적함수 사용 여부와 KL 가중치 |
| `optimizer_lr` / `optimizer_lr_backbone` | 1e-5 / 1e-5 | 학습률 (본체 / 백본) |
| `dropout` | 0.1 | 트랜스포머 드롭아웃 |
| `temporal_ensemble_coeff` | 없음 | 추론 시 행동 시간 앙상블 계수 (주면 `n_action_steps=1` 필요) |

WandB(`--wandb.*`):

| 인자 | 기본값 | 역할 |
|------|--------|------|
| `enable` | false | 로깅 켜기 |
| `project` / `entity` | `lerobot` / 없음 | 프로젝트·팀 이름 |
| `disable_artifact` | false | `true`면 체크포인트 파일(회당 1GB급)을 WandB에 올리지 않고 로그만 |
| `notes` / `run_id` | 없음 | 실행 메모 / 이어 붙일 실행 ID |
| `mode` | online | `offline`이면 나중에 동기화 |

## 추론

`lerobot-rollout`을 이 레포의 로봇 타입이 등록된 상태로 실행하는
진입점입니다(`lerobot-rollout` 명령을 직접 쓰면 `bi_so10x_client` 같은
타입을 모릅니다). 파이 host 구성이면 `bi_so10x_client`, 직접 연결이면
`bi_so10x_follower`를 씁니다.

### SmolVLA

```bash
PYTHONPATH=src python -m leader_teleop.rollout \
	--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--policy.path=outputs/train/bi_so102/smolvla/${TASK_NAME}/checkpoints/last/pretrained_model \
	--rename_map='{"observation.images.cam_top": "observation.images.camera1", "observation.images.cam_wrist_left": "observation.images.camera2", "observation.images.cam_wrist_right": "observation.images.camera3"}' \
	--task="Put the red die on the yellow cloth." \
	--duration=60 --fps=10 --policy.device=cuda
```

### ACT

```bash
PYTHONPATH=src python -m leader_teleop.rollout \
	--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \
	--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_left": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "cam_wrist_right": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--policy.path=outputs/train/bi_so102/act/<빨강_데이터셋>/checkpoints/last/pretrained_model \
	--duration=60 --fps=30 --policy.device=cuda
```

- `--rename_map`은 학습 때 준 것과 같아야 합니다. 빼면 `Visual feature
  mismatch`로 시작하지 않습니다. ACT는 필요 없습니다.
- 직접 연결(`bi_so10x_follower`)로 추론할 때 머리 모터가 달린 구성이면
  `--robot.has_head_motors=true`를 줍니다. 기본값은 머리 없음이며,
  teleoperate/record/host는 `--camera_head.mode`로 이 값을 자동 설정하지만
  rollout은 lerobot이 로봇을 직접 만들어 그 단계를 거치지 않습니다.
- `--task`는 SmolVLA가 읽는 문장입니다. ACT는 무시합니다. 학습에 넣지
  않은 표현으로도 시험해 보면 언어 일반화를 확인할 수 있습니다.
- `--policy.path`에는 허브 이름(`${HF_USER}/${TASK_NAME}_smolvla`)이나
  특정 체크포인트(`checkpoints/050000/pretrained_model`)도 됩니다.
- SmolVLA 동기 추론은 이 PC에서 5~6Hz라 `--fps=10`으로 둡니다. 한 번
  추론이 행동 50개를 내놓는 청킹 구조라 그 속도로도 동작이 이어집니다.
  ACT는 가벼워 30fps가 됩니다.
- 종료는 `Ctrl+C` 또는 `--duration` 만료입니다. 기본으로 시작 자세로
  되돌린 뒤 끝납니다(`--return_to_initial_position=false`로 끌 수 있음).
- 파이에서 직접 실행(`bi_so10x_follower`, `--policy.device=cpu`)하면 ACT는
  청크(행동 100개, 3.3초 분량) 경계마다 다음 추론이 끝날 때까지 멈춥니다.
  파이 CPU의 1회 추론이 수 초라 "목표 도달 → 수 초 정지 → 그리퍼 닫기"
  처럼 보이며, 카메라 읽기 부하로 루프가 30fps를 못 채워 전체가 느려집니다.
  고장이 아니라 청킹 구조입니다. 매끄러운 동작은 PC GPU 추론(host/client
  구성)으로 얻고, ACT에는 실행 중 다음 청크를 미리 계산하는
  `--inference.type=rtc`가 적용되지 않습니다(SmolVLA·π0 계열만 지원).

## 카메라 헤드 (선택)

머리 pan/tilt 모터가 있는 구성이면 `--camera_head.mode`로 제어합니다.
기본은 `none`이며 이때는 머리 모터를 검색하지 않습니다.

| 모드 | 동작 |
|------|------|
| `none` (기본) | 머리 미장착 구성. 머리 키가 데이터셋에서도 빠짐 |
| `fixed` | 시작 시 고정각(기본 홈각)으로 이동 후 유지 |
| `keyboard` | `a/d`=pan, `w/s`=tilt, `h`=홈 (record 방향키와 안 겹침) |

## 홈포즈 조정

종료 시 복귀하는 홈포즈 관절각은 `src/leader_teleop/config.py`의
`HomeReturnConfig`에 있습니다. 내 로봇에 맞는 값은 이렇게 잽니다.

```bash
PYTHONPATH=src python -m leader_teleop.scripts.capture_home_pose \
	--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1
```

토크가 풀린 상태에서 원하는 자세로 팔을 잡으면 config에 붙여넣을 값이
출력됩니다.

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
| MuJoCo 시뮬레이션 실습 | `mujoco_sim/` (별도 README) |
| 하드웨어 점검 / 홈포즈 캡처 | `scripts/check_robot.py` / `scripts/capture_home_pose.py` |
| 단위 테스트 | `tests/` (`PYTHONPATH=src python -m leader_teleop.tests.<모듈>`) |

## 안전 주의

- 실행 전 로봇 주변을 치웁니다. `Ctrl+C`를 바로 누를 수 있게 터미널에
  손을 둡니다.
- 전압 오류가 반복되면 전원부터 확인합니다. 리더암은 5V, 팔로워암은
  12V입니다. 데이지체인 커넥터도 다시 꽂아 봅니다.
- 리더암과 팔로워암의 좌/우를 바꿔 꽂으면 팔이 교차로 움직입니다.
  실행 직후에는 천천히 움직여 좌/우 대응부터 확인합니다.
- 추론 첫 실행은 `--duration`을 짧게 주고 손을 `Ctrl+C`에 둔 채 지켜봅니다.

## 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| `motor check failed ... Full found motor list: {}` | 서보 전원(12V) 없음, 포트가 다른 장치, 같은 포트를 쓰는 다른 프로세스 | 12V 어댑터 연결, `check_robot`으로 포트 재확인, `ps aux \| grep leader_teleop` |
| 캘리브레이션 중 `sync_read` 오류로 종료 | 관절을 움직이는 순간 전원이 끊김(배터리 출력 부족), 케이블 접촉 | 어댑터 전원으로 교체, 데이지체인 재연결 |
| host가 `Failed to sync read ... no status packet`으로 종료 | 첫 명령 순간 전류 피크로 버스가 잠깐 침묵, 전원 여유 부족 | 어댑터 전류 여유 확인, 연결 전 리더 자세를 팔로워와 맞춤. 코드는 연속 한도 안에서 자동 재시도 |
| 텔레옵 반응이 느리고 팔로워가 뒤늦게 따라옴 | host에 `--robot.max_relative_target`가 켜져 있음 | 옵션 제거 (관절 속도 상한과 추가 읽기로 루프가 느려짐) |
| `Couldn't find a choice class for 'opencv'` | 카메라 설정 클래스 미등록 (구버전 코드) | `git pull` |
| `ModuleNotFoundError: No module named 'zmq'` | PC 환경에 pyzmq 없음 | `pip install -r requirements.txt` |
| `Timed out waiting for frame from camera` | USB 대역폭 부족(무압축) 또는 첫 프레임 지연 | 카메라 설정에 `"fourcc": "MJPG", "warmup_s": 3`, 카메라를 USB 3.0/2.0 포트에 분산 |
| `Host is not sending camera 'top'` | host 명령의 카메라 줄이 빠짐(줄 끝 `\` 누락) 또는 이름 불일치 | host 시작 로그의 `cameras:`에 3대가 보이는지 확인 |
| `resume() requires an explicit 'root'` | `--resume=true`에 `--dataset.root` 없음 | 로컬 폴더 경로를 `--dataset.root`로 지정 |
| 병합본 `info`는 정상인데 학습·메타가 이상함 | 허브에 옛 파일이 남은 데이터셋을 병합 | `hf upload ... --delete`로 허브 정리 후 재병합, 메타 행 수 검증 |
| `'repo_id' argument missing` | `policy.push_to_hub`가 기본 true | `--policy.repo_id=<계정>/<모델>` 또는 `--policy.push_to_hub=false` |
| `No API key configured. Use wandb login` | WandB 미로그인 | `wandb login` 또는 `--wandb.enable=false` |
| `'transformers' is required but not installed` | SmolVLA 의존성 없음 | `pip install -r requirements.txt` (smolvla extra 포함) |
| `Output directory ... already exists` | 이전 실행 폴더 잔존 | 체크포인트가 없으면 폴더 삭제, 있으면 `--resume=true` |
| `Visual feature mismatch between policy and robot` | 추론에 `--rename_map` 누락 | 학습 때와 같은 `--rename_map` 추가 |
| 여러 줄 명령의 뒷부분 인자가 무시됨 | 줄 끝 `\` 뒤에 공백이 있거나 `\`가 빠짐 | 각 줄 끝이 `\`로 끝나는지 확인 |
